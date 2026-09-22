"""普通抓取节点：把 2D 检测 + 深度图转换成抓取计划与候选列表。

在系统中的位置
--------------
本节点是「检测 → 抓取计划」的胶水层，属于视觉包的遗留/常规抓取路径：
它本身不做几何推导，而是把工作交给外部的普通抓取算法实现
（由 ``ordinary_grasp.root`` 指向的目录提供 ``utils.ordinary_grasp`` 等模块），
再把结果整理成统一消息发布。**本节点只产生计划，不执行运动**；
真正的规划、碰撞检查与执行门控在下游运动/执行环节，因此这里不做安全授权。

话题与回调模型
--------------
* 订阅 ``ordinary_grasp.input_depth_topic``：深度图（传感器数据 QoS），
  回调只把最新一帧缓存进 ``latest_depth_mm``，不触发计算；
* 订阅 ``ordinary_grasp.input_detections_topic``：检测数组（深度 10），
  **该回调才是计算触发点**，使用最近缓存的深度图；
* 发布 ``ordinary_grasp.output_topic``（抓取计划）、
  ``ordinary_grasp.candidates_topic``（候选数组）、以及仅在计划有效时发布的
  ``ordinary_grasp.pregrasp_pose_topic`` / ``ordinary_grasp.grasp_pose_topic``
  （两个 PoseStamped，便于 RViz 直接显示）。
* 若在收到任何深度帧之前就有检测到达，检测回调直接返回（不做无深度的伪抓取）。

坐标系与安全相关约定
--------------------
* 深度值单位 mm，``0`` 表示无效像素；
* 最终位姿的 ``frame_id`` 由 ``ordinary_grasp.output_frame_id`` 指定，
  必须与深度相机光学坐标系一致，否则下游变换会整体偏掉；
* ``ordinary_grasp.root`` 为空时构造期直接抛 ``RuntimeError``（快速失败），
  避免节点静默地永远不出候选；
* 深度质量门控（``depth_quality.*``）会在生成候选前剔除深度不可信的检测。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rebotarm_msgs.msg import Detection2DArray, GraspCandidateArray, GraspPlan

from .converters.ordinary_grasp_adapter import CameraIntrinsics, plan_and_candidates_from_detections_and_depth
from .depth_quality import DepthQualityConfig


def depth_image_to_array(msg: Image) -> np.ndarray:
    """把深度图像消息解码成 ``(height, width)`` 的 uint16 毫米数组。

    仅接受 ``mono16`` / ``16UC1`` 两种 16 位单通道编码，其它编码抛 ``ValueError``
    （宁可拒帧也不做有损换算）。返回值是 ``msg.data`` 的只读视图再 reshape，
    因此长度必须恰好等于 ``height * width``；深度 0 表示无效像素。
    """
    if msg.encoding not in ("mono16", "16UC1"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    depth = np.frombuffer(msg.data, dtype=np.uint16)
    return depth.reshape((msg.height, msg.width))


class OrdinaryGraspNode(Node):
    """普通抓取节点：深度 + 检测 → 抓取计划/候选。

    生命周期：构造期声明并读取全部参数（含深度质量配置）、做必需的快速失败校验、
    建立发布者与订阅者；此后由两个回调驱动，单线程执行器下无并发问题。
    ``latest_depth_mm`` 是「最近一帧深度」缓存，不做时间戳配对，
    检测与深度的时间偏差由上游发布频率保证。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_ordinary_grasp_node")
        # 输入/输出话题（保持默认值即可与视觉链路其余节点对接）。
        self.declare_parameter("ordinary_grasp.input_detections_topic", "/grasp/detections")
        self.declare_parameter("ordinary_grasp.input_depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("ordinary_grasp.output_topic", "/grasp/plan")
        self.declare_parameter("ordinary_grasp.candidates_topic", "/grasp/candidates")
        self.declare_parameter("ordinary_grasp.pregrasp_pose_topic", "/grasp/pregrasp_pose")
        self.declare_parameter("ordinary_grasp.grasp_pose_topic", "/grasp/grasp_pose")
        # 外部普通抓取算法代码目录，必填；为空时构造期即报错。
        self.declare_parameter("ordinary_grasp.root", "")
        # 发布位姿的参考坐标系，单位 m 的深度反投影即在该坐标系下完成，
        # 必须与深度图的 frame_id 一致（默认取深度相机光学坐标系）。
        self.declare_parameter("ordinary_grasp.output_frame_id", "camera_depth_frame")
        # 深度取值的分位数，取值 (0, 1)。0.75 表示在检测框内取 75 分位深度作为目标距离，
        # 比直接取最小值更稳健（避免抓到前景噪点），调大→更贴近背景/更保守。
        self.declare_parameter("ordinary_grasp.depth_quantile", 0.75)
        # 预抓取点沿 TCP 接近轴退开的距离，单位 m；与运动侧的接近偏移保持一致更安全。
        self.declare_parameter("ordinary_grasp.pregrasp_offset_m", 0.08)
        # 相机内参（像素单位）。fx/fy 为焦距，cx/cy 为主点；
        # 默认值为占位值，真机必须用标定结果覆盖，否则反投影出的三维点会整体偏移。
        self.declare_parameter("ordinary_grasp.fx", 500.0)
        self.declare_parameter("ordinary_grasp.fy", 500.0)
        self.declare_parameter("ordinary_grasp.cx", 640.0)
        self.declare_parameter("ordinary_grasp.cy", 360.0)
        # 深度质量门控：过滤 0 值/噪声/深度跨度太大的检测，避免用坏深度生成抓取。
        self.declare_parameter("depth_quality.enabled", True)
        # 检测框内有效深度像素数下限（个）；太小的 ROI 统计不可信。
        self.declare_parameter("depth_quality.min_valid_pixels", 80)
        # 有效深度像素占 ROI 面积的比例下限，取值 [0, 1]。
        self.declare_parameter("depth_quality.min_valid_ratio", 0.20)
        # 允许的中位深度范围，单位 m；超出即认为目标太近/太远，本帧不抓。
        self.declare_parameter("depth_quality.min_depth_m", 0.15)
        self.declare_parameter("depth_quality.max_depth_m", 1.20)
        # 深度稳健离散度上限（MAD，单位 m）：衡量 ROI 内深度是否够「平」。
        self.declare_parameter("depth_quality.max_depth_mad_m", 0.025)
        # 深度跨度上限（95 分位 - 5 分位，单位 m）：防止把斜坡/多物体当成一个目标。
        self.declare_parameter("depth_quality.max_depth_span_m", 0.080)
        # 中心窗口边长（像素，奇数更对称），用于单独检查目标中心区域是否有效。
        self.declare_parameter("depth_quality.center_window_px", 9)
        # 中心窗口内有效深度比例下限，取值 [0, 1]；中心空洞常见于反光/透明目标。
        self.declare_parameter("depth_quality.min_center_valid_ratio", 0.30)
        # 是否用外部开关强制覆盖 enabled：launch 以此在启动时统一开关深度门控。
        self.declare_parameter("depth_quality.override_enabled", False)
        # 覆盖后的取值（True 开 / False 关）；仅当 override_enabled 为真时生效。
        self.declare_parameter("depth_quality.override_value", True)

        self.input_detections_topic = str(
            self.get_parameter("ordinary_grasp.input_detections_topic").value
        )
        self.input_depth_topic = str(self.get_parameter("ordinary_grasp.input_depth_topic").value)
        self.output_topic = str(self.get_parameter("ordinary_grasp.output_topic").value)
        self.candidates_topic = str(self.get_parameter("ordinary_grasp.candidates_topic").value)
        self.pregrasp_pose_topic = str(
            self.get_parameter("ordinary_grasp.pregrasp_pose_topic").value
        )
        self.grasp_pose_topic = str(self.get_parameter("ordinary_grasp.grasp_pose_topic").value)
        ordinary_grasp_root = str(self.get_parameter("ordinary_grasp.root").value).strip()
        # 快速失败：缺少算法目录时节点无法产出任何候选，与其静默运行不如启动即报错。
        if not ordinary_grasp_root:
            raise RuntimeError(
                "ordinary_grasp.root is required when the legacy ordinary grasp node is enabled"
            )
        # expanduser 支持以 ~ 开头的路径，方便本机调试配置。
        self.ordinary_grasp_root = Path(ordinary_grasp_root).expanduser()
        self.output_frame_id = str(self.get_parameter("ordinary_grasp.output_frame_id").value)
        self.depth_quantile = float(self.get_parameter("ordinary_grasp.depth_quantile").value)
        self.pregrasp_offset_m = float(self.get_parameter("ordinary_grasp.pregrasp_offset_m").value)
        self.intrinsics = CameraIntrinsics(
            fx=float(self.get_parameter("ordinary_grasp.fx").value),
            fy=float(self.get_parameter("ordinary_grasp.fy").value),
            cx=float(self.get_parameter("ordinary_grasp.cx").value),
            cy=float(self.get_parameter("ordinary_grasp.cy").value),
        )
        depth_quality_enabled = bool(self.get_parameter("depth_quality.enabled").value)
        if bool(self.get_parameter("depth_quality.override_enabled").value):
            # 覆盖生效后把结果回写进 depth_quality.enabled 参数本身，
            # 这样 `ros2 param get` 看到的实际状态与运行行为一致（避免参数显示与真实门控不一致）。
            depth_quality_enabled = bool(self.get_parameter("depth_quality.override_value").value)
            self.set_parameters(
                [Parameter("depth_quality.enabled", Parameter.Type.BOOL, depth_quality_enabled)]
            )
        self.depth_quality_config = DepthQualityConfig(
            enabled=depth_quality_enabled,
            min_valid_pixels=int(self.get_parameter("depth_quality.min_valid_pixels").value),
            min_valid_ratio=float(self.get_parameter("depth_quality.min_valid_ratio").value),
            min_depth_m=float(self.get_parameter("depth_quality.min_depth_m").value),
            max_depth_m=float(self.get_parameter("depth_quality.max_depth_m").value),
            max_depth_mad_m=float(self.get_parameter("depth_quality.max_depth_mad_m").value),
            max_depth_span_m=float(self.get_parameter("depth_quality.max_depth_span_m").value),
            center_window_px=int(self.get_parameter("depth_quality.center_window_px").value),
            min_center_valid_ratio=float(self.get_parameter("depth_quality.min_center_valid_ratio").value),
        )
        # 最近一帧深度（uint16 毫米数组）；None 表示尚未收到任何深度图。
        self.latest_depth_mm = None

        self.plan_pub = self.create_publisher(GraspPlan, self.output_topic, 10)
        self.candidates_pub = self.create_publisher(GraspCandidateArray, self.candidates_topic, 10)
        self.pregrasp_pose_pub = self.create_publisher(PoseStamped, self.pregrasp_pose_topic, 10)
        self.grasp_pose_pub = self.create_publisher(PoseStamped, self.grasp_pose_topic, 10)
        # 深度图用传感器数据 QoS（尽力而为、只保留最新），与相机发布端保持一致；
        # 检测结果用深度 10 的可靠 QoS，保证不漏帧。
        self.depth_subscription = self.create_subscription(
            Image,
            self.input_depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.detections_subscription = self.create_subscription(
            Detection2DArray,
            self.input_detections_topic,
            self._on_detections,
            10,
        )
        self.get_logger().info(
            "ordinary grasp node ready: "
            f"detections={self.input_detections_topic}, depth={self.input_depth_topic}, "
            f"output={self.output_topic}, candidates={self.candidates_topic}, "
            f"depth_quality_enabled={self.depth_quality_config.enabled}, root={self.ordinary_grasp_root}"
        )

    def _on_depth(self, msg: Image) -> None:
        # 只缓存不解算：真正的计算在检测回调里做，避免每帧深度都跑一遍。
        try:
            self.latest_depth_mm = depth_image_to_array(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_detections(self, msg: Detection2DArray) -> None:
        # 没有深度就不做抓取：绝不用零深度或上帧深度凑合。
        if self.latest_depth_mm is None:
            return
        try:
            plan, candidates = plan_and_candidates_from_detections_and_depth(
                msg,
                self.latest_depth_mm,
                self.intrinsics,
                ordinary_grasp_root=self.ordinary_grasp_root,
                output_frame_id=self.output_frame_id,
                depth_quantile=self.depth_quantile,
                pregrasp_offset_m=self.pregrasp_offset_m,
                depth_quality_config=self.depth_quality_config,
            )
        except Exception as exc:
            # 单帧失败不影响后续帧：记录后直接丢弃本帧（不发布过期计划）。
            self.get_logger().warn(f"ordinary grasp failed: {type(exc).__name__}: {exc}")
            return
        self.candidates_pub.publish(candidates)
        self.plan_pub.publish(plan)
        # 只在计划有效时发布位姿：无效计划里的位姿无意义，发出去会被下游误用。
        if plan.valid:
            self.pregrasp_pose_pub.publish(self._pose_stamped(plan, plan.pregrasp_pose))
            self.grasp_pose_pub.publish(self._pose_stamped(plan, plan.grasp_pose))

    @staticmethod
    def _pose_stamped(plan: GraspPlan, pose) -> PoseStamped:
        """把 ``GraspPlan`` 里的位姿包装成带 header 的 PoseStamped。

        直接复用计划的 header（时间戳与 frame_id），保证位姿、计划与候选三者在
        同一帧同一坐标系下，便于 RViz 叠加显示与下游做时间一致性判断。
        """
        msg = PoseStamped()
        msg.header = plan.header
        msg.pose = pose
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrdinaryGraspNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
