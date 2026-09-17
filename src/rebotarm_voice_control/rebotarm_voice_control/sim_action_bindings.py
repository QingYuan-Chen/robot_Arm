# 仿真动作绑定层：把工具调用里的通用参数字典翻译成具体 ROS 动作的 Goal 消息，
# 并在此处完成第一道参数校验（动作类型名、轴向、坐标系、必填字段）。
#
# 本层只做翻译与校验，不发送任何目标、不接触硬件，因此可以脱离运行中的节点
# 单独测试：动作类型与 Pose 工厂都以参数注入，生产路径才走延迟 import。
#
# 动作名 → 消息类型的映射约定（按动作名后缀匹配）：
# - /move_relative                → MoveRelative
# - /move_to_pose                 → MoveToPose
# - /pick_object、/place_object   → ExecuteGrasp
#
# 之所以延迟 import：让本模块在缺少部分依赖的环境里也能被导入，便于纯逻辑测试。

from __future__ import annotations

from typing import Any, Callable

from .models import SafetyViolationError


# 构造一个空 Pose 消息的工厂函数类型（延迟到调用时才真正 import）
PoseFactory = Callable[[], Any]


def _default_pose_factory() -> Any:
    """生产环境使用的 Pose 工厂：真实构造一个空的位姿消息对象。"""
    try:
        from geometry_msgs.msg import Pose
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("geometry_msgs is required for sim action bindings") from exc
    return Pose()


def _load_move_to_pose_type() -> Any:
    """加载绝对位姿运动动作的消息类型。"""
    try:
        from rebotarm_msgs.action import MoveToPose
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rebotarm_msgs.action.MoveToPose is required") from exc
    return MoveToPose


def _load_move_relative_type() -> Any:
    """加载相对运动动作的消息类型。"""
    try:
        from rebotarm_msgs.action import MoveRelative
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rebotarm_msgs.action.MoveRelative is required") from exc
    return MoveRelative


def _load_execute_grasp_type() -> Any:
    """加载抓取执行动作的消息类型（抓取与放置共用）。"""
    try:
        from rebotarm_msgs.action import ExecuteGrasp
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rebotarm_msgs.action.ExecuteGrasp is required") from exc
    return ExecuteGrasp


def _as_pose(pose_data: dict[str, Any], pose_factory: PoseFactory) -> Any:
    """把字典形式的位姿数据填充成 Pose 消息。

    位置单位米，姿态为四元数 (x, y, z, w)；缺失字段按零位处理，其中
    ``w`` 缺省为 1.0 表示无旋转的单位四元数——若也缺省为 0 会得到非法的
    全零四元数，下游求逆/归一化会失败。
    """
    pose = pose_factory()
    position = pose_data.get("position", {})
    orientation = pose_data.get("orientation", {})
    pose.position.x = float(position.get("x", 0.0))
    pose.position.y = float(position.get("y", 0.0))
    pose.position.z = float(position.get("z", 0.0))
    pose.orientation.x = float(orientation.get("x", 0.0))
    pose.orientation.y = float(orientation.get("y", 0.0))
    pose.orientation.z = float(orientation.get("z", 0.0))
    pose.orientation.w = float(orientation.get("w", 1.0))
    return pose


def build_move_to_pose_goal(
    move_to_pose_type: Any,
    pose_factory: PoseFactory,
    goal: dict[str, Any],
) -> Any:
    """构造绝对位姿运动目标。

    ``goal`` 必须含 ``pose`` 字典（缺失或类型不对直接拒绝，避免发出空目标）；
    ``duration`` 为期望运动时长，单位秒，缺省 2.0 秒——给得越小速度越快，
    因此这里保留一个偏保守的默认值。
    """
    pose_data = goal.get("pose")
    if not isinstance(pose_data, dict):
        raise SafetyViolationError("move_to_pose sim goal requires pose")
    request = move_to_pose_type.Goal()
    request.target_pose = _as_pose(pose_data, pose_factory)
    request.duration = float(goal.get("duration", 2.0))
    return request


def build_move_relative_goal(
    move_relative_type: Any,
    goal: dict[str, Any],
) -> Any:
    """构造相对运动目标（沿指定轴平移）。

    - ``axis``：只允许 x、y、z，其它值（含空串）直接拒绝；
    - ``distance_m``：位移量，单位米，缺省 0.0；具体上限由执行侧（仿真节点）
      按 0.05 m 再校验一次，这里不做幅度限制；
    - ``frame_id``：位移参考坐标系，缺省 base_link（机械臂基座系，最安全）；
    - ``speed_scale``：速度缩放系数，缺省 0.2，即按额定速度的 20% 低速运动。
    """
    axis = str(goal.get("axis", ""))
    if axis not in {"x", "y", "z"}:
        raise SafetyViolationError("move_relative sim goal axis must be x, y, or z")
    request = move_relative_type.Goal()
    request.axis = axis
    request.distance_m = float(goal.get("distance_m", 0.0))
    request.frame_id = str(goal.get("frame_id", "base_link"))
    request.speed_scale = float(goal.get("speed_scale", 0.2))
    return request


def build_execute_grasp_goal(
    execute_grasp_type: Any,
    pose_factory: PoseFactory,
    goal: dict[str, Any],
) -> Any:
    """构造抓取/放置目标。

    目标描述方式二选一（也可同时提供，由下游决定优先级）：
    - 按标签抓取：提供 ``label``（或别名 ``target_label``），置 ``use_label``；
    - 按位姿抓取：提供 ``pose`` 字典，置 ``use_pose``。

    两者都没有时抛错——空目标会让下游去抓未知物体，必须在入口拦住。
    """
    request = execute_grasp_type.Goal()
    label = str(goal.get("label", "") or goal.get("target_label", ""))
    pose_data = goal.get("pose")
    if label:
        request.target_label = label
        request.use_label = True
    if isinstance(pose_data, dict):
        request.target_pose = _as_pose(pose_data, pose_factory)
        request.use_pose = True
    if not request.use_label and not request.use_pose:
        raise SafetyViolationError("execute_grasp sim goal requires label or pose")
    return request


def resolve_sim_action_type(
    action_name: str,
    move_relative_type: Any | None = None,
    move_to_pose_type: Any | None = None,
    execute_grasp_type: Any | None = None,
) -> Any:
    """按动作名后缀返回对应的消息类型。

    显式传入的类型优先于延迟加载，便于测试时用假类型替换；无法识别动作名时
    抛 ``SafetyViolationError``，宁可不发也不猜。
    """
    if action_name.endswith("/move_relative"):
        return move_relative_type or _load_move_relative_type()
    if action_name.endswith("/move_to_pose"):
        return move_to_pose_type or _load_move_to_pose_type()
    if action_name.endswith("/pick_object") or action_name.endswith("/place_object"):
        return execute_grasp_type or _load_execute_grasp_type()
    raise SafetyViolationError(f"unsupported sim action: {action_name}")


def build_sim_goal(
    action_name: str,
    goal: dict[str, Any],
    move_relative_type: Any | None = None,
    move_to_pose_type: Any | None = None,
    execute_grasp_type: Any | None = None,
    pose_factory: PoseFactory | None = None,
) -> Any:
    """动作目标构造的统一入口：先按动作名分派，再委托给具体构造函数。

    供 ROS 动作传输层作为 ``goal_builder`` 回调使用；分派规则与
    ``resolve_sim_action_type`` 保持一致。
    """
    pose_factory = pose_factory or _default_pose_factory
    if action_name.endswith("/move_relative"):
        return build_move_relative_goal(
            move_relative_type or _load_move_relative_type(),
            goal,
        )
    if action_name.endswith("/move_to_pose"):
        return build_move_to_pose_goal(
            move_to_pose_type or _load_move_to_pose_type(),
            pose_factory,
            goal,
        )
    if action_name.endswith("/pick_object") or action_name.endswith("/place_object"):
        return build_execute_grasp_goal(
            execute_grasp_type or _load_execute_grasp_type(),
            pose_factory,
            goal,
        )
    raise SafetyViolationError(f"unsupported sim action: {action_name}")
