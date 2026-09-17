"""位姿预览求解器：复用机械臂 Python SDK 的运动学做纯计算预览。

本模块只做正/逆运动学计算，不建立 ROS 连接、不发送任何控制指令、不接触电机，
因此可以在任意进程里安全调用；真正的执行链路由运动执行节点与控制器负责。

对外提供两个能力：

- :meth:`PosePreviewSolver.solve_pose`：目标位姿（基坐标系，位置 m、RPY rad）→ 关节
  角（rad），把当前关节角作为迭代种子；
- :meth:`PosePreviewSolver.compute_pose_target`：当前关节角（rad）→ 末端位姿
  （RPY rad），用于把关节预览回显成位姿。

SDK 位置约定：:func:`ensure_sdk_on_syspath` 依次在 ``third_party/reBotArm_control_py``、
``sdk/reBotArm_control_py`` 及工作区同级目录 ``reBotArm_control_py-main`` 中查找，
命中后插入 ``sys.path``，因此该 import 必须延迟到查找完成之后执行。查找失败会抛出
``FileNotFoundError``，由调用方决定是否降级为「无位姿预览能力」。
"""

from __future__ import annotations

import sys
from pathlib import Path

from .command_models import PoseTarget, PreviewSolveResult


def _sdk_candidates(workspace_root: Path) -> list[Path]:
    """列出 SDK 仓库可能所在的目录，按优先级从高到低排列。"""
    return [
        workspace_root / "third_party" / "reBotArm_control_py",
        workspace_root / "sdk" / "reBotArm_control_py",
        workspace_root.parent / "reBotArm_control_py-main",
    ]


def ensure_sdk_on_syspath(workspace_root: Path) -> Path:
    """确保 SDK 可被 import，返回实际命中的 SDK 根目录。

    判定条件是目录下存在 ``reBotArm_control_py`` 包目录；命中后把它插到
    ``sys.path`` 最前面（幂等，重复调用不会重复插入）。全部候选都不存在时抛
    ``FileNotFoundError``，异常文本中列出所有已尝试的路径便于排查部署问题。
    """
    for root in _sdk_candidates(workspace_root):
        if (root / "reBotArm_control_py").is_dir():
            root_str = str(root)
            if root_str not in sys.path:
                sys.path.insert(0, root_str)
            return root
    candidates = "\n".join(f"  - {path}" for path in _sdk_candidates(workspace_root))
    raise FileNotFoundError(
        "Cannot find reBotArm_control_py for preview solving. Expected one of:\n"
        f"{candidates}"
    )


class PosePreviewSolver:
    """纯预览求解器：复用 Python SDK 的运动学，绝不驱动硬件。

    构造时定位并导入 SDK、加载机器人模型、缓存模型数据与末端帧 id；每次求解都在
    缓存的 ``_data`` 上复用，避免反复分配。实例不是线程安全的：如果多个线程共享
    同一个求解器，需要调用方自行加锁。
    """

    def __init__(self, workspace_root: Path, end_frame_name: str = "end_link") -> None:
        ensure_sdk_on_syspath(workspace_root)

        from reBotArm_control_py.kinematics import (  # pylint: disable=import-outside-toplevel
            get_end_effector_frame_id,
            load_robot_model,
            pos_rot_to_se3,
            solve_ik_with_retry,
        )

        self._load_robot_model = load_robot_model
        self._get_end_effector_frame_id = get_end_effector_frame_id
        self._pos_rot_to_se3 = pos_rot_to_se3
        self._solve_ik_with_retry = solve_ik_with_retry

        self._model = self._load_robot_model()
        self._data = self._model.createData()
        self._end_frame_id = self._get_end_effector_frame_id(self._model)
        self._end_frame_name = end_frame_name

    def solve_pose(
        self,
        pose_target: PoseTarget,
        seed_positions: tuple[float, ...],
        joint_names: tuple[str, ...],
    ) -> PreviewSolveResult:
        """把目标位姿反解为关节角。

        位置单位 m、姿态为固定轴 RPY（rad），二者先合成 4x4 齐次变换；``seed_positions``
        为当前关节角（rad），同时作为数值迭代初值与失败时的回退值。``joint_names``
        在本实现中不参与计算（关节顺序由 SDK 模型固定），因此显式丢弃。求解失败不抛
        异常，而是返回 ``success=False`` 并把当前关节角原样回填，交由上层判定为
        「不可达预览」。
        """
        del joint_names
        import numpy as np  # pylint: disable=import-outside-toplevel

        target = self._pos_rot_to_se3(
            np.array([pose_target.x, pose_target.y, pose_target.z], dtype=float),
            roll=pose_target.roll,
            pitch=pose_target.pitch,
            yaw=pose_target.yaw,
        )
        q_seed = np.array(seed_positions, dtype=float)
        result = self._solve_ik_with_retry(
            self._model,
            self._data,
            self._end_frame_id,
            target,
            q_seed,
        )
        if not result.success:
            return PreviewSolveResult(
                success=False,
                joint_positions=seed_positions,
                message=(
                    f"target pose unreachable for {self._end_frame_name}: "
                    f"error={result.error:.4e}"
                ),
            )
        return PreviewSolveResult(
            success=True,
            joint_positions=tuple(float(v) for v in result.q.tolist()),
            message="ik preview ok",
        )

    def compute_pose_target(self, joint_positions: tuple[float, ...]) -> PoseTarget:
        """正解：把关节角（rad）换算为末端位姿。

        位置直接取末端帧原点在基坐标系下的坐标（m）；姿态用旋转矩阵转 RPY，单位
        rad，取值范围 roll/yaw 为 (-π, π]、pitch 为 [-π/2, π/2]。返回值可直接喂给
        :meth:`solve_pose` 做回环校验。
        """
        import numpy as np  # pylint: disable=import-outside-toplevel
        import pinocchio as pin  # pylint: disable=import-outside-toplevel
        from reBotArm_control_py.kinematics import compute_fk  # pylint: disable=import-outside-toplevel

        q = np.array(joint_positions, dtype=float)
        position, rotation, _ = compute_fk(self._model, q, frame_name=self._end_frame_name)
        roll, pitch, yaw = pin.rpy.matrixToRpy(rotation)
        return PoseTarget(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
        )
