from __future__ import annotations

import logging
import threading
import time
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml

from .conversions import fk_to_pose

_LOG = logging.getLogger(__name__)

_G_MAX_DIST_M = 0.09
# CAD/software travel is 90 mm, but the currently installed gripper's
# operator-confirmed mechanical limit is 85 mm after the gear damage.
# Keep the mapping range above intact for feedback interpretation while
# refusing to command beyond this real-hardware limit.
_G_VERIFIED_OPEN_LIMIT_M = 0.085
_G_ANGLE_OPEN = -5.0
_G_OPEN_SOFT_LIMIT = -4.9
_G_ARRIVE_TOL = 0.12
_G_TAU_MAX = 1.5
# Normal position moves use a bounded commanded-position ramp.  The 0.5 rad/s
# default matches the existing web/keyboard teleop speed baseline; it remains a
# dedicated gripper parameter so the two mechanisms can be tuned independently.
_G_POSITION_MAX_SPEED_RAD_S = 0.5
_G_POSITION_MAX_SPEED_MIN_RAD_S = 0.05
_G_POSITION_MAX_SPEED_MAX_RAD_S = 3.0
_G_POSITION_TIMEOUT_MARGIN_SEC = 1.5
_G_POSITION_TIMEOUT_MARGIN_MAX_SEC = 10.0
_G_FEEDBACK_STALE_TIMEOUT_SEC = 0.25
_G_FEEDBACK_STALE_TIMEOUT_MAX_SEC = 2.0
# These are motor-side torque limits for the MOVE phase only; what prevents
# driving into a mechanical stop after arrival is the neutral+idle release in
# _release_gripper_position_target.
#
# 2026-08-14: measured on real hardware after the gear repair.  Under a 0.40 N.m
# cap a 0 -> 10 -> 0 mm no-load cycle stalled at 7.86-8.02 mm and 1.83-1.93 mm;
# all three legs needed kp*err of 0.536-0.594 N.m, i.e. they stopped at cap
# saturation rather than at the target, leaving ~2 mm of steady-state error that
# only passed _G_ARRIVE_TOL by as little as 0.02 mm.  Transmission resistance is
# therefore genuinely high.  The 2026-09-04 operator decision is to use the
# known-good Web teleop default of 1.0 N.m, retain 1.5 N.m only as the
# configurable ceiling, and bound motion with the 0.5 rad/s ramp above.
_G_LARGE_MOVE_MAX_TAU_NM = 1.0
_G_LARGE_MOVE_MAX_TAU_CAP_NM = 1.5
_G_KP_MOVE = 5.0
_G_KD_MOVE = 1.0
_G_DEFAULT_FORCE = 0.40
_G_GRASP_CLOSE_KP = 0.0
_G_GRASP_CLOSE_KD = 0.5
_G_GRASP_HOLD_KP = 5.0
_G_GRASP_HOLD_KD = 1.0
# NOTE on grip force: grasp_holding commands
#   kp*(hold_angle - pos) + kd*(-vel) + hold_force
# with hold_angle frozen at the stall position, so the POSITION term dominates
# and hold_force is only a feed-forward addition.  Actual grip force is
# therefore governed by how deep close_force drove the jaws before stalling,
# not by hold_force.  2026-08-14 real hardware, same hold_force=0.4 both runs:
#   close_force 0.4 -> stalled 2.02 mm into a 50 mm bottle, held 0.271-0.286 N.m
#                      (still creeping closed after 10 s; would not grip)
#   close_force 1.0 -> stalled 15.21 mm in (30% compression), 0.423-0.437 N.m
#                      (0.158 mm elastic pushback; gripped)
# Do not treat hold_force as the grip-force knob when tuning.
_G_GRASP_CLOSE_FORCE_DEFAULT = 0.40
_G_GRASP_CLOSE_FORCE_MAX = 1.0
_G_GRASP_HOLD_FORCE_DEFAULT = 0.40
# grasp_holding loads the motor continuously at 500 Hz.  Upstream never exits
# that state, so a successful grasp held torque until the next command or
# controller shutdown -- the same continuous-load mechanism implicated in the
# 2026-08-12 Joint4 over-temperature.  Hold is therefore bounded and always
# ends with a neutral release.
_G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC = 30.0
_G_GRASP_HOLD_TIMEOUT_MAX_SEC = 120.0
_G_GRASP_VEL_THRESHOLD = 0.04
_G_GRASP_MIN_CLOSE_TIME = 0.08
_G_GRASP_MIN_CLOSURE_M = 0.006
_G_GRASP_TIMEOUT = 2.0
_G_CTRL_RATE = 500.0
_GC_VEL_THRESHOLD = 0.04
_GC_W_VEL_THRESHOLD = 0.08
_GC_EE_FRAME = "end_link"
_GC_KP = 7.0
_GC_KD = 0.8
_GC_TAU_SCALE = np.ones(6, dtype=np.float64)
_FEEDBACK_REFRESH_RETRIES = 3
_FEEDBACK_RETRY_INTERVAL_SEC = 0.005

_JOINT_POSITION_LIMITS_RAD = {
    "joint1": (-2.8, 2.8),
    # The confirmed mechanical zero is the joint2 nominal upper endpoint.
    # Keep a small positive software margin for encoder quantization/backlash;
    # this is not an additional commanded workspace.
    "joint2": (-3.14, 0.02),
    "joint3": (-3.14, 0.0),
    "joint4": (-1.87, 1.57),
    "joint5": (-1.57, 1.57),
    "joint6": (-3.14, 3.14),
}

# The vendor ArmEndPos.safe_home drives every joint to 0 rad.  For this arm that
# is a folded posture, not an open one: joint3's origin is -0.264 m in x while
# joint4's is +0.2426 m, so with joints 2-5 near zero the forearm folds back onto
# the upper arm and link2/link5 close to ~1.6 mm.  MoveIt's
# CheckStartStateCollision then aborts every planning request from there
# ("1 contact(s) detected : link2 - link5"), so nothing can be planned out of the
# zero pose and it is unusable as a resting posture.
#
# Rest at the visual-ready posture instead: 33.2 mm measured link2/link5
# clearance, and physically the same compact stance as zero (end_link horizontal
# reach 0.254 m vs 0.260 m, 24 mm higher), so it costs nothing mechanically and
# does not leave the arm extended under gravity.  This is also the posture
# tests/test_paired_trajectory_protocol.py already treats as the canonical
# working pose, and it matches the "safe_home" named state in the MoveIt SRDF
# and the MuJoCo keyframes.
_SAFE_HOME_JOINT_POSITIONS = (-1.5707963267948966, -0.1, -0.2, 0.2, 0.0, 0.0)
_SAFE_HOME_ARRIVE_TOL_RAD = 0.02
_SAFE_HOME_TIMEOUT_SEC = 30.0

_LIFECYCLE_STATES = {
    "DISCONNECTED",
    "CONNECTED_DISABLED",
    "ENABLING",
    "ENABLED_HOLD",
    "TRAJECTORY_RUNNING",
    "DISABLING",
}


def apply_gravity_compensation_tau_scale(tau: np.ndarray) -> np.ndarray:
    scaled = np.array(tau, dtype=np.float64, copy=True)
    if scaled.shape == _GC_TAU_SCALE.shape:
        scaled *= _GC_TAU_SCALE
    return scaled


class HardwareManager:
    """Owns the single RobotArm instance used by the ROS driver."""

    def __init__(
        self,
        arm_cfg: Optional[str] = None,
        gripper_cfg: Optional[str] = None,
        channel: str = "",
        gripper_position_torque_cap_nm: float = _G_LARGE_MOVE_MAX_TAU_NM,
        gripper_position_max_speed_rad_s: float = _G_POSITION_MAX_SPEED_RAD_S,
        gripper_position_timeout_margin_sec: float = _G_POSITION_TIMEOUT_MARGIN_SEC,
        gripper_feedback_stale_timeout_sec: float = _G_FEEDBACK_STALE_TIMEOUT_SEC,
        grasp_hold_timeout_sec: float = _G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC,
    ) -> None:
        requested_cap = float(gripper_position_torque_cap_nm)
        if not 0.05 <= requested_cap <= _G_LARGE_MOVE_MAX_TAU_CAP_NM:
            raise ValueError(
                "gripper_position_torque_cap_nm must be within "
                f"[0.05, {_G_LARGE_MOVE_MAX_TAU_CAP_NM:g}] N.m"
            )
        self._gripper_position_torque_cap_nm = requested_cap
        requested_speed = float(gripper_position_max_speed_rad_s)
        if not _G_POSITION_MAX_SPEED_MIN_RAD_S <= requested_speed <= _G_POSITION_MAX_SPEED_MAX_RAD_S:
            raise ValueError(
                "gripper_position_max_speed_rad_s must be within "
                f"[{_G_POSITION_MAX_SPEED_MIN_RAD_S:g}, "
                f"{_G_POSITION_MAX_SPEED_MAX_RAD_S:g}] rad/s"
            )
        self._gripper_position_max_speed_rad_s = requested_speed
        requested_timeout_margin = float(gripper_position_timeout_margin_sec)
        if not 0.1 <= requested_timeout_margin <= _G_POSITION_TIMEOUT_MARGIN_MAX_SEC:
            raise ValueError(
                "gripper_position_timeout_margin_sec must be within "
                f"[0.1, {_G_POSITION_TIMEOUT_MARGIN_MAX_SEC:g}] s"
            )
        self._gripper_position_timeout_margin_sec = requested_timeout_margin
        requested_stale_timeout = float(gripper_feedback_stale_timeout_sec)
        if not 0.05 <= requested_stale_timeout <= _G_FEEDBACK_STALE_TIMEOUT_MAX_SEC:
            raise ValueError(
                "gripper_feedback_stale_timeout_sec must be within "
                f"[0.05, {_G_FEEDBACK_STALE_TIMEOUT_MAX_SEC:g}] s"
            )
        self._gripper_feedback_stale_timeout_sec = requested_stale_timeout
        requested_hold_timeout = float(grasp_hold_timeout_sec)
        if not 0.1 <= requested_hold_timeout <= _G_GRASP_HOLD_TIMEOUT_MAX_SEC:
            raise ValueError(
                "grasp_hold_timeout_sec must be within "
                f"[0.1, {_G_GRASP_HOLD_TIMEOUT_MAX_SEC:g}] s"
            )
        self._grasp_hold_timeout_sec = requested_hold_timeout
        self._sdk_root = self._ensure_rebot_sdk_in_syspath()

        from reBotArm_control_py.actuator import RobotArm
        from reBotArm_control_py.controllers import ArmEndPos
        from reBotArm_control_py.kinematics import load_robot_model
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        import pinocchio as pin

        cfg_path = Path(arm_cfg).expanduser() if arm_cfg else self.default_arm_cfg()
        cfg_path = self._arm_cfg_with_channel(cfg_path, channel)
        self._arm = RobotArm(cfg_path=str(cfg_path))
        self._gc_model = load_robot_model()
        self._gc_data = self._gc_model.createData()
        self._gc_ee_frame_id = self._gc_model.getFrameId(_GC_EE_FRAME)
        self._gc_compute_generalized_gravity = compute_generalized_gravity
        self._gc_pin = pin

        self._gripper_cfg_path = (
            Path(gripper_cfg).expanduser() if gripper_cfg else self.default_gripper_cfg()
        )
        self._gripper_cfg = None
        self._gripper_mot = None
        self._gripper_ctrl = None
        self._gripper_target_angle = 0.0
        self._gripper_goal_angle = 0.0
        self._gripper_target_effort = _G_DEFAULT_FORCE
        self._gripper_close_force = _G_GRASP_CLOSE_FORCE_DEFAULT
        self._gripper_hold_force = _G_GRASP_HOLD_FORCE_DEFAULT
        self._gripper_hold_angle = 0.0
        self._gripper_hold_deadline: float | None = None
        self._gripper_hold_release_reason: str | None = None
        self._gripper_mode = "idle"
        self._gripper_active = False
        self._gripper_pos = 0.0
        self._gripper_vel = 0.0
        self._gripper_torque = 0.0
        self._gripper_status_code = 255
        self._gripper_feedback_updated_monotonic: float | None = None
        self._gripper_feedback_error: str | None = "gripper feedback not received"
        self._gripper_command_error: str | None = None
        self._gripper_position_result = "idle"
        self._gripper_target_timeout_sec = 0.0
        self._gripper_target_deadline_monotonic: float | None = None
        self._gripper_loop_stop = threading.Event()
        self._gripper_loop_thread: threading.Thread | None = None
        self._gripper_loop_running = False
        self._gripper_last_tick_monotonic: float | None = None
        self._gripper_lock = threading.RLock()
        self._motor_lifecycle_lock = threading.RLock()

        self._endpos_ctrl = ArmEndPos(self._arm)
        self._connected = False
        self._enabled = False
        self._lifecycle_state = "DISCONNECTED"
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []
        self._gravity_comp_active = False
        self._gravity_comp_q_target: np.ndarray | None = None
        self._gravity_comp_integral: np.ndarray | None = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last: np.ndarray | None = None

        self._patch_arm_bus_lock()

    def default_arm_cfg(self) -> Path:
        return self._sdk_root / "config" / "arm.yaml"

    def default_gripper_cfg(self) -> Path:
        return self._sdk_root / "config" / "gripper.yaml"

    @staticmethod
    def _workspace_root() -> Path:
        return Path(__file__).resolve().parents[3]

    @classmethod
    def _sdk_candidates(cls) -> list[Path]:
        workspace = cls._workspace_root()
        return [
            workspace / "third_party" / "reBotArm_control_py",
            workspace / "sdk" / "reBotArm_control_py",
            Path.cwd() / "third_party" / "reBotArm_control_py",
            Path.cwd() / "sdk" / "reBotArm_control_py",
            Path.home() / "robotarm_ros2" / "third_party" / "reBotArm_control_py",
            Path.home() / "robotarm_ros2" / "sdk" / "reBotArm_control_py",
            Path.home() / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
        ]

    @classmethod
    def _ensure_rebot_sdk_in_syspath(cls) -> Path:
        for root in cls._sdk_candidates():
            if (root / "reBotArm_control_py").is_dir():
                root_str = str(root)
                if root_str not in sys.path:
                    sys.path.insert(0, root_str)
                return root
        candidates = "\n".join(f"  - {path}" for path in cls._sdk_candidates())
        raise FileNotFoundError(
            "Cannot find reBotArm_control_py. Clone it into one of:\n"
            f"{candidates}"
        )

    @staticmethod
    def _arm_cfg_with_channel(cfg_path: Path, channel: str) -> Path:
        normalized_channel = str(channel or "").strip()
        if not normalized_channel or normalized_channel.lower() == "auto":
            return cfg_path
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        data["channel"] = normalized_channel
        tmp_dir = Path("/tmp") / "rebotarm_ros2"
        tmp_dir.mkdir(parents=True, exist_ok=True)
        tmp_path = tmp_dir / "arm_channel_override.yaml"
        with open(tmp_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return tmp_path

    @property
    def arm(self):
        return self._arm

    @property
    def endpos_ctrl(self):
        return self._endpos_ctrl

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def control_loop_active(self) -> bool:
        return bool(self._arm.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return self._gripper_mot is not None

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def lifecycle_state(self) -> str:
        return self._lifecycle_state

    @property
    def ready_for_motion(self) -> bool:
        return bool(
            self._connected
            and self._enabled
            and self._lifecycle_state in ("ENABLED_HOLD", "TRAJECTORY_RUNNING")
        )

    @property
    def gripper_active(self) -> bool:
        with self._gripper_lock:
            return bool(self._gripper_active)

    @property
    def gripper_mode(self) -> str:
        with self._gripper_lock:
            return str(self._gripper_mode)

    @property
    def error_codes(self) -> list[str]:
        return list(self._error_codes)

    def set_state_machine(self, state: str) -> None:
        if state not in ("IDLE", "TRAJ_RUNNING", "LOWLEVEL_STREAMING", "GRAVITY_COMP"):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state
        if state == "TRAJ_RUNNING" and self._enabled:
            self._set_lifecycle_state("TRAJECTORY_RUNNING")
        elif state == "IDLE" and self._enabled:
            self._set_lifecycle_state("ENABLED_HOLD")

    def _set_lifecycle_state(self, state: str) -> None:
        if state not in _LIFECYCLE_STATES:
            raise ValueError(f"unsupported lifecycle state: {state}")
        self._lifecycle_state = state

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("hardware is not connected")

    def _require_enabled(self) -> None:
        self._require_connected()
        if not self._enabled:
            raise RuntimeError(
                "hardware is disabled; call explicit /rebotarm/enable first"
            )

    def _refresh_all_feedback(self) -> None:
        self._refresh_arm_feedback()

        self._refresh_gripper_feedback()

    @staticmethod
    def _validated_gripper_feedback_values(state) -> tuple[float, float, float, int]:
        if state is None:
            raise RuntimeError("gripper feedback unavailable")
        position = float(state.pos)
        velocity = float(state.vel)
        torque = float(state.torq)
        if not all(np.isfinite(value) for value in (position, velocity, torque)):
            raise RuntimeError("gripper feedback contains non-finite values")
        return position, velocity, torque, int(state.status_code)

    def _record_gripper_feedback(self, state, *, observed_at: float | None = None) -> None:
        position, velocity, torque, status = self._validated_gripper_feedback_values(state)
        timestamp = time.monotonic() if observed_at is None else float(observed_at)
        with self._gripper_lock:
            recovered = self._gripper_feedback_error is not None
            self._gripper_pos = position
            self._gripper_vel = velocity
            self._gripper_torque = torque
            self._gripper_status_code = status
            self._gripper_feedback_updated_monotonic = timestamp
            self._gripper_feedback_error = None
        if recovered:
            _LOG.info(
                "gripper feedback recovered pos=%.6frad vel=%.6frad/s "
                "torque=%.6fNm status=%d updated=%.6f",
                position,
                velocity,
                torque,
                status,
                timestamp,
            )

    def _record_gripper_feedback_error(self, reason: str) -> None:
        message = str(reason)
        with self._gripper_lock:
            changed = message != self._gripper_feedback_error
            self._gripper_feedback_error = message
        if changed:
            _LOG.error("gripper feedback error: %s", message)

    def _gripper_feedback_age_sec(self, *, now: float | None = None) -> float:
        with self._gripper_lock:
            updated = self._gripper_feedback_updated_monotonic
        if updated is None:
            return float("inf")
        current = time.monotonic() if now is None else float(now)
        return max(current - updated, 0.0)

    def _gripper_feedback_failure_reason_locked(self, *, now: float | None = None) -> str | None:
        if self._gripper_feedback_error is not None:
            return f"gripper feedback unavailable: {self._gripper_feedback_error}"
        age = self._gripper_feedback_age_sec(now=now)
        if age > self._gripper_feedback_stale_timeout_sec:
            return (
                "gripper feedback stale: "
                f"age={age:.3f}s limit={self._gripper_feedback_stale_timeout_sec:.3f}s"
            )
        return None

    def _refresh_gripper_feedback(self):
        if self._gripper_mot is None or self._gripper_ctrl is None:
            raise RuntimeError("gripper feedback unavailable: motor/controller not initialized")

        def refresh_transaction():
            last_error: Exception | None = None
            for attempt in range(_FEEDBACK_REFRESH_RETRIES):
                try:
                    self._gripper_mot.request_feedback()
                    self._gripper_ctrl.poll_feedback_once()
                    state = self._gripper_mot.get_state()
                    self._record_gripper_feedback(state)
                    return state
                except Exception as exc:
                    last_error = exc
                if attempt + 1 < _FEEDBACK_REFRESH_RETRIES:
                    time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)
            detail = f": {last_error}" if last_error is not None else ""
            error = RuntimeError(
                "gripper feedback unavailable after "
                f"{_FEEDBACK_REFRESH_RETRIES} attempts{detail}"
            )
            self._record_gripper_feedback_error(str(error))
            raise error

        lock = getattr(self._gripper_ctrl, "_bus_lock", None)
        if lock is None:
            return refresh_transaction()
        with lock:
            return refresh_transaction()

    def _validated_joint_feedback(
        self,
        *,
        expected_status: int | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[int]]:
        self._refresh_arm_feedback()
        joint_names = self.joint_names
        if len(joint_names) != len(set(joint_names)):
            raise RuntimeError("joint names contain duplicates")
        if len(joint_names) != len(_JOINT_POSITION_LIMITS_RAD):
            raise RuntimeError(
                f"joint count mismatch: got {len(joint_names)}, "
                f"expected {len(_JOINT_POSITION_LIMITS_RAD)}"
            )

        positions: list[float] = []
        velocities: list[float] = []
        torques: list[float] = []
        statuses: list[int] = []
        for name in joint_names:
            if name not in _JOINT_POSITION_LIMITS_RAD:
                raise RuntimeError(f"no hardware soft limit configured for {name}")
            motor = self._arm._motor_map.get(name)
            state = motor.get_state() if motor is not None else None
            if state is None:
                raise RuntimeError(f"{name} feedback unavailable")
            values = (float(state.pos), float(state.vel), float(state.torq))
            if not all(np.isfinite(value) for value in values):
                raise RuntimeError(f"{name} feedback contains non-finite values")
            lower, upper = _JOINT_POSITION_LIMITS_RAD[name]
            if values[0] < lower or values[0] > upper:
                raise RuntimeError(
                    f"{name} position {values[0]:.6f} rad outside "
                    f"hardware soft limit [{lower:.6f}, {upper:.6f}]"
                )
            status = int(state.status_code)
            if expected_status is not None and status != expected_status:
                raise RuntimeError(
                    f"{name} status_code={status}, expected {expected_status}"
                )
            positions.append(values[0])
            velocities.append(values[1])
            torques.append(values[2])
            statuses.append(status)

        return (
            np.array(positions, dtype=np.float64),
            np.array(velocities, dtype=np.float64),
            np.array(torques, dtype=np.float64),
            statuses,
        )

    def _validated_gripper_status(self, expected_status: int) -> None:
        if self._gripper_mot is None:
            return
        self._refresh_gripper_feedback()
        state = self._gripper_mot.get_state()
        if state is None:
            raise RuntimeError("gripper feedback unavailable")
        status = int(state.status_code)
        if status != expected_status:
            raise RuntimeError(
                f"gripper status_code={status}, expected {expected_status}"
            )

    def _disable_all_motors(self) -> None:
        errors: list[str] = []
        try:
            self._arm.disable()
        except Exception as exc:
            errors.append(f"arm disable: {exc}")
        if self._gripper_mot is not None:
            try:
                self._gripper_mot.disable()
            except Exception as exc:
                errors.append(f"gripper disable: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._arm.connect()
            self._connected = True
            self.init_gripper(str(self._gripper_cfg_path))
            _positions, _velocities, _torques, statuses = (
                self._validated_joint_feedback()
            )
            gripper_status = None
            if self._gripper_mot is not None:
                self._refresh_gripper_feedback()
                gripper_state = self._gripper_mot.get_state()
                if gripper_state is None:
                    raise RuntimeError("gripper feedback unavailable")
                gripper_status = int(gripper_state.status_code)
            if any(status != 0 for status in statuses) or gripper_status not in (None, 0):
                self._disable_all_motors()
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            self._enabled = False
            self._set_lifecycle_state("CONNECTED_DISABLED")
        except Exception:
            self._disconnect_after_failed_connect()
            raise

    def _disconnect_after_failed_connect(self) -> None:
        try:
            self._stop_gripper_loop()
        except Exception:
            pass
        try:
            self._stop_control_loop()
        except Exception:
            pass
        try:
            self._disable_all_motors()
        except Exception:
            pass
        try:
            self._arm.disconnect()
        except Exception:
            pass
        self._connected = False
        self._enabled = False
        self._set_lifecycle_state("DISCONNECTED")

    def shutdown(self) -> None:
        if not self._connected:
            return
        try:
            self._stop_gripper_loop()
            self.stop_gravity_compensation()
            self._stop_control_loop()
            try:
                self._disable_all_motors()
            except Exception:
                pass
            # ArmEndPos.end() runs the vendor safe_home() before disconnecting, and
            # that targets the all-zero folded pose (see _SAFE_HOME_JOINT_POSITIONS).
            # The control loop is already stopped and the motors already disabled at
            # this point, so the call cannot move the arm at all -- it only polls for
            # its full 30 s timeout and leaves _q_target at zero for whoever enables
            # next.  Retire the controller directly; a deliberate safe_home belongs to
            # the driver's shutdown hook, which runs earlier while still enabled.
            self._endpos_ctrl._running = False
            self._arm.disconnect()
        finally:
            self._connected = False
            self._enabled = False
            self._set_lifecycle_state("DISCONNECTED")

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._motor_lifecycle_lock:
            if not self._connected:
                return self._arm.get_state()
            expected_status = 1 if self._enabled else 0
            positions, velocities, torques, _statuses = self._validated_joint_feedback(
                expected_status=expected_status,
            )
            return positions, velocities, torques

    def hold_current_position(self) -> np.ndarray:
        q, _, _ = self.get_joint_state()
        current = np.array(q, dtype=np.float64, copy=True)
        self._endpos_ctrl._q_target[:] = current
        return current

    def stop_active_motion(self) -> None:
        with self._motor_lifecycle_lock:
            self._endpos_ctrl._stop_send.set()
            self._endpos_ctrl._moving = False
            if self._enabled:
                self.hold_current_position()
            self.set_state_machine("IDLE")

    def safe_home_target(self) -> np.ndarray:
        return np.array(_SAFE_HOME_JOINT_POSITIONS, dtype=np.float64)

    def _validated_safe_home_target(
        self, target: Optional[Sequence[float]] = None
    ) -> np.ndarray:
        if target is None:
            return self.safe_home_target()
        values = np.array([float(value) for value in target], dtype=np.float64)
        expected = len(_SAFE_HOME_JOINT_POSITIONS)
        if values.shape != (expected,):
            raise ValueError(
                f"safe_home target must hold {expected} joint positions, got {values.shape[0]}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("safe_home target must be finite")
        for name, value in zip(self.joint_names, values):
            limits = _JOINT_POSITION_LIMITS_RAD.get(name)
            if limits is None:
                continue
            low, high = limits
            if not low <= value <= high:
                raise ValueError(
                    f"safe_home target for {name} is {value:.4f} rad, "
                    f"outside the allowed range [{low}, {high}]"
                )
        return values

    def safe_home(
        self,
        target: Optional[Sequence[float]] = None,
        vlim: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> np.ndarray:
        """Drive the arm to a collision-free resting posture.

        Replaces ArmEndPos.safe_home, which hardcodes an all-zero target; see
        _SAFE_HOME_JOINT_POSITIONS for why the zero pose cannot be rested in.
        The vendor velocity override and arrival polling are kept, but arrival is
        checked against the requested target rather than against zero, and the
        move is never allowed to exceed the vendor's own homing speed.
        """
        goal = self._validated_safe_home_target(target)
        ctrl = self._endpos_ctrl
        with self._motor_lifecycle_lock:
            self._require_enabled()
            if not ctrl._running:
                raise RuntimeError(
                    "control loop is inactive; safe_home needs position-velocity control"
                )
            speed = ctrl._home_vel if vlim is None else float(vlim)
            if not 0.0 < speed <= ctrl._home_vel:
                raise ValueError(
                    f"safe_home vlim must be within (0, {ctrl._home_vel:g}] rad/s"
                )
            ctrl._vlim_override = np.full(ctrl._n, speed, dtype=np.float64)
            ctrl._q_target[:] = goal
            ctrl._stop_send.set()
            ctrl._moving = False
            send_thread = ctrl._send_thread
            self.set_state_machine("TRAJ_RUNNING")
        try:
            if send_thread is not None:
                send_thread.join()
            deadline = time.monotonic() + (
                _SAFE_HOME_TIMEOUT_SEC if timeout_sec is None else float(timeout_sec)
            )
            while True:
                positions, _velocities, _torques = self.get_joint_state()
                reached = np.array(positions, dtype=np.float64)
                worst = float(np.max(np.abs(reached - goal)))
                if worst < _SAFE_HOME_ARRIVE_TOL_RAD:
                    return reached
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"safe_home did not reach {goal.tolist()} within "
                        f"{_SAFE_HOME_ARRIVE_TOL_RAD} rad; worst joint error {worst:.4f} rad"
                    )
                time.sleep(ctrl._dt)
        finally:
            ctrl._vlim_override = None
            with self._motor_lifecycle_lock:
                if self._enabled:
                    self.hold_current_position()
                self.set_state_machine("IDLE")

    def enable(self) -> None:
        from motorbridge import Mode

        with self._motor_lifecycle_lock:
            self._require_connected()
            if self._enabled:
                self.hold_current_position()
                self._set_lifecycle_state("ENABLED_HOLD")
                return

            self._set_lifecycle_state("ENABLING")
            try:
                positions, _velocities, _torques, _statuses = (
                    self._validated_joint_feedback(expected_status=0)
                )
                self._validated_gripper_status(expected_status=0)
                self._endpos_ctrl._q_target[:] = positions
                if self._arm.mode_pos_vel() is False:
                    raise RuntimeError("failed to enter position-velocity control mode")
                if self._gripper_mot is not None:
                    self._gripper_mot.ensure_mode(Mode.MIT, 1000)
                self._arm.enable()
                if self._gripper_mot is not None:
                    self._gripper_mot.enable()
                self._validated_joint_feedback(expected_status=1)
                self._validated_gripper_status(expected_status=1)
                self._enabled = True
                self._start_pos_vel_loop(target=positions)
                self.set_state_machine("IDLE")
                self._set_lifecycle_state("ENABLED_HOLD")
            except Exception as exc:
                rollback_error = self._rollback_failed_enable()
                if rollback_error is not None:
                    raise RuntimeError(
                        f"enable failed: {exc}; rollback verification failed: "
                        f"{rollback_error}"
                    ) from exc
                raise RuntimeError(f"enable failed and was rolled back: {exc}") from exc

    def _rollback_failed_enable(self) -> str | None:
        rollback_errors: list[str] = []
        try:
            self._stop_gripper_loop()
        except Exception as exc:
            rollback_errors.append(f"stop gripper loop: {exc}")
        try:
            self._stop_control_loop()
        except Exception as exc:
            rollback_errors.append(f"stop arm loop: {exc}")
        try:
            self._disable_all_motors()
        except Exception as exc:
            rollback_errors.append(f"disable command: {exc}")
        if not rollback_errors:
            try:
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            except Exception as exc:
                rollback_errors.append(f"disabled feedback: {exc}")
        self._state_machine = "IDLE"
        if rollback_errors:
            self._enabled = True
            self._error_codes.append("ENABLE_ROLLBACK_FAILED")
            self._set_lifecycle_state("DISABLING")
            return "; ".join(rollback_errors)
        self._enabled = False
        self._set_lifecycle_state("CONNECTED_DISABLED" if self._connected else "DISCONNECTED")
        return None

    def disable(self) -> None:
        with self._motor_lifecycle_lock:
            self._require_connected()
            self._set_lifecycle_state("DISABLING")
            was_enabled = self._enabled
            self._enabled = False
            self.stop_gravity_compensation()
            self._stop_gripper_loop()
            self._stop_control_loop()
            try:
                self._disable_all_motors()
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            except Exception:
                self._enabled = was_enabled
                self._error_codes.append("DISABLE_VERIFICATION_FAILED")
                raise
            else:
                self._enabled = False
                self._state_machine = "IDLE"
                self._set_lifecycle_state("CONNECTED_DISABLED")

    def set_mode(self, mode: str) -> bool:
        self._require_connected()
        mode = mode.strip().lower()
        if mode not in ("mit", "pos_vel", "vel"):
            raise ValueError(f"unsupported mode: {mode}")
        self.stop_gravity_compensation()

        if mode == self.mode:
            if mode == "pos_vel" and self._enabled:
                if self.control_loop_active:
                    self.hold_current_position()
                else:
                    self._start_pos_vel_loop()
            self.set_state_machine("IDLE")
            return True

        self._stop_control_loop()
        if mode == "mit":
            ok = self._arm.mode_mit()
        elif mode == "pos_vel":
            ok = self._arm.mode_pos_vel()
            if self._enabled:
                self._start_pos_vel_loop()
        else:
            ok = self._arm.mode_vel()
        self.set_state_machine("IDLE")
        return bool(ok)

    def set_zero(self, joint_name: str = "") -> bool:
        self._require_connected()
        if self._enabled:
            raise RuntimeError("set_zero requires CONNECTED_DISABLED state")
        self._stop_control_loop()
        joint_name = str(joint_name).strip()
        if joint_name == "gripper":
            if self._gripper_mot is None:
                raise RuntimeError("gripper is not initialized")
            self._stop_gripper_loop()
            self._validated_gripper_status(expected_status=0)
            try:
                self._gripper_mot.set_zero_position()
            except Exception as exc:
                raise RuntimeError(f"gripper set_zero failed: {exc}") from exc
            self._enabled = False
            self.set_state_machine("IDLE")
            return True
        if joint_name:
            ok = self._arm.set_zero_single(joint_name)
        else:
            self._arm.set_zero()
            ok = True
        self._enabled = False
        self.set_state_machine("IDLE")
        return bool(ok)

    def ensure_pos_vel_control(self) -> None:
        self._require_enabled()
        if self.mode != "pos_vel":
            self._stop_control_loop()
            self._arm.mode_pos_vel()
        if not self.control_loop_active:
            self._start_pos_vel_loop()
        else:
            self.hold_current_position()

    def send_joint_motor_cmd(self, joint_name: str, cmd) -> None:
        self._require_enabled()
        if joint_name not in self._arm._motor_map:
            raise KeyError(f"unknown joint: {joint_name}")

        mot = self._arm._motor_map[joint_name]
        jc = next(j for j in self._arm._joints if j.name == joint_name)
        state = mot.get_state()

        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(jc.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(jc.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(jc.vlim)

        if int(cmd.mode) == 0:
            mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(mot, "send_vel"):
                raise RuntimeError(f"{joint_name} does not support send_vel")
            mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        self.set_state_machine("LOWLEVEL_STREAMING")

    def start_gravity_compensation(self) -> None:
        self._require_enabled()
        self.stop_gravity_compensation()
        self._stop_control_loop()
        self._endpos_ctrl._stop_send.set()
        self._endpos_ctrl._moving = False
        self._gravity_comp_q_target = self._arm.get_positions(request=True).copy()
        self._gravity_comp_q_last = self._gravity_comp_q_target.copy()
        self._arm.mode_mit(
            kp=np.full(self._arm.num_joints, _GC_KP, dtype=np.float64),
            kd=np.full(self._arm.num_joints, _GC_KD, dtype=np.float64),
        )
        self._refresh_arm_feedback()
        self._gravity_comp_integral = np.zeros_like(self._gravity_comp_q_target)
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_active = True
        self._gravity_comp_tick(self._arm, 1.0 / float(self._arm._rate))
        self._arm.start_control_loop(self._gravity_comp_tick, rate=self._arm._rate)
        self.set_state_machine("GRAVITY_COMP")

    def stop_gravity_compensation(self) -> None:
        if not self._gravity_comp_active:
            return
        hold_target = (
            self._gravity_comp_q_last.copy()
            if self._gravity_comp_q_last is not None
            else None
        )
        self._arm.stop_control_loop()
        self._gravity_comp_active = False
        self._gravity_comp_q_target = None
        self._gravity_comp_integral = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last = None
        if self._enabled:
            self._arm.mode_pos_vel()
            self._start_pos_vel_loop(target=hold_target)
        self.set_state_machine("IDLE")

    def gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    def gravity_compensation_target(self) -> np.ndarray | None:
        if self._gravity_comp_q_target is None:
            return None
        return self._gravity_comp_q_target.copy()

    def _refresh_arm_feedback(self) -> None:
        ctrl_map = getattr(self._arm, "_ctrl_map", None)
        motor_map = getattr(self._arm, "_motor_map", None)
        joints = getattr(self._arm, "_joints", None)
        if ctrl_map and motor_map and joints:
            try:
                for vendor, controller in ctrl_map.items():
                    def refresh_vendor_transaction() -> None:
                        for joint in joints:
                            if joint.vendor != vendor:
                                continue
                            motor = motor_map[joint.name]
                            last_error: Exception | None = None
                            for attempt in range(_FEEDBACK_REFRESH_RETRIES):
                                try:
                                    motor.request_feedback()
                                    controller.poll_feedback_once()
                                    if motor.get_state() is not None:
                                        break
                                except Exception as exc:
                                    last_error = exc
                                if attempt + 1 < _FEEDBACK_REFRESH_RETRIES:
                                    time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)
                            else:
                                detail = (
                                    f": {last_error}"
                                    if last_error is not None
                                    else ""
                                )
                                raise RuntimeError(
                                    f"{joint.name} feedback unavailable after "
                                    f"{_FEEDBACK_REFRESH_RETRIES} attempts{detail}"
                                )

                    lock = getattr(controller, "_bus_lock", None)
                    if lock is None:
                        refresh_vendor_transaction()
                    else:
                        with lock:
                            refresh_vendor_transaction()
            except Exception as exc:
                raise RuntimeError(f"arm feedback refresh failed: {exc}") from exc
            return

        fresh = getattr(self._arm, "fresh", None)
        if callable(fresh):
            fresh()
            return
        request_and_poll = getattr(self._arm, "_request_and_poll", None)
        if callable(request_and_poll):
            request_and_poll()
            return
        self._arm.get_positions(request=True)

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        q = self._arm.get_positions(request=request)
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy()

    def _gravity_comp_tick(self, arm, dt: float) -> None:
        del dt
        if not self._gravity_comp_active or self._gravity_comp_q_target is None:
            return

        q = self._read_gravity_comp_positions()
        qd = arm.get_velocities()
        tau_g = self._gc_compute_generalized_gravity(q=q)
        tau_g = apply_gravity_compensation_tau_scale(tau_g)

        q_error = self._gravity_comp_q_target - q
        if self._gravity_comp_integral is None:
            self._gravity_comp_integral = np.zeros_like(q)
        self._gravity_comp_integral += q_error * 1.0
        np.clip(self._gravity_comp_integral, -0.5, 0.5, out=self._gravity_comp_integral)

        self._gc_pin.computeJointJacobians(self._gc_model, self._gc_data, q)
        self._gc_pin.updateFramePlacements(self._gc_model, self._gc_data)
        jacobian = self._gc_pin.getFrameJacobian(
            self._gc_model,
            self._gc_data,
            self._gc_ee_frame_id,
            self._gc_pin.ReferenceFrame.WORLD,
        )
        spatial_velocity = jacobian @ qd
        linear_speed = float(np.linalg.norm(spatial_velocity[:3]))
        angular_speed = float(np.linalg.norm(spatial_velocity[3:]))

        if linear_speed > _GC_VEL_THRESHOLD or angular_speed > _GC_W_VEL_THRESHOLD:
            self._gravity_comp_q_target = q.copy()
            self._gravity_comp_lock_counter = 0
            self._gravity_comp_integral *= 0.9
        else:
            self._gravity_comp_lock_counter += 1

        arm.mit(
            pos=self._gravity_comp_q_target,
            vel=np.zeros(arm.num_joints),
            kp=np.full(arm.num_joints, _GC_KP),
            kd=np.full(arm.num_joints, _GC_KD),
            tau=tau_g + self._gravity_comp_integral,
        )

    def current_pose(self):
        from reBotArm_control_py.kinematics import compute_fk

        q, _, _ = self.get_joint_state()
        position, rotation, _ = compute_fk(self._endpos_ctrl._model, q)
        return fk_to_pose(position, rotation)

    def get_joint_status_codes(self) -> list[int]:
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._arm._motor_map[name].get_state()
                codes.append(int(st.status_code if st is not None else 0))
            except Exception:
                codes.append(0)
        return codes

    def init_gripper(self, cfg_path: str) -> None:
        from reBotArm_control_py.actuator.gripper import load_cfg as load_gripper_cfg

        gcfg = load_gripper_cfg(cfg_path)
        gc = gcfg["gripper"]
        self._gripper_cfg = gc

        vendor = gc.vendor
        if vendor not in self._arm._ctrl_map:
            raise RuntimeError(
                f"gripper vendor={vendor!r} cannot share the arm Controller"
            )
        ctrl = self._arm._ctrl_map[vendor]

        if vendor == "damiao":
            self._gripper_mot = ctrl.add_damiao_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "myactuator":
            self._gripper_mot = ctrl.add_myactuator_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "robstride":
            self._gripper_mot = ctrl.add_robstride_motor(gc.motor_id, gc.feedback_id, gc.model)
        else:
            raise ValueError(f"unsupported gripper vendor: {vendor!r}")

        self._gripper_ctrl = ctrl

        self._patch_controller_bus(ctrl)
        self._wrap_motor_bus(self._gripper_mot, ctrl._bus_lock)
        # Keep initialization single-threaded while the arm and gripper perform
        # their first fresh-feedback validation on the shared serial bus.  The
        # position/grasp entry points start this loop on demand.

    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        self._require_enabled()
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        state = self._refresh_gripper_feedback()
        start_angle, _velocity, _torque, status = self._validated_gripper_feedback_values(state)
        if status != 1:
            raise RuntimeError(f"gripper status_code={status}, expected 1 before position command")
        distance = float(np.clip(position_m, 0.0, _G_VERIFIED_OPEN_LIMIT_M))
        target = max((distance / _G_MAX_DIST_M) * _G_ANGLE_OPEN, _G_OPEN_SOFT_LIMIT)
        effort = _G_DEFAULT_FORCE if max_effort <= 0.0 else float(max_effort)
        now = time.monotonic()
        target_timeout_sec = (
            abs(target - start_angle) / self._gripper_position_max_speed_rad_s
            + self._gripper_position_timeout_margin_sec
        )
        with self._gripper_lock:
            self._gripper_target_angle = start_angle
            self._gripper_goal_angle = target
            self._gripper_target_effort = float(
                np.clip(
                    effort,
                    0.05,
                    getattr(
                        self,
                        "_gripper_position_torque_cap_nm",
                        _G_LARGE_MOVE_MAX_TAU_NM,
                    ),
                )
            )
            self._gripper_mode = "position"
            self._gripper_active = True
            self._gripper_position_result = "active"
            self._gripper_command_error = None
            self._gripper_last_tick_monotonic = now
            self._gripper_target_timeout_sec = target_timeout_sec
            self._gripper_target_deadline_monotonic = now + target_timeout_sec
            feedback_updated = self._gripper_feedback_updated_monotonic
            applied_effort = self._gripper_target_effort
        _LOG.info(
            "gripper position command requested=%.6fm clamped=%.6fm "
            "start=%.6frad goal=%.6frad effort=%.6fNm speed=%.6frad/s "
            "timeout=%.3fs feedback_updated=%s",
            float(position_m),
            distance,
            start_angle,
            target,
            applied_effort,
            self._gripper_position_max_speed_rad_s,
            target_timeout_sec,
            feedback_updated,
        )
        self._start_gripper_loop()

    def gripper_target_timeout_sec(self) -> float:
        with self._gripper_lock:
            return float(self._gripper_target_timeout_sec)

    @property
    def gripper_command_error(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_command_error

    @property
    def gripper_feedback_error(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_feedback_error

    def wait_gripper_target(self, timeout: float | None = None) -> bool:
        # Remember which goal this waiter owns so neither the arrival nor the
        # timeout path can release a newer command issued in the meantime.
        with self._gripper_lock:
            owned_goal = self._gripper_goal_angle
            deadline = self._gripper_target_deadline_monotonic
        now = time.monotonic()
        explicit_deadline = (
            None if timeout is None else now + max(float(timeout), 0.0)
        )
        if deadline is None:
            deadline = now if explicit_deadline is None else explicit_deadline
        elif explicit_deadline is not None:
            deadline = min(deadline, explicit_deadline)
        while time.monotonic() < deadline:
            with self._gripper_lock:
                if self._gripper_goal_angle != owned_goal:
                    return False
                feedback_failure = self._gripper_feedback_failure_reason_locked()
                if feedback_failure is not None:
                    self._fail_active_gripper_command_locked(feedback_failure)
                    return False
                if not self._gripper_active:
                    return self._gripper_position_result == "succeeded"
                arrived = abs(self._gripper_pos - owned_goal) < _G_ARRIVE_TOL
            if arrived:
                # A completed positioning move is not a grasp hold.  Release the
                # MIT position command before acknowledging success so the motor
                # cannot keep driving toward the target after this returns.
                self._release_gripper_position_target(owned_goal)
                return self.gripper_reached_target()
            time.sleep(0.02)
        # A timeout also ends this command's ownership of the gripper: stop
        # driving instead of leaving a stale position hold running.
        self.cancel_gripper_position_command(
            "gripper target timeout: "
            f"goal={owned_goal:.6f}rad feedback={self._gripper_pos:.6f}rad"
        )
        return False

    def set_gripper_position(self, position_m: float, max_effort: float = 0.0) -> tuple[bool, float]:
        self.set_gripper_target(position_m, max_effort)
        reached = self.wait_gripper_target()
        return reached, self.gripper_position_m()

    def grasp_gripper(
        self,
        close_force: float = _G_GRASP_CLOSE_FORCE_DEFAULT,
        hold_force: float = _G_GRASP_HOLD_FORCE_DEFAULT,
        close_timeout_sec: float = _G_GRASP_TIMEOUT,
        min_close_time_sec: float = _G_GRASP_MIN_CLOSE_TIME,
        velocity_threshold: float = _G_GRASP_VEL_THRESHOLD,
        min_closure_distance_m: float = _G_GRASP_MIN_CLOSURE_M,
        hold_timeout_sec: float | None = None,
    ) -> tuple[bool, bool, float, float, float, str]:
        self._require_enabled()
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")

        close_effort = float(np.clip(close_force, 0.05, _G_GRASP_CLOSE_FORCE_MAX))
        hold_effort = float(np.clip(hold_force, 0.05, _G_TAU_MAX))
        # A grasp hold is always bounded; an unbounded hold keeps loading the
        # motor at 500 Hz with no exit path.
        requested_hold = (
            self._grasp_hold_timeout_sec
            if hold_timeout_sec is None
            else float(hold_timeout_sec)
        )
        hold_timeout = float(np.clip(requested_hold, 0.1, _G_GRASP_HOLD_TIMEOUT_MAX_SEC))
        timeout = max(float(close_timeout_sec), 0.1)
        min_time = max(float(min_close_time_sec), 0.0)
        velocity_limit = max(float(velocity_threshold), 0.0)
        min_closure = max(float(min_closure_distance_m), 0.0)
        start_position_m = self.gripper_position_m()
        start = time.monotonic()

        with self._gripper_lock:
            self._gripper_close_force = close_effort
            self._gripper_hold_force = hold_effort
            self._gripper_hold_deadline = None
            self._gripper_hold_release_reason = None
            self._gripper_mode = "grasp_closing"
            self._gripper_active = True
        self._start_gripper_loop()

        while time.monotonic() - start < timeout:
            elapsed = time.monotonic() - start
            reached_position_m = self.gripper_position_m()
            closure_m = max(start_position_m - reached_position_m, 0.0)
            if (
                elapsed >= min_time
                and closure_m >= min_closure
                and abs(float(self._gripper_vel)) <= velocity_limit
            ):
                with self._gripper_lock:
                    self._gripper_hold_angle = float(self._gripper_pos)
                    self._gripper_hold_force = hold_effort
                    self._gripper_hold_deadline = time.monotonic() + hold_timeout
                    self._gripper_mode = "grasp_holding"
                    self._gripper_active = True
                contact_position_m = self.gripper_position_m()
                return (
                    True,
                    True,
                    contact_position_m,
                    contact_position_m,
                    hold_effort,
                    # No force sensor: this is closure travel plus a velocity
                    # stall, i.e. stall detection, not a measured contact force.
                    "closing stalled; holding "
                    f"(bounded to {hold_timeout:g} s)",
                )
            time.sleep(0.01)

        # Upstream switched to idle here without neutralizing, leaving the last
        # closing torque applied.
        with self._gripper_lock:
            self._release_grasp_hold_locked("close timeout before stall")
        return (
            False,
            False,
            0.0,
            self.gripper_position_m(),
            hold_effort,
            "grasp close timeout before stall detected",
        )

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        status = 255
        if self._gripper_mot is not None:
            try:
                st = self._gripper_mot.get_state()
                position, velocity, torque, status = self._validated_gripper_feedback_values(st)
                with self._gripper_lock:
                    self._gripper_pos = position
                    self._gripper_vel = velocity
                    self._gripper_torque = torque
                    self._gripper_status_code = status
                age = self._gripper_feedback_age_sec()
                if age > self._gripper_feedback_stale_timeout_sec:
                    self._record_gripper_feedback_error(
                        "gripper feedback stale: "
                        f"age={age:.3f}s "
                        f"limit={self._gripper_feedback_stale_timeout_sec:.3f}s"
                    )
                    status = 255
            except Exception as exc:
                self._record_gripper_feedback_error(str(exc))
                status = 255
        return self._gripper_pos, self._gripper_vel, self._gripper_torque, status

    def gripper_position_m(self) -> float:
        distance = (self._gripper_pos / _G_ANGLE_OPEN) * _G_MAX_DIST_M
        return float(np.clip(distance, 0.0, _G_MAX_DIST_M))

    def gripper_reached_target(self) -> bool:
        with self._gripper_lock:
            if self._gripper_position_result == "succeeded":
                return True
            if self._gripper_position_result == "failed" or not self._gripper_active:
                return False
            feedback_failure = self._gripper_feedback_failure_reason_locked()
            if feedback_failure is not None:
                self._fail_active_gripper_command_locked(feedback_failure)
                return False
            goal = self._gripper_goal_angle
            return abs(self._gripper_pos - goal) < _G_ARRIVE_TOL

    def send_gripper_motor_cmd(self, cmd) -> None:
        self._require_enabled()
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        state = self._gripper_mot.get_state()
        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(self._gripper_cfg.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(self._gripper_cfg.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(self._gripper_cfg.vlim)

        if int(cmd.mode) == 0:
            self._gripper_mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            self._gripper_mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(self._gripper_mot, "send_vel"):
                raise RuntimeError("gripper does not support send_vel")
            self._gripper_mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        with self._gripper_lock:
            self._gripper_active = False
            self._gripper_mode = "idle"

    def _patch_arm_bus_lock(self) -> None:
        for ctrl in self._arm._ctrl_map.values():
            self._patch_controller_bus(ctrl)

        if not hasattr(self._arm, "_bus_lock_patched"):
            for jc in self._arm._joints:
                mot = self._arm._motor_map[jc.name]
                ctrl = self._arm._ctrl_map[jc.vendor]
                self._wrap_motor_bus(mot, ctrl._bus_lock)
            self._arm._bus_lock_patched = True

    @staticmethod
    def _patch_controller_bus(ctrl) -> None:
        if not hasattr(ctrl, "_bus_lock"):
            ctrl._bus_lock = threading.RLock()
        if hasattr(ctrl, "_bus_lock_patched"):
            return
        lock = ctrl._bus_lock

        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in ("poll_feedback_once", "enable_all", "disable_all"):
            if hasattr(ctrl, attr):
                wrapped = _wrap(getattr(ctrl, attr))
                wrapped._rebotarm_locked = True
                setattr(ctrl, attr, wrapped)
        ctrl._bus_lock_patched = True

    @staticmethod
    def _wrap_motor_bus(mot, lock) -> None:
        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in (
            "send_pos_vel",
            "send_mit",
            "send_vel",
            "request_feedback",
            "enable",
            "disable",
            "ensure_mode",
            "write_register_f32",
            "set_zero_position",
        ):
            if hasattr(mot, attr) and not hasattr(getattr(mot, attr), "_rebotarm_locked"):
                wrapped = _wrap(getattr(mot, attr))
                wrapped._rebotarm_locked = True
                setattr(mot, attr, wrapped)

    def _start_pos_vel_loop(self, target: np.ndarray | None = None) -> None:
        if self.control_loop_active:
            return
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.array(target, dtype=np.float64)
        self._arm.start_control_loop(self._endpos_ctrl._loop_cb)
        self._endpos_ctrl._running = True

    def _stop_control_loop(self) -> None:
        self._arm.stop_control_loop()
        self._endpos_ctrl._running = False

    def _gripper_safe_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau_ff: float = 0.0,
        tau_limit: float = _G_TAU_MAX,
    ) -> None:
        if self._gripper_mot is None or self._gripper_ctrl is None:
            return
        pos_cmd = float(np.clip(pos, _G_OPEN_SOFT_LIMIT, 0.0))
        pos_term = kp * (pos_cmd - self._gripper_pos) + kd * (-self._gripper_vel)
        limit = float(np.clip(abs(tau_limit), 0.05, _G_TAU_MAX))
        tau_safe = float(np.clip(pos_term + tau_ff, -limit, limit)) - pos_term
        lock = getattr(self._gripper_ctrl, "_bus_lock", None)
        try:
            if lock:
                with lock:
                    self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
                    self._gripper_mot.request_feedback()
                    self._gripper_ctrl.poll_feedback_once()
                    state = self._gripper_mot.get_state()
            else:
                self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
                self._gripper_mot.request_feedback()
                self._gripper_ctrl.poll_feedback_once()
                state = self._gripper_mot.get_state()
            self._record_gripper_feedback(state)
        except Exception as exc:
            self._record_gripper_feedback_error(str(exc))
            raise RuntimeError(f"gripper MIT command feedback failed: {exc}") from exc

    def _fail_active_gripper_command_locked(self, reason: str) -> None:
        message = str(reason)
        self._gripper_command_error = message
        self._gripper_position_result = "failed"
        self._gripper_active = False
        self._gripper_mode = "idle"
        self._gripper_target_deadline_monotonic = None
        _LOG.error(
            "gripper command failed reason=%s goal=%.6frad command=%.6frad "
            "feedback=%.6frad updated=%s",
            message,
            self._gripper_goal_angle,
            self._gripper_target_angle,
            self._gripper_pos,
            self._gripper_feedback_updated_monotonic,
        )
        if self._gripper_mot is None:
            return
        try:
            self._gripper_mot.send_mit(
                float(self._gripper_pos),
                0.0,
                0.0,
                0.0,
                0.0,
            )
        except Exception as exc:
            _LOG.error("gripper neutral command failed after %s: %s", message, exc)

    def cancel_gripper_position_command(self, reason: str = "position command canceled") -> bool:
        with self._gripper_lock:
            if not self._gripper_active or self._gripper_mode != "position":
                return False
            self._fail_active_gripper_command_locked(reason)
            return True

    def _release_gripper_position_target(
        self,
        expected_goal: float,
        *,
        require_arrived: bool = True,
    ) -> bool:
        """End a normal position move and leave the gripper idle.

        Upstream only returns success from ``wait_gripper_target`` and keeps
        ``_gripper_active``/``_gripper_mode`` unchanged, so the 500 Hz tick goes
        on sending MIT position commands after the service has already
        answered.  This sends one neutral MIT command (zero stiffness, damping
        and feed-forward torque) and then atomically switches to idle.

        The gripper lock is held for the whole sequence so a newer target
        cannot be clobbered by a stale completion.  ``grasp_closing`` and
        ``grasp_holding`` are explicitly requested force operations and are
        never released here.
        """
        with self._gripper_lock:
            return self._release_gripper_position_target_locked(
                expected_goal,
                require_arrived=require_arrived,
            )

    def _release_gripper_position_target_locked(
        self,
        expected_goal: float,
        *,
        require_arrived: bool = True,
    ) -> bool:
        """``_release_gripper_position_target`` body; caller must hold the lock.

        This split keeps the ownership checks and neutralization in one place
        for callers that already hold ``_gripper_lock``.
        """
        if not self._gripper_active or self._gripper_mode != "position":
            return False
        if self._gripper_goal_angle != expected_goal:
            return False
        arrived_angle = float(self._gripper_pos)
        if require_arrived and abs(arrived_angle - expected_goal) >= _G_ARRIVE_TOL:
            return False
        # Clear ownership BEFORE sending neutral.  The 500 Hz tick tests
        # _gripper_active first, so once this is false no further position
        # command can be produced and the neutral below is the last word.
        self._gripper_target_angle = arrived_angle
        self._gripper_goal_angle = arrived_angle
        self._gripper_active = False
        self._gripper_mode = "idle"
        self._gripper_target_deadline_monotonic = None
        if require_arrived:
            self._gripper_position_result = "releasing"
        elif self._gripper_position_result != "failed":
            self._gripper_position_result = "failed"
        try:
            self._gripper_safe_mit(
                arrived_angle,
                0.0,
                0.0,
                0.0,
                tau_limit=0.05,
            )
        except Exception as exc:
            self._gripper_position_result = "failed"
            self._gripper_command_error = f"gripper neutral release failed: {exc}"
            raise
        if require_arrived:
            self._gripper_position_result = "succeeded"
            self._gripper_command_error = None
        return True

    def _grasp_hold_expired_locked(self) -> bool:
        """Whether the bounded grasp hold has run out.  Caller holds the lock."""
        deadline = self._gripper_hold_deadline
        if deadline is None:
            return False
        return time.monotonic() >= deadline

    def _release_grasp_hold_locked(self, reason: str) -> None:
        """Neutralize a grasp close/hold and go idle.  Caller holds the lock.

        Upstream left ``grasp_holding`` loaded until the next command and its
        close-timeout path switched to idle without ever neutralizing the last
        torque command.  Both paths end here instead.
        """
        self._gripper_hold_deadline = None
        self._gripper_hold_release_reason = reason
        self._gripper_active = False
        self._gripper_mode = "idle"
        self._gripper_safe_mit(
            float(self._gripper_pos),
            0.0,
            0.0,
            0.0,
            tau_limit=0.05,
        )

    def release_grasp_hold(self, reason: str = "external release") -> bool:
        """Release an active grasp hold without commanding a new position."""
        with self._gripper_lock:
            if not self._gripper_active:
                return False
            if self._gripper_mode not in ("grasp_closing", "grasp_holding"):
                return False
            self._release_grasp_hold_locked(reason)
        return True

    @property
    def grasp_hold_release_reason(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_hold_release_reason

    def _gripper_tick(self) -> None:
        try:
            st = self._gripper_mot.get_state()
            position, velocity, torque, status = self._validated_gripper_feedback_values(st)
            self._gripper_pos = position
            self._gripper_vel = velocity
            self._gripper_torque = torque
            self._gripper_status_code = status
        except Exception as exc:
            self._record_gripper_feedback_error(str(exc))
            with self._gripper_lock:
                if self._gripper_active:
                    self._fail_active_gripper_command_locked(
                        f"gripper feedback read failed: {exc}"
                    )
            return

        # The arrival test and the command emission must be atomic.  Sampling
        # state, releasing the lock, then sending allowed a tick that had
        # already passed the arrival test to emit one more torque-carrying
        # command after wait_gripper_target had released -- observed on real
        # hardware as ~1.2-1.33 mm of extra closing travel after the service
        # returned.  Closing to zero is the exposed direction because
        # abs(target) < 1e-6 turns effort into a feed-forward term there.
        with self._gripper_lock:
            mode = self._gripper_mode
            if not self._gripper_active:
                command = None
            elif (feedback_failure := self._gripper_feedback_failure_reason_locked()) is not None:
                self._fail_active_gripper_command_locked(feedback_failure)
                command = None
            elif mode == "position":
                goal = self._gripper_goal_angle
                if abs(self._gripper_pos - goal) < _G_ARRIVE_TOL:
                    # Reached with no synchronous waiter, or after one returned.
                    self._release_gripper_position_target_locked(goal)
                    command = None
                else:
                    now = time.monotonic()
                    previous_tick = self._gripper_last_tick_monotonic
                    elapsed = 0.0 if previous_tick is None else max(now - previous_tick, 0.0)
                    max_step = self._gripper_position_max_speed_rad_s * elapsed
                    target = self._gripper_target_angle
                    remaining = goal - target
                    if abs(remaining) <= max_step:
                        target = goal
                    elif max_step > 0.0:
                        target += float(np.copysign(max_step, remaining))
                    self._gripper_target_angle = target
                    self._gripper_last_tick_monotonic = now
                    effort = self._gripper_target_effort
                    # ``effort`` caps the whole move, not just the closed
                    # target; without the explicit tau_limit the command torque
                    # fell back to _G_TAU_MAX.
                    tau_ff = effort if abs(target) < 1e-6 else 0.0
                    command = (target, 0.0, _G_KP_MOVE, _G_KD_MOVE, tau_ff, effort)
            elif mode == "grasp_closing":
                command = (
                    0.0,
                    0.0,
                    _G_GRASP_CLOSE_KP,
                    _G_GRASP_CLOSE_KD,
                    self._gripper_close_force,
                    _G_TAU_MAX,
                )
            elif mode == "grasp_holding":
                if self._grasp_hold_expired_locked():
                    self._release_grasp_hold_locked("hold timeout")
                    command = None
                else:
                    command = (
                        self._gripper_hold_angle,
                        0.0,
                        _G_GRASP_HOLD_KP,
                        _G_GRASP_HOLD_KD,
                        self._gripper_hold_force,
                        _G_TAU_MAX,
                    )
            else:
                command = None

            if command is None:
                try:
                    self._gripper_mot.request_feedback()
                    self._gripper_ctrl.poll_feedback_once()
                    self._record_gripper_feedback(self._gripper_mot.get_state())
                except Exception as exc:
                    self._record_gripper_feedback_error(str(exc))
                return

            pos, vel, kp, kd, tau_ff, tau_limit = command
            try:
                self._gripper_safe_mit(pos, vel, kp, kd, tau_ff, tau_limit=tau_limit)
            except Exception as exc:
                self._fail_active_gripper_command_locked(str(exc))

    def _gripper_loop(self) -> None:
        dt = 1.0 / _G_CTRL_RATE
        last = time.perf_counter()
        while not self._gripper_loop_stop.is_set():
            now = time.perf_counter()
            if now - last >= dt:
                last += dt
                self._gripper_tick()
            else:
                time.sleep(1e-4)

    def _start_gripper_loop(self) -> None:
        if self._gripper_loop_running:
            return
        self._gripper_loop_stop.clear()
        self._gripper_loop_thread = threading.Thread(
            target=self._gripper_loop,
            name="rebotarm-gripper-loop",
            daemon=True,
        )
        self._gripper_loop_thread.start()
        self._gripper_loop_running = True

    def _stop_gripper_loop(self) -> None:
        if not self._gripper_loop_running:
            return
        self._gripper_loop_stop.set()
        if self._gripper_loop_thread is not None:
            self._gripper_loop_thread.join(timeout=1.0)
            self._gripper_loop_thread = None
        self._gripper_loop_running = False
