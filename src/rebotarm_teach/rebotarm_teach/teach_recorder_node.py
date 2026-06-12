from __future__ import annotations

import json
import select
import sys
import termios
import tty
from contextlib import suppress
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import SetTeachRecordPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .parameter_helpers import sensor_qos_kwargs
from .teach_recording import TeachSample, encode_teach_sample, is_quit_key


class TeachRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("teach_recorder_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("sample_rate_hz", 50.0)
        self.declare_parameter("require_gravity_comp", True)
        self.declare_parameter("auto_start_gravity_comp", False)
        self.declare_parameter("auto_start_gravity_comp_retry_sec", 1.0)
        self.declare_parameter("auto_start_gravity_comp_max_attempts", 30)
        self.declare_parameter("keyboard_quit_enabled", True)
        self.declare_parameter("quit_key", "q")
        self.declare_parameter("start_on_launch", True)
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        self._record_path = Path(str(self.get_parameter("record_path").value))
        self._record_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = None
        self._recording_active = False
        self._require_gravity_comp = bool(self.get_parameter("require_gravity_comp").value)
        self._latest_joint_state: JointState | None = None
        self._arm_state = ""
        self._motor_status: dict[str, int] = {}
        self._samples_written = 0
        self._first_sample_stamp: float | None = None
        self._last_sample_stamp: float | None = None
        self._writing_state = "open"
        self._last_write_error = ""
        self._auto_start_attempts = 0
        self._auto_start_in_flight = False
        self._terminal_settings = None
        self._keyboard_stream = None
        self._owns_keyboard_stream = False
        self._shutdown_requested = False
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/recording_status",
            10,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/start",
            self._handle_start_recording,
        )
        self.create_service(
            SetTeachRecordPath,
            f"/{self._arm_namespace}/teleop/teach_record/set_path",
            self._handle_set_record_path,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/stop",
            self._handle_stop_recording,
        )
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        arm_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_subscription(
            ArmStatus,
            f"/{self._arm_namespace}/arm_status",
            self._on_arm_status,
            arm_status_qos,
        )
        for joint_name in self._joint_names:
            self.create_subscription(
                JointMotorState,
                f"/{self._arm_namespace}/joints/{joint_name}/state",
                self._on_motor_state,
                sensor_qos,
            )
        self._gravity_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/start",
        )
        if bool(self.get_parameter("auto_start_gravity_comp").value):
            retry_sec = max(float(self.get_parameter("auto_start_gravity_comp_retry_sec").value), 0.2)
            self.create_timer(retry_sec, self._try_auto_start_gravity_comp)
        period = 1.0 / max(float(self.get_parameter("sample_rate_hz").value), 1.0)
        self.create_timer(period, self._write_sample)
        if bool(self.get_parameter("keyboard_quit_enabled").value):
            self._setup_keyboard_quit()
            self.create_timer(0.05, self._poll_quit_key)
        if bool(self.get_parameter("start_on_launch").value):
            self._start_recording(truncate=False)
            message = f"recording to {self._record_path}; press q to stop"
        else:
            message = f"teach recorder idle; service start writes to {self._record_path}"
        self._publish_status("ready" if self._recording_active else "idle", message)

    def _start_recording(self, *, truncate: bool) -> None:
        if self._handle is not None:
            return
        self._record_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "w" if truncate else "a"
        self._handle = self._record_path.open(mode, encoding="utf-8")
        self._recording_active = True
        self._samples_written = 0
        self._first_sample_stamp = None
        self._last_sample_stamp = None
        self._writing_state = "open"
        self._last_write_error = ""

    def _normalize_record_path(self, value: str) -> Path:
        raw = str(value).strip()
        if not raw:
            raw = "teach_record"
        raw = raw.replace("\\", "/")
        name = Path(raw).name
        if not name.endswith(".jsonl"):
            name = f"{name}.jsonl"
        if name in (".jsonl", "/", "") or ".." in Path(name).parts:
            raise ValueError("invalid teach record file name")
        return Path("teleop_records") / name

    def _stop_recording(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None
        self._recording_active = False
        self._writing_state = "stopped"

    def _handle_start_recording(self, _request, response):
        if self._recording_active:
            response.success = True
            response.message = f"teach recording already active: {self._record_path}"
            self._publish_status("recording", response.message)
            return response
        try:
            self._start_recording(truncate=True)
            response.success = True
            response.message = f"teach recording started: {self._record_path}"
            self._publish_status("starting", response.message)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._publish_status("error", f"failed to start teach recording: {exc}")
        return response

    def _handle_set_record_path(self, request, response):
        if self._recording_active:
            response.success = False
            response.message = "cannot change record path while recording"
            response.normalized_path = str(self._record_path)
            return response
        try:
            self._record_path = self._normalize_record_path(request.record_path)
            self._record_path.parent.mkdir(parents=True, exist_ok=True)
            response.success = True
            response.message = f"teach record path set: {self._record_path}"
            response.normalized_path = str(self._record_path)
            self._publish_status("idle", response.message)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.normalized_path = str(self._record_path)
        return response

    def _handle_stop_recording(self, _request, response):
        if not self._recording_active:
            response.success = True
            response.message = "teach recording already stopped"
            self._publish_status("stopped", response.message)
            return response
        self._stop_recording()
        response.success = True
        response.message = f"teach recording stopped: samples={self._samples_written}"
        self._publish_status("stopped", response.message)
        return response

    def _setup_keyboard_quit(self) -> None:
        try:
            self._keyboard_stream = sys.stdin
            if not self._keyboard_stream.isatty():
                tty_path = Path("/dev/tty")
                if tty_path.exists():
                    self._keyboard_stream = tty_path.open("r", encoding="utf-8", buffering=1)
                    self._owns_keyboard_stream = True
            if self._keyboard_stream.isatty():
                self._terminal_settings = termios.tcgetattr(self._keyboard_stream)
                tty.setcbreak(self._keyboard_stream.fileno())
                self.get_logger().info("press q to stop teach recording")
            else:
                self.get_logger().warn("keyboard quit unavailable: stdin is not a terminal")
        except Exception as exc:
            self.get_logger().warn(f"keyboard quit unavailable: {exc}")
            self._keyboard_stream = None

    def _poll_quit_key(self) -> None:
        if self._shutdown_requested:
            return
        if self._keyboard_stream is None:
            return
        try:
            readable, _, _ = select.select([self._keyboard_stream], [], [], 0.0)
            if not readable:
                return
            key = self._keyboard_stream.read(1)
        except Exception:
            return
        if key == "":
            return
        if is_quit_key(key, quit_key=str(self.get_parameter("quit_key").value)):
            self._shutdown_requested = True
            self._publish_status("stopped", "quit key pressed; stopping recorder")
            self.get_logger().info("quit key pressed; stopping teach recorder")
            self._stop_recording()
            if rclpy.ok():
                rclpy.shutdown()

    def _try_auto_start_gravity_comp(self) -> None:
        if self._shutdown_requested or self._arm_state == "GRAVITY_COMP":
            return
        if self._auto_start_in_flight:
            return
        max_attempts = int(self.get_parameter("auto_start_gravity_comp_max_attempts").value)
        if max_attempts > 0 and self._auto_start_attempts >= max_attempts:
            self._publish_status("waiting", "gravity compensation start service unavailable; start manually")
            return
        if not self._gravity_start_client.wait_for_service(timeout_sec=0.0):
            self._auto_start_attempts += 1
            self._publish_status(
                "waiting",
                f"waiting for gravity compensation start service; attempt={self._auto_start_attempts}",
            )
            return
        self._auto_start_attempts += 1
        self._auto_start_in_flight = True
        future = self._gravity_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_auto_start_gravity_comp_done)

    def _on_auto_start_gravity_comp_done(self, future) -> None:
        self._auto_start_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status("waiting", f"gravity compensation start failed: {exc}")
            return
        if bool(getattr(response, "success", False)):
            self._publish_status("starting", "gravity compensation start requested")
        else:
            message = str(getattr(response, "message", "gravity compensation start rejected"))
            self._publish_status("waiting", message)

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _on_arm_status(self, msg: ArmStatus) -> None:
        self._arm_state = str(msg.state_machine)

    def _on_motor_state(self, msg: JointMotorState) -> None:
        self._motor_status[str(msg.joint_name)] = int(msg.status_code)

    def _write_sample(self) -> None:
        if not self._recording_active:
            self._writing_state = "idle"
            self._publish_status("idle", "teach recorder idle")
            return
        joint_state = self._latest_joint_state
        if joint_state is None:
            self._writing_state = "waiting_joint_states"
            self._publish_status("waiting", "waiting for joint_states")
            return
        if self._require_gravity_comp and self._arm_state != "GRAVITY_COMP":
            self._writing_state = "waiting_gravity_comp"
            self._publish_status("waiting", "waiting for GRAVITY_COMP state")
            return
        sample_stamp = self.get_clock().now().nanoseconds / 1_000_000_000.0
        sample = TeachSample(
            stamp=sample_stamp,
            joint_names=tuple(str(v) for v in joint_state.name),
            positions=tuple(float(v) for v in joint_state.position),
            velocities=tuple(float(v) for v in joint_state.velocity),
            efforts=tuple(float(v) for v in joint_state.effort),
            motor_status=dict(self._motor_status),
            arm_state=self._arm_state,
        )
        if self._first_sample_stamp is None:
            self._first_sample_stamp = sample_stamp
        self._last_sample_stamp = sample_stamp
        try:
            if self._handle is None:
                raise OSError("record file is not open")
            self._handle.write(encode_teach_sample(sample) + "\n")
            self._handle.flush()
        except OSError as exc:
            self._writing_state = "error"
            self._last_write_error = str(exc)
            self._publish_status("error", f"failed to write teach sample: {exc}")
            return
        self._writing_state = "writing"
        self._last_write_error = ""
        self._samples_written += 1
        self._publish_status("recording", f"samples={self._samples_written}")

    def _publish_status(self, state: str, message: str) -> None:
        msg = String()
        elapsed_sec = 0.0
        if self._first_sample_stamp is not None and self._last_sample_stamp is not None:
            elapsed_sec = max(0.0, self._last_sample_stamp - self._first_sample_stamp)
        actual_sample_rate_hz = 0.0
        if elapsed_sec > 0.0 and self._samples_written > 1:
            actual_sample_rate_hz = (self._samples_written - 1) / elapsed_sec
        file_size_bytes = 0
        with suppress(OSError):
            file_size_bytes = int(self._record_path.stat().st_size)
        if state == "stopped":
            self._writing_state = "stopped"
        msg.data = json.dumps(
            {
                "state": state,
                "message": message,
                "record_path": str(self._record_path),
                "samples": self._samples_written,
                "elapsed_sec": elapsed_sec,
                "sample_rate_hz": float(self.get_parameter("sample_rate_hz").value),
                "actual_sample_rate_hz": actual_sample_rate_hz,
                "last_sample_time": self._last_sample_stamp,
                "file_size_bytes": file_size_bytes,
                "writing_state": self._writing_state,
                "last_write_error": self._last_write_error,
                "arm_state": self._arm_state,
                "require_gravity_comp": self._require_gravity_comp,
                "gravity_comp_active": self._arm_state == "GRAVITY_COMP",
            },
            separators=(",", ":"),
        )
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._stop_recording()
            if self._terminal_settings is not None:
                stream = self._keyboard_stream if self._keyboard_stream is not None else sys.stdin
                termios.tcsetattr(stream, termios.TCSADRAIN, self._terminal_settings)
            if self._owns_keyboard_stream and self._keyboard_stream is not None:
                self._keyboard_stream.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()
