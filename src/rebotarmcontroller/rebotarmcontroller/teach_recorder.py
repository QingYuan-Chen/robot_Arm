from __future__ import annotations

import json
import time
from contextlib import suppress
from pathlib import Path

from rebotarm_msgs.srv import SetTeachRecordPath
from std_msgs.msg import String
from std_srvs.srv import Trigger


class InternalTeachRecorder:
    """Record teach samples from the controller's live HardwareManager state."""

    def __init__(
        self,
        node,
        hardware,
        namespace: str,
        *,
        record_path: str,
        rate_hz: float,
        require_gravity_comp: bool,
    ) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace.strip("/")
        self._record_path = self._normalize_record_path(record_path)
        self._rate_hz = max(float(rate_hz), 1.0)
        self._require_gravity_comp = bool(require_gravity_comp)
        self._handle = None
        self._samples_written = 0
        self._recording_active = False
        self._first_sample_stamp: float | None = None
        self._last_sample_stamp: float | None = None
        self._start_monotonic: float | None = None
        self._writing_state = "idle"
        self._last_write_error = ""

        self._status_pub = node.create_publisher(
            String,
            f"/{namespace}/teleop/recording_status",
            10,
            callback_group=node.reentrant_group,
        )
        node.create_service(
            Trigger,
            f"/{namespace}/teleop/teach_record/start",
            self._handle_start_recording,
            callback_group=node.slow_group,
        )
        node.create_service(
            SetTeachRecordPath,
            f"/{namespace}/teleop/teach_record/set_path",
            self._handle_set_record_path,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            f"/{namespace}/teleop/teach_record/stop",
            self._handle_stop_recording,
            callback_group=node.slow_group,
        )
        node.create_timer(
            1.0 / self._rate_hz,
            self._write_sample,
            callback_group=node.reentrant_group,
        )
        self._publish_status("idle", f"controller teach recorder idle: {self._record_path}")

    @staticmethod
    def _normalize_record_path(value: str) -> Path:
        raw = str(value or "").strip().replace("\\", "/")
        if not raw:
            raw = "teach_record"
        name = Path(raw).name
        if not name.endswith(".jsonl"):
            name = f"{name}.jsonl"
        if name in (".jsonl", "/", "") or ".." in Path(name).parts:
            raise ValueError("invalid teach record file name")
        return Path("teleop_records") / name

    def _start_recording(self) -> None:
        if self._handle is not None:
            return
        self._record_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self._record_path.open("w", encoding="utf-8")
        self._samples_written = 0
        self._first_sample_stamp = None
        self._last_sample_stamp = None
        self._start_monotonic = time.monotonic()
        self._writing_state = "open"
        self._last_write_error = ""
        self._recording_active = True

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
            self._start_recording()
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

    def _write_sample(self) -> None:
        if not self._recording_active:
            return
        arm_state = str(self._hardware.state_machine)
        if self._require_gravity_comp and arm_state != "GRAVITY_COMP":
            self._writing_state = "waiting_gravity_comp"
            self._publish_status("waiting", "waiting for GRAVITY_COMP state")
            return

        try:
            pos, vel, effort = self._hardware.get_joint_state()
            status_codes = self._hardware.get_joint_status_codes()
        except Exception as exc:
            self._writing_state = "error"
            self._last_write_error = str(exc)
            self._publish_status("error", f"joint state read failed: {exc}")
            return

        start = self._start_monotonic if self._start_monotonic is not None else time.monotonic()
        sample_stamp = time.monotonic() - start
        names = self._hardware.joint_names
        payload = {
            "stamp": sample_stamp,
            "joint_names": list(names),
            "positions": [float(v) for v in pos],
            "velocities": [float(v) for v in vel],
            "efforts": [float(v) for v in effort],
            "motor_status": {
                str(name): int(status_codes[i]) if i < len(status_codes) else 0
                for i, name in enumerate(names)
            },
            "arm_state": arm_state,
        }
        if self._first_sample_stamp is None:
            self._first_sample_stamp = sample_stamp
        self._last_sample_stamp = sample_stamp
        try:
            if self._handle is None:
                raise OSError("record file is not open")
            self._handle.write(json.dumps(payload, separators=(",", ":")) + "\n")
            self._handle.flush()
        except OSError as exc:
            self._writing_state = "error"
            self._last_write_error = str(exc)
            self._publish_status("error", f"failed to write teach sample: {exc}")
            return
        self._samples_written += 1
        self._writing_state = "writing"
        self._last_write_error = ""
        self._publish_status("recording", f"samples={self._samples_written}")

    def _publish_status(self, state: str, message: str) -> None:
        elapsed_sec = 0.0
        if self._first_sample_stamp is not None and self._last_sample_stamp is not None:
            elapsed_sec = max(0.0, self._last_sample_stamp - self._first_sample_stamp)
        actual_sample_rate_hz = 0.0
        if elapsed_sec > 0.0 and self._samples_written > 1:
            actual_sample_rate_hz = (self._samples_written - 1) / elapsed_sec
        file_size_bytes = 0
        with suppress(OSError):
            file_size_bytes = int(self._record_path.stat().st_size)
        msg = String()
        msg.data = json.dumps(
            {
                "state": state,
                "message": message,
                "record_path": str(self._record_path),
                "samples": self._samples_written,
                "elapsed_sec": elapsed_sec,
                "sample_rate_hz": self._rate_hz,
                "actual_sample_rate_hz": actual_sample_rate_hz,
                "last_sample_time": self._last_sample_stamp,
                "file_size_bytes": file_size_bytes,
                "writing_state": self._writing_state,
                "last_write_error": self._last_write_error,
                "arm_state": str(self._hardware.state_machine),
                "require_gravity_comp": self._require_gravity_comp,
                "gravity_comp_active": str(self._hardware.state_machine) == "GRAVITY_COMP",
                "source": "controller_internal",
            },
            separators=(",", ":"),
        )
        self._status_pub.publish(msg)

    def shutdown(self) -> None:
        self._stop_recording()
