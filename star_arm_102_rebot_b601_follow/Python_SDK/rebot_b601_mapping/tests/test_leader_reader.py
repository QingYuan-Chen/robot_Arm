from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from rebot_b601_mapping.leader_reader import LeaderReader
from rebot_b601_mapping.ports import PortIdentity


class FakeSerial:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class FakeManager:
    def __init__(self, angles=None) -> None:
        values = tuple(range(7)) if angles is None else tuple(angles)
        self.states = {
            servo_id: SimpleNamespace(
                id=servo_id,
                is_online=True,
                angle_monitor=value,
            )
            for servo_id, value in enumerate(values)
        }
        self.calls: list[tuple[str, tuple[int, ...], bool]] = []

    def send_sync_servo_monitor(self, servo_ids, realtime=False):
        self.calls.append(("send_sync_servo_monitor", tuple(servo_ids), realtime))
        return dict(self.states)


def make_reader(fake_serial: FakeSerial, fake_manager: FakeManager) -> LeaderReader:
    return LeaderReader(
        "/dev/fake-leader",
        serial_factory=lambda **kwargs: fake_serial,
        manager_factory=lambda serial_obj: fake_manager,
        clock=lambda: 12.5,
        identity_factory=lambda path: PortIdentity(path, path, 188),
        identity_checker=lambda identity: None,
    )


def test_read_sample_uses_monitor_query_only() -> None:
    fake_serial = FakeSerial()
    fake_manager = FakeManager()
    serial_arguments = {}

    def serial_factory(**kwargs):
        serial_arguments.update(kwargs)
        return fake_serial

    reader = LeaderReader(
        "/dev/fake-leader",
        serial_factory=serial_factory,
        manager_factory=lambda serial_obj: fake_manager,
        clock=lambda: 12.5,
        identity_factory=lambda path: PortIdentity(path, path, 188),
        identity_checker=lambda identity: None,
    )
    reader.open()

    sample = reader.read_sample()

    assert fake_manager.calls == [
        ("send_sync_servo_monitor", (0, 1, 2, 3, 4, 5, 6), True)
    ]
    assert sample.timestamp_s == 12.5
    assert sample.angles_deg == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert serial_arguments == {
        "port": "/dev/fake-leader",
        "baudrate": 1_000_000,
        "parity": "N",
        "stopbits": 1,
        "bytesize": 8,
        "timeout": 0.05,
        "exclusive": True,
    }


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda manager: manager.states.pop(3), "ID 3.*缺失"),
        (
            lambda manager: setattr(manager.states[2], "angle_monitor", None),
            "ID 2.*无有效角度",
        ),
        (
            lambda manager: setattr(manager.states[5], "angle_monitor", math.nan),
            "ID 5.*非有限",
        ),
    ],
)
def test_read_sample_rejects_incomplete_or_invalid_monitor_data(
    mutate,
    message: str,
) -> None:
    fake_serial = FakeSerial()
    fake_manager = FakeManager()
    mutate(fake_manager)
    reader = make_reader(fake_serial, fake_manager)
    reader.open()

    with pytest.raises(RuntimeError, match=message):
        reader.read_sample()


def test_reader_rejects_duplicate_open() -> None:
    reader = make_reader(FakeSerial(), FakeManager())
    reader.open()

    with pytest.raises(RuntimeError, match="已经打开"):
        reader.open()


def test_context_exit_closes_serial_on_keyboard_interrupt() -> None:
    fake_serial = FakeSerial()
    reader = make_reader(fake_serial, FakeManager())

    with pytest.raises(KeyboardInterrupt):
        with reader:
            raise KeyboardInterrupt

    assert fake_serial.closed is True
