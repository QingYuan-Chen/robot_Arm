from __future__ import annotations

from pathlib import Path

import pytest

import rebot_b601_mapping.ports as ports_module
from rebot_b601_mapping.ports import (
    PortIdentity,
    assert_ports_unoccupied,
    assert_same_port,
)


def test_assert_ports_unoccupied_reports_every_owner(tmp_path: Path) -> None:
    device = tmp_path / "ttyACM0"
    device.touch()
    for pid, fd in (("4242", "7"), ("4343", "9")):
        fd_dir = tmp_path / pid / "fd"
        fd_dir.mkdir(parents=True)
        (fd_dir / fd).symlink_to(device)

    with pytest.raises(RuntimeError, match=r"ttyACM0.*4242.*4343"):
        assert_ports_unoccupied([str(device)], proc_root=tmp_path)


def test_assert_ports_unoccupied_ignores_other_devices(tmp_path: Path) -> None:
    requested = tmp_path / "ttyUSB0"
    other = tmp_path / "ttyACM0"
    requested.touch()
    other.touch()
    fd_dir = tmp_path / "4242" / "fd"
    fd_dir.mkdir(parents=True)
    (fd_dir / "7").symlink_to(other)

    assert_ports_unoccupied([str(requested)], proc_root=tmp_path)


def test_assert_same_port_rejects_device_identity_change(monkeypatch) -> None:
    expected = PortIdentity(
        requested_path="/dev/ttyUSB0",
        resolved_path="/dev/ttyUSB0",
        device_id=188,
    )
    changed = PortIdentity(
        requested_path="/dev/ttyUSB0",
        resolved_path="/dev/ttyUSB1",
        device_id=189,
    )
    monkeypatch.setattr(
        ports_module.PortIdentity,
        "capture",
        classmethod(lambda cls, path: changed),
    )

    with pytest.raises(RuntimeError, match="串口设备身份发生变化"):
        assert_same_port(expected)


def test_port_identity_rejects_non_character_device(tmp_path: Path) -> None:
    regular_file = tmp_path / "not-a-serial-port"
    regular_file.touch()

    with pytest.raises(ValueError, match="不是字符设备"):
        PortIdentity.capture(str(regular_file))
