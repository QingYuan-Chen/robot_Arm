from __future__ import annotations

import os
import stat
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class PortIdentity:
    requested_path: str
    resolved_path: str
    device_id: int

    @classmethod
    def capture(cls, path: str) -> "PortIdentity":
        requested = str(path)
        resolved = str(Path(requested).expanduser().resolve(strict=True))
        metadata = os.stat(resolved)
        if not stat.S_ISCHR(metadata.st_mode):
            raise ValueError(f"{requested} 不是字符设备")
        return cls(
            requested_path=requested,
            resolved_path=resolved,
            device_id=int(metadata.st_rdev),
        )


def assert_same_port(expected: PortIdentity) -> None:
    current = PortIdentity.capture(expected.requested_path)
    if current != expected:
        raise RuntimeError(
            "串口设备身份发生变化："
            f"原为 {expected.resolved_path}({expected.device_id})，"
            f"现为 {current.resolved_path}({current.device_id})"
        )


def assert_ports_unoccupied(
    paths: Sequence[str],
    *,
    proc_root: Path = Path("/proc"),
) -> None:
    """通过 `/proc/*/fd` 检查占用，只报告而不终止任何进程。"""

    targets = {
        str(Path(path).expanduser().resolve(strict=True)): str(path) for path in paths
    }
    owners: dict[str, set[int]] = {target: set() for target in targets}
    for process_dir in proc_root.glob("[0-9]*"):
        try:
            pid = int(process_dir.name)
        except ValueError:
            continue
        fd_dir = process_dir / "fd"
        try:
            descriptors = tuple(fd_dir.iterdir())
        except (FileNotFoundError, PermissionError, OSError):
            continue
        for descriptor in descriptors:
            try:
                resolved = str(descriptor.resolve(strict=True))
            except (FileNotFoundError, PermissionError, OSError):
                continue
            if resolved in owners:
                owners[resolved].add(pid)

    occupied = [
        f"{targets[target]}: PID {','.join(str(pid) for pid in sorted(pids))}"
        for target, pids in owners.items()
        if pids
    ]
    if occupied:
        raise RuntimeError("串口已被占用：" + "；".join(occupied))
