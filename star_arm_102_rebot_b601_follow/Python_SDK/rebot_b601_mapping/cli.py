from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import tempfile
import time
from collections.abc import Callable, Sequence
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .follower_reader import FollowerReader
from .leader_reader import LeaderReader
from .live_follow import StopRequest, run_live_follow
from .mapping import (
    apply_confirmation,
    capture_baseline,
    infer_direction,
    map_virtual_follower,
    validate_paired_sample,
)
from .models import DirectionEvidence
from .models import FollowerSample, LeaderSample, load_mapping_config
from .ports import assert_ports_unoccupied
from .rviz_preview import run_rviz_preview
from .safety_supervisor import FollowState


DEFAULT_CONFIG = Path(__file__).with_name("mapping.example.json")
DEFAULT_LIVE_CONFIG = Path(__file__).with_name("live_follow.example.json")


def _make_sigint_handler(stop_request: StopRequest):
    def _handle(signum, frame) -> None:
        del signum, frame
        if stop_request.normal_requested:
            stop_request.request_emergency()
            print("再次收到 Ctrl+C：请求紧急停机。", file=sys.stderr)
        else:
            stop_request.request_normal()
            print("收到 Ctrl+C：停止跟随并受控返回安全位。", file=sys.stderr)

    return _handle


def _atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_name = temporary.name
            json.dump(data, temporary, ensure_ascii=False, indent=2, allow_nan=False)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass


def _mapping_evidence(config) -> list[dict[str, Any]]:
    return [
        {
            "leader_id": joint.leader_id,
            "follower_name": joint.follower_name,
            "sign": joint.sign,
            "scale": joint.scale,
            "verified": joint.verified,
        }
        for joint in config.arm_joints + (config.gripper,)
    ]


def _sample_evidence(
    leader: LeaderSample,
    follower: FollowerSample,
    virtual_positions: Sequence[float],
) -> dict[str, Any]:
    return {
        "leader_timestamp_s": leader.timestamp_s,
        "leader_angles_deg": list(leader.angles_deg),
        "follower_timestamp_s": follower.timestamp_s,
        "follower": [
            {
                "name": motor.name,
                "position_rad": motor.position_rad,
                "velocity_rad_s": motor.velocity_rad_s,
                "torque_nm": motor.torque_nm,
                "status_code": motor.status_code,
            }
            for motor in follower.motors
        ],
        "virtual_follower_rad": list(virtual_positions),
    }


def _close_readers(follower, leader, *, error_active: bool) -> None:
    errors: list[str] = []
    for label, reader in (("从臂", follower), ("引导臂", leader)):
        try:
            reader.close()
        except Exception as exc:
            errors.append(f"{label}通信关闭失败：{exc}")
    if errors and not error_active:
        raise RuntimeError("；".join(errors))


def run_snapshot(
    *,
    leader_port: str,
    follower_port: str,
    config_path: Path,
    output_path: Path,
    baseline_samples: int = 5,
    sample_count: int = 20,
    interval_s: float = 0.02,
    leader_factory: Callable[[str], Any] = LeaderReader,
    follower_factory: Callable[[str], Any] = FollowerReader,
    port_checker: Callable[[Sequence[str]], None] = assert_ports_unoccupied,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """采集只读快照并写入 JSON 证据。"""

    config = load_mapping_config(Path(config_path))
    if baseline_samples < config.thresholds.sign_window_size:
        raise ValueError(
            f"baseline_samples 至少为 {config.thresholds.sign_window_size}"
        )
    if sample_count < 1:
        raise ValueError("sample_count 必须大于零")
    if interval_s < 0.0:
        raise ValueError("interval_s 不得为负数")

    port_checker((leader_port, follower_port))
    leader = leader_factory(leader_port)
    follower = follower_factory(follower_port)
    try:
        leader.open()
        follower.open()
        baseline_window: list[tuple[LeaderSample, FollowerSample]] = []
        for index in range(baseline_samples):
            baseline_window.append((leader.read_sample(), follower.read_sample()))
            if interval_s and index + 1 < baseline_samples:
                sleep(interval_s)
        baseline_now = float(clock())
        baseline = capture_baseline(baseline_window, config, now_s=baseline_now)

        samples: list[dict[str, Any]] = []
        for index in range(sample_count):
            leader_sample = leader.read_sample()
            follower_sample = follower.read_sample()
            validate_paired_sample(
                leader_sample,
                follower_sample,
                config,
                now_s=float(clock()),
            )
            mapped = map_virtual_follower(leader_sample, baseline, config)
            samples.append(
                _sample_evidence(
                    leader_sample,
                    follower_sample,
                    mapped.positions_rad,
                )
            )
            if interval_s and index + 1 < sample_count:
                sleep(interval_s)

        evidence: dict[str, Any] = {
            "schema_version": 1,
            "mode": "snapshot",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "leader_port": str(leader_port),
            "follower_port": str(follower_port),
            "sample_count": sample_count,
            "baseline": {
                "captured_at_s": baseline.captured_at_s,
                "leader_angles_deg": list(baseline.leader_angles_deg),
                "follower_positions_rad": list(baseline.follower_positions_rad),
            },
            "mapping": _mapping_evidence(config),
            "samples": samples,
        }
        _atomic_write_json(Path(output_path), evidence)
        return evidence
    finally:
        _close_readers(
            follower,
            leader,
            error_active=sys.exc_info()[0] is not None,
        )


def persist_verified_direction(
    config_path: Path,
    evidence: DirectionEvidence,
    *,
    confirmed: bool,
) -> bool:
    """经人工确认后，原子写入一个关节的方向验证证据。"""

    if not confirmed or not evidence.verified:
        return False
    if evidence.inferred_sign != evidence.candidate_sign:
        return False
    path = Path(config_path)
    data = json.loads(path.read_text(encoding="utf-8"))
    matches = [
        item
        for item in data.get("mapping", [])
        if item.get("follower_name") == evidence.follower_name
    ]
    if len(matches) != 1:
        raise ValueError(f"配置中无法唯一定位 {evidence.follower_name}")
    item = matches[0]
    if item.get("sign") != evidence.candidate_sign:
        raise ValueError(f"{evidence.follower_name} 候选符号在验证期间发生变化")
    item["verified"] = True
    item["evidence"] = {
        "observed_at_s": evidence.observed_at_s,
        "leader_delta_rad": evidence.leader_delta_rad,
        "follower_delta_rad": evidence.follower_delta_rad,
        "inferred_sign": evidence.inferred_sign,
        "candidate_sign": evidence.candidate_sign,
        "consistent": evidence.consistent,
        "confirmed": evidence.confirmed,
    }
    _atomic_write_json(path, data)
    return True


def run_calibration(
    *,
    selected_joint: str,
    leader_port: str,
    follower_port: str,
    config_path: Path,
    output_path: Path,
    baseline_samples: int = 5,
    window_samples: int = 5,
    interval_s: float = 0.02,
    leader_factory: Callable[[str], Any] = LeaderReader,
    follower_factory: Callable[[str], Any] = FollowerReader,
    port_checker: Callable[[Sequence[str]], None] = assert_ports_unoccupied,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    input_fn: Callable[[str], str] = input,
    print_fn: Callable[[str], None] = print,
) -> dict[str, Any]:
    """交互采集一个关节的方向证据，只在完整确认后更新配置。"""

    config_path = Path(config_path)
    config = load_mapping_config(config_path)
    joint_names = tuple(joint.follower_name for joint in config.arm_joints)
    if selected_joint not in joint_names:
        raise ValueError("方向标定只接受 joint1..joint6，不接受夹爪")
    if config_path.resolve() == DEFAULT_CONFIG.resolve():
        raise ValueError("不得直接修改 mapping.example.json，请先复制为 mapping.local.json")
    required_window = config.thresholds.sign_window_size
    if baseline_samples < required_window or window_samples < required_window:
        raise ValueError(f"基线和方向窗口样本数均不得少于 {required_window}")
    if interval_s < 0.0:
        raise ValueError("interval_s 不得为负数")

    port_checker((leader_port, follower_port))
    leader = leader_factory(leader_port)
    follower = follower_factory(follower_port)
    try:
        leader.open()
        follower.open()
        baseline_window: list[tuple[LeaderSample, FollowerSample]] = []
        for index in range(baseline_samples):
            baseline_window.append((leader.read_sample(), follower.read_sample()))
            if interval_s and index + 1 < baseline_samples:
                sleep(interval_s)
        baseline = capture_baseline(
            baseline_window,
            config,
            now_s=float(clock()),
        )

        selected_index = joint_names.index(selected_joint)
        spec = config.arm_joints[selected_index]
        print_fn(f"当前标定关节：{selected_joint}，候选符号：{spec.sign:+d}")
        print_fn(
            "基线：引导臂 "
            f"{baseline.leader_angles_deg[spec.leader_id]:.6f} 度；"
            f"从臂 {baseline.follower_positions_rad[selected_index]:.6f} 弧度"
        )
        print_fn("请托住失能机械臂，只移动两台机械臂的当前指定关节。")
        input_fn("移动到可辨识的新位置并保持稳定后，按回车开始采样：")

        paired_window: list[tuple[LeaderSample, FollowerSample]] = []
        for index in range(window_samples):
            paired_window.append((leader.read_sample(), follower.read_sample()))
            if interval_s and index + 1 < window_samples:
                sleep(interval_s)
        inferred = infer_direction(
            baseline,
            paired_window,
            selected_joint,
            config,
            now_s=float(clock()),
        )
        print_fn(
            f"引导臂增量：{inferred.leader_delta_rad:+.6f} 弧度；"
            f"从臂增量：{inferred.follower_delta_rad:+.6f} 弧度；"
            f"推断符号：{inferred.inferred_sign:+d}"
        )
        confirmation = input_fn("确认该关节方向映射正确请输入“确认”，否则直接回车：")
        confirmed = apply_confirmation(inferred, confirmation == "确认")
        persisted = persist_verified_direction(
            config_path,
            confirmed,
            confirmed=confirmation == "确认",
        )
        result: dict[str, Any] = {
            "schema_version": 1,
            "mode": "calibrate-directions",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "joint": selected_joint,
            "leader_port": str(leader_port),
            "follower_port": str(follower_port),
            "baseline": {
                "captured_at_s": baseline.captured_at_s,
                "leader_angles_deg": list(baseline.leader_angles_deg),
                "follower_positions_rad": list(baseline.follower_positions_rad),
            },
            "direction": asdict(confirmed),
            "persisted": persisted,
            "window": [
                _sample_evidence(
                    leader_sample,
                    follower_sample,
                    (),
                )
                for leader_sample, follower_sample in paired_window
            ],
        }
        _atomic_write_json(Path(output_path), result)
        return result
    finally:
        _close_readers(
            follower,
            leader,
            error_active=sys.exc_info()[0] is not None,
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Star Arm 102-LD 到 reBot B601 的方向映射和实时跟随工具"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    snapshot = subparsers.add_parser("snapshot", help="采集只读关节快照")
    snapshot.add_argument("--leader-port", default="/dev/ttyUSB0")
    snapshot.add_argument("--follower-port", default="/dev/ttyACM0")
    snapshot.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    snapshot.add_argument("--output", type=Path, required=True)
    snapshot.add_argument("--baseline-samples", type=int, default=5)
    snapshot.add_argument("--samples", type=int, default=20)
    snapshot.add_argument("--interval-s", type=float, default=0.02)
    calibrate = subparsers.add_parser(
        "calibrate-directions",
        help="人工移动并验证一个关节的方向",
    )
    calibrate.add_argument(
        "--joint",
        required=True,
        choices=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
    )
    calibrate.add_argument("--leader-port", default="/dev/ttyUSB0")
    calibrate.add_argument("--follower-port", default="/dev/ttyACM0")
    calibrate.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("mapping.local.json"),
    )
    calibrate.add_argument("--output", type=Path, required=True)
    calibrate.add_argument("--baseline-samples", type=int, default=5)
    calibrate.add_argument("--window-samples", type=int, default=5)
    calibrate.add_argument("--interval-s", type=float, default=0.02)
    preview = subparsers.add_parser(
        "rviz-preview",
        help="只读取引导臂并驱动 RViz 虚拟从臂",
    )
    preview.add_argument("--leader-port", default="/dev/ttyUSB0")
    preview.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    preview.add_argument("--topic", default="/mapping_preview/joint_states")
    preview.add_argument("--baseline-samples", type=int, default=5)
    preview.add_argument("--rate-hz", type=float, default=20.0)
    preview.add_argument("--max-samples", type=int)
    follow = subparsers.add_parser(
        "follow",
        help="执行 Python SDK 六轴实时主从跟随",
    )
    follow.add_argument("--leader-port", default="/dev/ttyUSB0")
    follow.add_argument("--follower-port", default="/dev/ttyACM0")
    follow.add_argument("--mapping-config", type=Path, default=DEFAULT_CONFIG)
    follow.add_argument("--live-config", type=Path, default=DEFAULT_LIVE_CONFIG)
    follow.add_argument(
        "--log",
        type=Path,
        default=Path("/tmp/stararm-rebot-live-follow.jsonl"),
    )
    follow.add_argument("--speed-rad-s", type=float)
    follow.add_argument("--confirm-live-motion", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.command == "follow":
            stop_request = StopRequest()
            previous_handler = signal.getsignal(signal.SIGINT)
            signal.signal(signal.SIGINT, _make_sigint_handler(stop_request))
            try:
                summary = run_live_follow(
                    leader_port=args.leader_port,
                    follower_port=args.follower_port,
                    mapping_path=args.mapping_config,
                    live_config_path=args.live_config,
                    log_path=args.log,
                    confirmed=bool(args.confirm_live_motion),
                    speed_rad_s=args.speed_rad_s,
                    stop_request=stop_request,
                )
            finally:
                signal.signal(signal.SIGINT, previous_handler)
            if not args.confirm_live_motion:
                print("静态预检完成：未使能、未发送运动命令。")
                print(f"证据文件：{summary.log_path}")
                return 0
            print(f"实时跟随结束：{summary.final_state.name}")
            print(f"有效控制周期：{summary.cycles}")
            print(f"安全位验证：{summary.safe_home_verified}")
            print(f"失能验证：{summary.disable_verified}")
            print(f"证据文件：{summary.log_path}")
            if summary.final_state is FollowState.CRITICAL_STOP:
                return 130 if stop_request.emergency_requested else 2
            return 0
        if args.command == "snapshot":
            evidence = run_snapshot(
                leader_port=args.leader_port,
                follower_port=args.follower_port,
                config_path=args.config,
                output_path=args.output,
                baseline_samples=args.baseline_samples,
                sample_count=args.samples,
                interval_s=args.interval_s,
            )
            latest = evidence["samples"][-1]
            print("只读快照完成，未发送任何生命周期或运动指令。")
            print(f"有效样本：{evidence['sample_count']}")
            print(f"引导臂角度（度）：{latest['leader_angles_deg']}")
            print(
                "从臂位置（弧度）："
                f"{[item['position_rad'] for item in latest['follower']]}"
            )
            print(f"虚拟从臂目标（弧度）：{latest['virtual_follower_rad']}")
            print(f"证据文件：{args.output}")
            return 0
        if args.command == "calibrate-directions":
            evidence = run_calibration(
                selected_joint=args.joint,
                leader_port=args.leader_port,
                follower_port=args.follower_port,
                config_path=args.config,
                output_path=args.output,
                baseline_samples=args.baseline_samples,
                window_samples=args.window_samples,
                interval_s=args.interval_s,
            )
            if evidence["persisted"]:
                print(f"{args.joint} 方向验证已确认并写入配置。")
                print(f"证据文件：{args.output}")
                return 0
            print(f"{args.joint} 未通过确认，配置未修改。", file=sys.stderr)
            print(f"证据文件：{args.output}", file=sys.stderr)
            return 3
        if args.command == "rviz-preview":
            result = run_rviz_preview(
                leader_port=args.leader_port,
                config_path=args.config,
                topic=args.topic,
                baseline_samples=args.baseline_samples,
                rate_hz=args.rate_hz,
                max_samples=args.max_samples,
            )
            print(f"RViz 预览结束，有效发布样本：{result['published_samples']}")
            return 0
    except KeyboardInterrupt:
        if args.command == "follow":
            print("用户中断了实时跟随。", file=sys.stderr)
        else:
            print("用户中断：通信已关闭，未发送控制指令。", file=sys.stderr)
        return 130
    except Exception as exc:
        prefix = "实时跟随失败" if args.command == "follow" else "只读映射失败"
        print(f"{prefix}：{exc}", file=sys.stderr)
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
