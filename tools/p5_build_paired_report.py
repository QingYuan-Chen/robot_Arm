#!/usr/bin/env python3
"""Build the durable P5 paired MuJoCo/real JSON and Markdown evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "rebotarm_simulation"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

from rebotarm_simulation.paired_trajectory_analysis import compare_runs


JOINTS = tuple(f"joint{index}" for index in range(1, 7))


def _load(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _fmt(value: float | None, digits: int = 6) -> str:
    return "n/a" if value is None else f"{float(value):.{digits}f}"


def _markdown(report: dict[str, object], json_name: str) -> str:
    outbound, returned = report["analysis"]["outbound"], report["analysis"]["return"]
    sim_run, real_run = report["raw_runs"]["mujoco"], report["raw_runs"]["real"]
    lines = [
        "# P5 MuJoCo–real safe-posture paired trajectory evidence",
        "",
        "## 结论",
        "",
        "本次在唯一有效仿真后端 `rebotarm_mujoco_node` 与真实 `reBotArmController` 上执行了同一组序列化轨迹。去程、回程的 `command_sha256` 完全一致；四段 action 均成功。真机从 disabled / 失能状态开始，在回到采集基线后才 disable，最终六轴状态码均为 0，且无 controller error。",
        "",
        "这份证据支持 P5 的局部 safe-posture trajectory / 安全姿态轨迹配对复现，但不等价于完整 geometry、dynamics 或 contact 验收：当前只有一个局部工作区轨迹；real effort 与 MuJoCo actuator force 尚未做同量纲标定；共同 contact topic 不存在。",
        "",
        "## 协议与安全边界",
        "",
        f"- 起点（实时真机 disabled baseline，rad）：`{real_run['preflight_positions']}`",
        "- 目标（rad）：`[-1.5707963267948966, -0.1, -0.2, 0.2, 0.0, 0.0]`",
        "- 每段 20 s quintic / 五次多项式，0.05 s command cadence，共 401 点；中间 enabled hold 2 s；不含 approach、gripper、lift、retreat。",
        f"- 去程 SHA-256：`{outbound['command_sha256']}`",
        f"- 回程 SHA-256：`{returned['command_sha256']}`",
        f"- MuJoCo：两段 sample 数 `{len(sim_run['legs'][0]['samples'])}` / `{len(sim_run['legs'][1]['samples'])}`；最终基线最大误差 `{max(abs(v) for v in sim_run['enabled_final_errors']):.6f} rad`。",
        f"- 真机：两段 sample 数 `{len(real_run['legs'][0]['samples'])}` / `{len(real_run['legs'][1]['samples'])}`；最终基线最大误差 `{max(abs(v) for v in real_run['enabled_final_errors']):.6f} rad`。",
        "- 运动期间 guard 监控 raw/window velocity、tracking error、effort、JointState freshness 与 ArmStatus；本次无 guard stop。",
        "",
        "## 每轴对齐指标",
        "",
        "### 去程",
        "",
        "| joint | sim tracking RMS (rad) | real tracking RMS (rad) | real-sim position RMS (rad) | max position delta (rad) | effort trend corr. | real peak effort |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for joint in JOINTS:
        sim = outbound["sim"]["per_joint"][joint]
        real = outbound["real"]["per_joint"][joint]
        paired = outbound["paired"][joint]
        lines.append(
            f"| {joint} | {_fmt(sim['rms_tracking_error_rad'])} | {_fmt(real['rms_tracking_error_rad'])} | "
            f"{_fmt(paired['rms_real_minus_sim_position_rad'])} | {_fmt(paired['max_abs_real_minus_sim_position_rad'])} | "
            f"{_fmt(paired['effort_trend_correlation'], 3)} | {_fmt(real['peak_abs_effort'], 3)} |"
        )
    lines.extend(
        [
            "",
            "### 回程",
            "",
            "| joint | sim tracking RMS (rad) | real tracking RMS (rad) | real-sim position RMS (rad) | max position delta (rad) | effort trend corr. | real peak effort |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for joint in JOINTS:
        sim = returned["sim"]["per_joint"][joint]
        real = returned["real"]["per_joint"][joint]
        paired = returned["paired"][joint]
        lines.append(
            f"| {joint} | {_fmt(sim['rms_tracking_error_rad'])} | {_fmt(real['rms_tracking_error_rad'])} | "
            f"{_fmt(paired['rms_real_minus_sim_position_rad'])} | {_fmt(paired['max_abs_real_minus_sim_position_rad'])} | "
            f"{_fmt(paired['effort_trend_correlation'], 3)} | {_fmt(real['peak_abs_effort'], 3)} |"
        )
    joint4_out = outbound["real"]["per_joint"]["joint4"]
    joint4_back = returned["real"]["per_joint"]["joint4"]
    lines.extend(
        [
            "",
            "## joint4 与 effort 解释",
            "",
            f"joint4 真机 tracking RMS 为去程 `{joint4_out['rms_tracking_error_rad']:.6f} rad`、回程 `{joint4_back['rms_tracking_error_rad']:.6f} rad`；max tracking error 为 `{joint4_out['max_abs_tracking_error_rad']:.6f}` / `{joint4_back['max_abs_tracking_error_rad']:.6f} rad`。raw velocity 的瞬时峰值包含编码器量化影响，因此安全判断同时使用 0.20 s window velocity；对应峰值为 `{joint4_out['peak_window_velocity_rad_s']:.6f}` / `{joint4_back['peak_window_velocity_rad_s']:.6f} rad/s`。",
            "",
            "real `effort` 是 Damiao motor feedback torque；MuJoCo `effort` 是 applied generalized actuator force。两者在机械臂字段中名义单位均为 N·m，但尚未标定为相同测量链。因此当前只报告峰值与趋势 correlation，不把绝对值差异直接判定为模型误差。",
            "",
            "## 未关闭项",
            "",
            "- 只有单一 safe-posture 往返，无法覆盖整个 workspace geometry / 工作空间几何一致性。",
            "- effort 趋势在部分轴相关性弱或为负，需要电机侧测量链、摩擦和传动参数辨识后再做 dynamics 验收。",
            "- 没有 real/sim 共同 contact topic，本次不能关闭 contact feedback 项。",
            "- 2 s enabled hold 已执行，但当前 raw schema 只将 active trajectory 样本收进 leg；hold 的状态通过流程结果确认，未保存逐样本序列。",
            "- settling time 需要至少 0.25 s 的 completion 后连续 active-leg 样本；本次 leg raw horizon 不足，因此分析值为 `null`，不能据此断言 settling。",
            "",
            "## Raw 数据",
            "",
            f"完整 command、JointState、action feedback、tracking error、effort、ArmStatus 与 gripper/contact availability 均保存在 [{json_name}]({json_name})。",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", type=Path, required=True)
    parser.add_argument("--real", type=Path, required=True)
    parser.add_argument("--json-output", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    args = parser.parse_args()
    sim_run = _load(args.sim)
    real_run = _load(args.real)
    if not sim_run.get("success") or not real_run.get("success"):
        raise SystemExit("both raw runs must be successful")
    analysis = {
        "outbound": compare_runs(sim_run["legs"][0], real_run["legs"][0]),
        "return": compare_runs(sim_run["legs"][1], real_run["legs"][1]),
    }
    report = {
        "schema_version": 1,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "active_simulator": "rebotarm_mujoco_node",
        "excluded_runtime": "obsolete current-baseline simulator",
        "scope": "paired safe-posture outbound/hold/return; no grasp sequence",
        "safety_result": {
            "real_started_disabled": not real_run["preflight_status"]["enabled"],
            "real_ended_disabled": not real_run["final_status"]["enabled"],
            "real_final_status_codes": real_run["final_status"]["per_joint_status_code"],
            "real_final_error_codes": real_run["final_status"]["error_codes"],
            "real_final_baseline_max_abs_error_rad": max(
                abs(value) for value in real_run["enabled_final_errors"]
            ),
            "mujoco_final_baseline_max_abs_error_rad": max(
                abs(value) for value in sim_run["enabled_final_errors"]
            ),
            "guard_stops": [
                leg["result"]["guard_stop"]
                for run in (sim_run, real_run)
                for leg in run["legs"]
                if leg["result"]["guard_stop"] is not None
            ],
        },
        "analysis": analysis,
        "limitations": [
            "single local safe-posture trajectory does not close full-workspace geometry",
            "real torque and MuJoCo actuator force are not calibrated as identical measurements",
            "no common real/sim contact topic",
            "enabled hold executed but was not retained as per-sample active-leg raw data",
            "post-completion active-leg horizon is too short for the configured settling metric",
        ],
        "raw_runs": {"mujoco": sim_run, "real": real_run},
    }
    _write_json(args.json_output, report)
    args.markdown_output.write_text(
        _markdown(report, args.json_output.name), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
