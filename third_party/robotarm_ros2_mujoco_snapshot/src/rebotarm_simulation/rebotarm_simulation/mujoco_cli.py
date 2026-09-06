from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
import json
import math
from pathlib import Path
import sys

from .mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco


def _nonnegative_finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be a nonnegative finite number")
    return number


def _positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run or manually control reBotArm MuJoCo")
    parser.add_argument("--model", help="path to an MJCF scene (defaults to packaged scene.xml)")
    parser.add_argument("--headless", action="store_true", help="run without an interactive prompt")
    parser.add_argument("--duration", type=_nonnegative_finite, help="simulation seconds to run")
    parser.add_argument("--steps", type=_positive_int, help="number of physics steps to run")
    return parser


def _plain(value):
    if is_dataclass(value):
        return {field.name: _plain(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "__dict__"):
        return {key: _plain(item) for key, item in vars(value).items()}
    raise TypeError(f"cannot serialize {type(value).__name__}")


def _emit(value, stdout) -> None:
    print(json.dumps(_plain(value), ensure_ascii=False, sort_keys=True), file=stdout)


def dispatch_command(sim, line: str, *, paused: bool = False):
    parts = line.split()
    if not parts:
        return paused, None, False
    command, arguments = parts[0].lower(), parts[1:]
    if command == "quit":
        if arguments:
            raise ValueError("usage: quit")
        return paused, "bye", True
    if command == "state":
        if arguments:
            raise ValueError("usage: state")
        return paused, sim.get_state(), False
    if command == "joint":
        if len(arguments) != 2 or arguments[0] not in ARM_JOINT_NAMES:
            raise ValueError("usage: joint NAME VALUE")
        return paused, sim.set_joint_position_targets({arguments[0]: float(arguments[1])}), False
    if command == "joints":
        if len(arguments) != 6:
            raise ValueError("usage: joints J1 J2 J3 J4 J5 J6")
        values = [float(value) for value in arguments]
        return paused, sim.set_joint_position_targets(values), False
    if command == "jog":
        if len(arguments) != 2 or arguments[0] not in ARM_JOINT_NAMES:
            raise ValueError("usage: jog NAME DELTA")
        state = sim.get_state()
        index = ARM_JOINT_NAMES.index(arguments[0])
        target = float(state.joint_positions[index]) + float(arguments[1])
        return paused, sim.set_joint_position_targets({arguments[0]: target}), False
    if command == "gripper":
        if len(arguments) != 1:
            raise ValueError("usage: gripper WIDTH")
        return paused, sim.set_gripper_width(float(arguments[0])), False
    if command == "step":
        if len(arguments) > 1:
            raise ValueError("usage: step [N]")
        count = 1 if not arguments else _positive_int(arguments[0])
        if paused:
            return paused, "paused; step ignored", False
        return paused, sim.step(count), False
    if command == "reset":
        if arguments:
            raise ValueError("usage: reset")
        return paused, sim.reset(), False
    if command == "contacts":
        if arguments:
            raise ValueError("usage: contacts")
        return paused, sim.get_contacts(), False
    if command == "pause":
        if arguments:
            raise ValueError("usage: pause")
        return True, "paused", False
    if command == "resume":
        if arguments:
            raise ValueError("usage: resume")
        return False, "running", False
    raise ValueError(f"unknown command: {command}")


def _run_headless(sim, duration: float | None, steps: int | None):
    if steps is not None:
        sim.step(steps)
    requested_duration = duration
    duration_start = float(sim.get_state().simulation_time)
    if requested_duration is not None:
        target = duration_start + requested_duration
        while float(sim.get_state().simulation_time) + 1e-15 < target:
            sim.step()
    elif steps is None:
        sim.step()
    state = _plain(sim.get_state())
    state["requested_duration"] = requested_duration
    state["achieved_duration"] = (
        float(state["simulation_time"]) - duration_start if requested_duration is not None else None
    )
    return state


def _interactive(sim, stdin, stdout) -> int:
    paused = False
    for line in stdin:
        try:
            paused, result, should_quit = dispatch_command(sim, line, paused=paused)
            if result is not None:
                _emit(result, stdout)
            if should_quit:
                return 0
        except (argparse.ArgumentTypeError, TypeError, ValueError) as exc:
            print(f"error: {exc}", file=stdout)
    return 0


def main(argv=None, *, sim_factory=RebotArmMujoco, stdin=None, stdout=None, stderr=None) -> int:
    stdin = sys.stdin if stdin is None else stdin
    stdout = sys.stdout if stdout is None else stdout
    stderr = sys.stderr if stderr is None else stderr
    args = build_parser().parse_args(argv)
    sim = None
    try:
        sim = sim_factory(args.model)
        if args.headless:
            _emit(_run_headless(sim, args.duration, args.steps), stdout)
            return 0
        return _interactive(sim, stdin, stdout)
    except Exception as exc:
        print(f"MuJoCo CLI error: {exc}", file=stderr)
        return 1
    finally:
        if sim is not None:
            sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
