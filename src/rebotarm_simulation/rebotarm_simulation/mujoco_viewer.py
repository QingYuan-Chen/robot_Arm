from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import importlib
import math
from queue import Empty, SimpleQueue
import sys
import time
from typing import Callable, Sequence

from .mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco


HELP = (
    "[ / ] select | 1-6 select | hold J/K joint | hold C/O gripper | "
    "G gravity | H hold | P pos | R zero | T home | Q quit"
)
_RETAINED_UNSAFE_VIEWERS = []


@dataclass(frozen=True)
class ViewerControlState:
    selected_joint: int = 0
    joint_targets: tuple[float, ...] = (0.0,) * 6
    joint_positions: tuple[float, ...] = (0.0,) * 6
    gripper_width: float = 0.09
    paused: bool = False
    joint_delta: int = 0
    gripper_delta: int = 0
    joint_jog_direction: int = 0
    gripper_jog_direction: int = 0
    jog_time_remaining: float = 0.0
    single_step: bool = False
    reset: bool = False
    home: bool = False
    mode: str | None = None
    active_mode: str = "pos_vel"
    quit: bool = False


def reduce_key(state: ViewerControlState, key: str) -> ViewerControlState:
    key = key.lower()
    if key == "]":
        return replace(state, selected_joint=(state.selected_joint + 1) % 6)
    if key == "[":
        return replace(state, selected_joint=(state.selected_joint - 1) % 6)
    if key in "123456":
        return replace(state, selected_joint=int(key) - 1)
    if key == "j":
        return replace(state, joint_delta=state.joint_delta - 1, joint_jog_direction=-1)
    if key == "k":
        return replace(state, joint_delta=state.joint_delta + 1, joint_jog_direction=1)
    if key == "c":
        return replace(state, gripper_delta=state.gripper_delta - 1, gripper_jog_direction=-1)
    if key == "o":
        return replace(state, gripper_delta=state.gripper_delta + 1, gripper_jog_direction=1)
    if key == " ":
        return replace(state, paused=not state.paused, single_step=False)
    if key == "." and state.paused:
        return replace(state, single_step=True)
    if key == "r":
        return replace(state, reset=True)
    if key == "t":
        return replace(state, home=True)
    if key == "g":
        return replace(state, mode="gravity_comp")
    if key == "h":
        return replace(state, mode="hold")
    if key == "p":
        return replace(state, mode="pos_vel")
    if key in ("q", "\x1b"):
        return replace(state, quit=True)
    return state


def _take_key_snapshot(events: SimpleQueue) -> tuple[int, ...]:
    snapshot = []
    for _ in range(events.qsize()):
        try:
            snapshot.append(events.get_nowait())
        except Empty:
            break
    return tuple(snapshot)


def drain_key_events(events: SimpleQueue, state: ViewerControlState) -> ViewerControlState:
    """Consume a finite FIFO snapshot, leaving new events for the next cycle."""
    for keycode in _take_key_snapshot(events):
        state = reduce_key(state, _decode_key(keycode))
    return state


def process_key_events(
    sim,
    events: SimpleQueue,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    """Execute one finite event snapshot sequentially on the simulation thread."""
    for keycode in _take_key_snapshot(events):
        state = reduce_key(state, _decode_key(keycode))
        state = apply_pending_commands(
            sim, state, joint_step, gripper_step, jog_hold_time=jog_hold_time
        )
        if state.quit:
            break
        if state.single_step:
            sim.step()
            state = replace(state, single_step=False)
    return state


def _state_from_sim(sim, *, paused: bool = False, selected_joint: int = 0) -> ViewerControlState:
    state = sim.get_state()
    targets = tuple(float(value) for value in sim.control_targets[:6])
    positions = tuple(float(value) for value in state.joint_positions[:6])
    return ViewerControlState(
        selected_joint=selected_joint,
        joint_targets=targets,
        joint_positions=positions,
        gripper_width=float(state.gripper_width),
        paused=paused,
        active_mode=str(getattr(sim, "control_mode", "pos_vel")),
    )


def apply_pending_commands(
    sim,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    *,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    if state.reset or state.home:
        pending_joint_delta = state.joint_delta
        pending_gripper_delta = state.gripper_delta
        quit_requested = state.quit
        if state.home:
            sim.reset_home()
        else:
            sim.reset()
        if hasattr(sim, "set_control_mode"):
            sim.set_control_mode("gravity_comp")
        state = _state_from_sim(
            sim, paused=state.paused, selected_joint=state.selected_joint
        )
        state = replace(
            state,
            joint_delta=pending_joint_delta,
            gripper_delta=pending_gripper_delta,
            quit=quit_requested,
        )

    if state.mode is not None and hasattr(sim, "set_control_mode"):
        active_mode = sim.set_control_mode(state.mode)
    else:
        active_mode = getattr(sim, "control_mode", state.active_mode)

    targets = state.joint_targets
    jog_time_remaining = state.jog_time_remaining
    if state.joint_delta:
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = targets[state.selected_joint] + state.joint_delta * joint_step
        targets = tuple(sim.set_joint_position_targets({name: requested}))
        active_mode = getattr(sim, "control_mode", "pos_vel")
        jog_time_remaining = jog_hold_time

    width = state.gripper_width
    if state.gripper_delta:
        width = float(sim.set_gripper_width(width + state.gripper_delta * gripper_step))
        jog_time_remaining = jog_hold_time

    sim_state = sim.get_state()
    return replace(
        state,
        joint_targets=targets,
        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
        gripper_width=width,
        joint_delta=0,
        gripper_delta=0,
        jog_time_remaining=jog_time_remaining,
        reset=False,
        home=False,
        mode=None,
        active_mode=str(active_mode),
    )


def apply_continuous_jog(
    sim,
    state: ViewerControlState,
    *,
    dt: float,
    joint_rate: float,
    gripper_rate: float,
) -> ViewerControlState:
    if state.jog_time_remaining <= 0.0:
        return replace(state, joint_jog_direction=0, gripper_jog_direction=0)

    targets = state.joint_targets
    if state.joint_jog_direction:
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = targets[state.selected_joint] + state.joint_jog_direction * joint_rate * dt
        targets = tuple(sim.set_joint_position_targets({name: requested}))

    width = state.gripper_width
    if state.gripper_jog_direction:
        width = float(sim.set_gripper_width(width + state.gripper_jog_direction * gripper_rate * dt))

    sim_state = sim.get_state()
    remaining = max(0.0, state.jog_time_remaining - dt)
    return replace(
        state,
        joint_targets=targets,
        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
        gripper_width=width,
        jog_time_remaining=remaining,
        joint_jog_direction=state.joint_jog_direction if remaining > 0.0 else 0,
        gripper_jog_direction=state.gripper_jog_direction if remaining > 0.0 else 0,
        active_mode=str(getattr(sim, "control_mode", state.active_mode)),
    )


def overlay_text(state: ViewerControlState) -> str:
    name = ARM_JOINT_NAMES[state.selected_joint]
    run_state = "paused" if state.paused else "running"
    return (
        f"mode: {state.active_mode}  selected: {name}  "
        f"q: {state.joint_positions[state.selected_joint]:.3f} rad  "
        f"target: {state.joint_targets[state.selected_joint]:.3f} rad\n"
        f"gripper target: {state.gripper_width:.3f} m  state: {run_state}\n"
        "MuJoCo control panel shows torque/force, not joint position.\n"
        f"{HELP}"
    )


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control reBotArm in the MuJoCo viewer")
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--joint-step", type=_positive_float, default=0.01, help="joint jog in radians")
    parser.add_argument("--gripper-step", type=_positive_float, default=0.001, help="gripper jog in metres")
    parser.add_argument("--joint-rate", type=_positive_float, default=0.08, help="held joint jog rate in rad/s")
    parser.add_argument("--gripper-rate", type=_positive_float, default=0.01, help="held gripper jog rate in m/s")
    parser.add_argument(
        "--jog-hold-time",
        type=_positive_float,
        default=0.18,
        help="seconds to keep jogging after the latest key-repeat event",
    )
    parser.add_argument(
        "--duration",
        type=_positive_float,
        default=None,
        help="exit after this many seconds of simulated time",
    )
    return parser


def _decode_key(keycode: int) -> str:
    if keycode == 256:
        return "\x1b"
    try:
        return chr(keycode)
    except (TypeError, ValueError):
        return ""


def _close_viewer_then_sim(
    viewer,
    sim,
    model,
    data,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    timeout: float = 5.0,
) -> None:
    """Release native state only after the passive viewer thread lets it go."""
    if viewer is None:
        sim.close()
        return
    try:
        viewer.close()
    except BaseException:
        model_is_released = False
        try:
            model_is_released = viewer.m is None
        except BaseException:
            # If the public lifecycle signal is itself unavailable, releasing
            # native state would be an unsafe guess.
            pass
        if model_is_released:
            sim.close()
        else:
            _RETAINED_UNSAFE_VIEWERS.append((viewer, sim, model, data))
        raise

    viewer_model = getattr(viewer, "m", None)
    if viewer_model is None:
        sim.close()
        return
    started = clock()
    while viewer_model is not None:
        if clock() - started >= timeout:
            # Releasing MjModel/MjData while the viewer still exposes its model
            # can crash the process. Retain the complete ownership graph and
            # report the failed teardown instead of risking use-after-free.
            _RETAINED_UNSAFE_VIEWERS.append((viewer, sim, model, data))
            raise TimeoutError("MuJoCo passive viewer did not finish closing")
        sleep(0.01)
        viewer_model = getattr(viewer, "m", None)
    sim.close()


def main(
    argv: Sequence[str] | None = None,
    *,
    sim_factory: Callable = RebotArmMujoco,
    launch_passive: Callable | None = None,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
    status_stream=None,
) -> int:
    args = build_parser().parse_args(argv)
    if status_stream is None:
        status_stream = sys.stderr
    sim = sim_factory(args.model)
    viewer = None
    model = data = None
    try:
        sim.reset()
        if hasattr(sim, "set_control_mode"):
            sim.set_control_mode("gravity_comp")
        if launch_passive is None:
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive

        state = _state_from_sim(sim)
        start_simulation_time = float(sim.get_state().simulation_time)
        events = SimpleQueue()
        previous_status = overlay_text(state)
        print(previous_status, file=status_stream, flush=True)

        def on_key(keycode: int) -> None:
            events.put(keycode)

        model, data = sim._unsafe_viewer_handles()
        viewer = launch_passive(
            model,
            data,
            key_callback=on_key,
        )
        try:
            while viewer.is_running():
                state = process_key_events(
                    sim,
                    events,
                    state,
                    args.joint_step,
                    args.gripper_step,
                    args.jog_hold_time,
                )
                if state.quit:
                    break
                cycle_start = clock()
                if not state.paused or state.single_step:
                    state = apply_continuous_jog(
                        sim,
                        state,
                        dt=sim.timestep,
                        joint_rate=args.joint_rate,
                        gripper_rate=args.gripper_rate,
                    )
                    sim.step()
                    sim_state = sim.get_state()
                    state = replace(
                        state,
                        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
                        single_step=False,
                    )
                current_status = overlay_text(state)
                if current_status != previous_status:
                    print(current_status, file=status_stream, flush=True)
                    previous_status = current_status
                viewer.sync()
                elapsed = float(sim.get_state().simulation_time) - start_simulation_time
                if args.duration is not None and elapsed >= args.duration:
                    break
                sleep(max(0.0, sim.timestep - (clock() - cycle_start)))
            return 0
        except KeyboardInterrupt:
            return 130
    finally:
        _close_viewer_then_sim(
            viewer,
            sim,
            model,
            data,
            clock=clock,
            sleep=sleep,
        )


if __name__ == "__main__":
    raise SystemExit(main())
