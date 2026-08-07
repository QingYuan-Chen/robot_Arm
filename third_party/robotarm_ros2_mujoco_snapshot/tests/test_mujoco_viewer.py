from __future__ import annotations

import io
from queue import SimpleQueue
import threading
from types import SimpleNamespace

import pytest

from rebotarm_simulation import mujoco_viewer
from rebotarm_simulation.mujoco_sim import RebotArmMujoco


class FakeSim:
    joint_names = tuple(f"joint{i}" for i in range(1, 7)) + (
        "left_finger_joint",
        "right_finger_joint",
    )

    def __init__(self, model_path=None):
        self.model_path = model_path
        self.viewer_model = object()
        self.viewer_data = object()
        self.timestep = 0.01
        self.targets = [0.0] * 6
        self.width = 0.05
        self.time = 0.0
        self.control_mode = "pos_vel"
        self.calls = []
        self.call_threads = []
        self.closed = False

    @property
    def control_targets(self):
        return tuple(self.targets) + (self.width / 2.0, -self.width / 2.0)

    def _unsafe_viewer_handles(self):
        return self.viewer_model, self.viewer_data

    def get_state(self):
        return SimpleNamespace(
            joint_positions=tuple(self.targets) + (self.width / 2.0, -self.width / 2.0),
            gripper_width=self.width,
            simulation_time=self.time,
        )

    def set_joint_position_targets(self, targets):
        self.call_threads.append(threading.get_ident())
        self.calls.append(("joints", dict(targets)))
        for name, value in targets.items():
            index = int(name.removeprefix("joint")) - 1
            self.targets[index] = min(1.0, max(-1.0, float(value)))
        return tuple(self.targets)

    def set_gripper_width(self, width):
        self.call_threads.append(threading.get_ident())
        self.calls.append(("gripper", width))
        self.width = min(0.09, max(0.0, float(width)))
        return self.width

    def set_control_mode(self, mode):
        self.calls.append(("mode", mode))
        self.control_mode = mode
        return mode

    def step(self):
        self.call_threads.append(threading.get_ident())
        self.calls.append(("step",))
        self.time += self.timestep
        return self.get_state()

    def reset(self):
        self.calls.append(("reset",))
        self.targets[:] = [0.0] * 6
        self.width = 0.09
        self.time = 0.0
        return self.get_state()

    def reset_home(self):
        self.calls.append(("reset_home",))
        self.targets[:] = [0.0] * 6
        self.width = 0.09
        self.time = 0.0
        return self.get_state()

    def close(self):
        self.calls.append(("close",))
        self.closed = True


class FakeViewer:
    def __init__(self, callback, keys):
        self.callback = callback
        self.keys = iter(keys)
        self.sync_count = 0
        self.closed = False

    def is_running(self):
        try:
            self.callback(next(self.keys))
            return True
        except StopIteration:
            return False

    def sync(self):
        self.sync_count += 1

    def close(self):
        self.closed = True


def test_internal_viewer_handles_reject_closed_simulation():
    sim = RebotArmMujoco.__new__(RebotArmMujoco)
    sim._closed = True
    with pytest.raises(RuntimeError, match="closed"):
        sim._unsafe_viewer_handles()


def test_control_state_selects_six_arm_joints_with_wrapping():
    state = mujoco_viewer.ViewerControlState()
    state = mujoco_viewer.reduce_key(state, "]")
    assert state.selected_joint == 1
    state = mujoco_viewer.reduce_key(state, "[")
    state = mujoco_viewer.reduce_key(state, "[")
    assert state.selected_joint == 5
    assert mujoco_viewer.reduce_key(state, "3").selected_joint == 2


@pytest.mark.parametrize(
    ("key", "field", "value"),
    [
        ("j", "joint_delta", -1),
        ("k", "joint_delta", 1),
        ("c", "gripper_delta", -1),
        ("o", "gripper_delta", 1),
        (".", "single_step", True),
        ("r", "reset", True),
        ("q", "quit", True),
    ],
)
def test_key_reducer_maps_commands(key, field, value):
    result = mujoco_viewer.reduce_key(mujoco_viewer.ViewerControlState(paused=True), key)
    assert getattr(result, field) == value


def test_reducer_accumulates_repeated_jog_and_gripper_events():
    state = mujoco_viewer.ViewerControlState()
    for key in ("k", "k", "j", "o", "o", "c"):
        state = mujoco_viewer.reduce_key(state, key)
    assert state.joint_delta == 1
    assert state.gripper_delta == 1


def test_drain_key_events_preserves_burst_order_and_counts():
    events = SimpleQueue()
    for key in (ord("k"), ord("k"), ord("j"), ord("o"), ord(" "), ord(".")):
        events.put(key)
    state = mujoco_viewer.drain_key_events(events, mujoco_viewer.ViewerControlState())
    assert state.joint_delta == 1
    assert state.gripper_delta == 1
    assert state.paused is True
    assert state.single_step is True
    assert events.empty()


def test_drain_processes_finite_snapshot_when_producer_keeps_adding():
    class GrowingQueue:
        def __init__(self):
            self.items = [ord("k")]
            self.get_calls = 0

        def qsize(self):
            return len(self.items)

        def get_nowait(self):
            self.get_calls += 1
            if self.get_calls > 3:
                raise RuntimeError("drain starved simulation loop")
            value = self.items.pop(0)
            self.items.append(ord("k"))
            return value

    events = GrowingQueue()
    state = mujoco_viewer.drain_key_events(events, mujoco_viewer.ViewerControlState())
    assert state.joint_delta == 1
    assert events.get_calls == 1
    assert events.items == [ord("k")]


def test_interleaved_jog_selection_jog_preserves_target_joint_order():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord("k"), ord("]"), ord("k")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert sim.targets[:2] == pytest.approx([0.05, 0.05])
    assert state.selected_joint == 1


def test_interleaved_open_reset_close_uses_reset_width_before_close():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord("o"), ord("r"), ord("c")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(gripper_width=0.05), 0.05, 0.005
    )
    assert ("reset",) in sim.calls
    assert sim.calls[-1][0] == "gripper"
    assert sim.calls[-1][1] == pytest.approx(0.085)
    assert state.gripper_width == pytest.approx(0.085)


def test_pause_single_step_resume_executes_step_at_its_ordered_position():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord(" "), ord("."), ord(" ")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert sum(call == ("step",) for call in sim.calls) == 1
    assert state.paused is False
    assert state.single_step is False


def test_quit_discards_later_jog_from_same_snapshot():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord("q"), ord("k")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert state.quit is True
    assert not any(call[0] == "joints" for call in sim.calls)
    assert events.empty()


def test_escape_discards_later_reset_from_same_snapshot():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (256, ord("r")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert state.quit is True
    assert ("reset",) not in sim.calls
    assert events.empty()


def test_pause_toggles_and_single_step_only_queues_while_paused():
    running = mujoco_viewer.reduce_key(mujoco_viewer.ViewerControlState(), ".")
    assert running.single_step is False
    paused = mujoco_viewer.reduce_key(running, " ")
    assert paused.paused is True
    assert mujoco_viewer.reduce_key(paused, ".").single_step is True
    assert mujoco_viewer.reduce_key(paused, " ").paused is False


def test_apply_pending_commands_uses_core_clamping_and_clears_one_shots():
    sim = FakeSim()
    sim.targets[0] = 0.95
    state = mujoco_viewer.ViewerControlState(
        joint_targets=(0.95, 0.0, 0.0, 0.0, 0.0, 0.0),
        gripper_width=0.05,
        joint_delta=1,
        gripper_delta=1,
    )
    state = mujoco_viewer.apply_pending_commands(sim, state, 0.2, 0.1)
    assert sim.targets[0] == 1.0
    assert sim.width == 0.09
    assert state.joint_targets[0] == 1.0
    assert state.gripper_width == 0.09
    assert state.joint_delta == state.gripper_delta == 0


def test_reset_preserves_pause_and_resynchronizes_targets():
    sim = FakeSim()
    state = mujoco_viewer.apply_pending_commands(
        sim, mujoco_viewer.ViewerControlState(paused=True, reset=True), 0.1, 0.01
    )
    assert ("reset",) in sim.calls
    assert state.paused is True
    assert state.reset is False
    assert state.gripper_width == 0.09


def test_reset_then_jog_in_same_event_batch_applies_jog_after_reset():
    sim = FakeSim()
    sim.targets[0] = 0.8
    state = mujoco_viewer.ViewerControlState(reset=True, joint_delta=2)
    state = mujoco_viewer.apply_pending_commands(sim, state, 0.1, 0.01)
    assert sim.calls[0] == ("reset",)
    assert ("joints", {"joint1": 0.2}) in sim.calls
    assert state.joint_targets[0] == pytest.approx(0.2)


def test_overlay_contains_selected_target_gripper_pause_and_help():
    state = mujoco_viewer.ViewerControlState(
        selected_joint=2,
        joint_targets=(0.0, 0.0, 0.25, 0.0, 0.0, 0.0),
        joint_positions=(0.0, 0.0, 0.20, 0.0, 0.0, 0.0),
        gripper_width=0.04,
        paused=True,
        active_mode="gravity_comp",
    )
    text = mujoco_viewer.overlay_text(state)
    for expected in (
        "gravity_comp", "joint3", "0.200", "0.250", "0.040", "paused",
        "torque/force", "[ / ]", "hold J/K",
    ):
        assert expected in text


def test_continuous_jog_advances_target_until_hold_window_expires():
    sim = FakeSim()
    state = mujoco_viewer.ViewerControlState(
        joint_jog_direction=1,
        jog_time_remaining=0.02,
    )
    state = mujoco_viewer.apply_continuous_jog(
        sim,
        state,
        dt=0.01,
        joint_rate=0.4,
        gripper_rate=0.03,
    )
    assert sim.targets[0] == pytest.approx(0.004)
    assert state.joint_jog_direction == 1

    state = mujoco_viewer.apply_continuous_jog(
        sim,
        state,
        dt=0.01,
        joint_rate=0.4,
        gripper_rate=0.03,
    )
    assert sim.targets[0] == pytest.approx(0.008)
    assert state.joint_jog_direction == 0
    assert state.jog_time_remaining == pytest.approx(0.0)


def test_parser_accepts_model_and_positive_steps():
    args = mujoco_viewer.build_parser().parse_args(
        [
            "--model", "scene.xml", "--joint-step", "0.2",
            "--gripper-step", "0.01", "--duration", "1.5",
        ]
    )
    assert (args.model, args.joint_step, args.gripper_step, args.duration) == (
        "scene.xml", 0.2, 0.01, 1.5,
    )
    for flag in ("--joint-step", "--gripper-step", "--duration"):
        with pytest.raises(SystemExit):
            mujoco_viewer.build_parser().parse_args([flag, "0"])


@pytest.mark.parametrize("value", ["-1", "nan", "inf"])
def test_parser_rejects_invalid_duration(value):
    with pytest.raises(SystemExit):
        mujoco_viewer.build_parser().parse_args(["--duration", value])


def test_runtime_launches_passively_steps_syncs_sleeps_and_always_closes():
    sim = FakeSim("scene.xml")
    holder = {}

    def launch(model, data, *, key_callback):
        assert (model, data) == (sim.viewer_model, sim.viewer_data)
        holder["viewer"] = FakeViewer(key_callback, [ord("k"), ord(" "), ord("."), ord("q")])
        return holder["viewer"]

    sleeps = []
    status = io.StringIO()
    clock_values = iter((0.0, 0.002, 1.0, 1.002, 2.0, 2.002))
    code = mujoco_viewer.main(
        ["--model", "scene.xml"],
        sim_factory=lambda _: sim,
        launch_passive=launch,
        sleep=sleeps.append,
        clock=lambda: next(clock_values),
        status_stream=status,
    )
    assert code == 0
    assert ("joints", {"joint1": pytest.approx(0.01)}) in sim.calls
    assert sum(call == ("step",) for call in sim.calls) == 2
    assert holder["viewer"].sync_count >= 3
    output = status.getvalue()
    for expected in ("joint1", "target", "gripper", "paused", "hold J/K", "torque/force"):
        assert expected in output
    assert sleeps == pytest.approx([0.008, 0.008, 0.008])
    assert holder["viewer"].closed is True
    assert sim.closed is True


def test_duration_stops_at_requested_simulation_time_and_closes():
    sim = FakeSim()

    class RunningViewer:
        closed = False

        def is_running(self):
            return True

        def sync(self):
            pass

        def close(self):
            self.closed = True

    viewer = RunningViewer()
    assert mujoco_viewer.main(
        ["--duration", "0.025"],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, **_kwargs: viewer,
        sleep=lambda _: None,
        status_stream=io.StringIO(),
    ) == 0
    assert sim.time == pytest.approx(0.03)
    assert viewer.closed is True
    assert sim.closed is True


def test_keyboard_interrupt_from_sync_returns_130_and_closes_without_escaping():
    sim = FakeSim()

    class InterruptingViewer:
        closed = False

        def is_running(self):
            return True

        def sync(self):
            raise KeyboardInterrupt

        def close(self):
            self.closed = True

    viewer = InterruptingViewer()
    assert mujoco_viewer.main(
        [],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, **_kwargs: viewer,
        sleep=lambda _: None,
        status_stream=io.StringIO(),
    ) == 130
    assert viewer.closed is True
    assert sim.closed is True


def test_callback_from_other_thread_only_enqueues_until_main_loop_drains():
    sim = FakeSim()
    main_thread = threading.get_ident()

    class ConcurrentViewer:
        def __init__(self, callback):
            self.callback = callback
            self.iteration = 0

        def is_running(self):
            self.iteration += 1
            return self.iteration <= 2

        def sync(self):
            worker = threading.Thread(target=lambda: (self.callback(ord("k")), self.callback(ord("k"))))
            worker.start()
            worker.join()

        def close(self):
            pass

    assert mujoco_viewer.main(
        [],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, key_callback: ConcurrentViewer(key_callback),
        sleep=lambda _: None,
        status_stream=io.StringIO(),
    ) == 0
    assert sim.targets[0] > 0.02
    assert sim.call_threads and set(sim.call_threads) == {main_thread}


def test_runtime_closes_sim_when_viewer_launch_fails():
    sim = FakeSim()

    def broken(*_args, **_kwargs):
        raise RuntimeError("display unavailable")

    with pytest.raises(RuntimeError, match="display unavailable"):
        mujoco_viewer.main([], sim_factory=lambda _: sim, launch_passive=broken)
    assert sim.closed is True


def test_runtime_prints_terminal_help_even_when_viewer_is_already_closed():
    sim = FakeSim()
    status = io.StringIO()

    class ClosedViewer:
        def is_running(self):
            return False

        def close(self):
            pass

    assert mujoco_viewer.main(
        [],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, **_kwargs: ClosedViewer(),
        status_stream=status,
    ) == 0
    assert "joint1" in status.getvalue()
    assert "J/K" in status.getvalue()


def test_runtime_closes_sim_even_when_viewer_close_raises():
    sim = FakeSim()

    class BadCloseViewer:
        m = None

        def is_running(self):
            return False

        def close(self):
            raise RuntimeError("viewer close failed")

    with pytest.raises(RuntimeError, match="viewer close failed"):
        mujoco_viewer.main(
            [],
            sim_factory=lambda _: sim,
            launch_passive=lambda *_args, **_kwargs: BadCloseViewer(),
            status_stream=io.StringIO(),
        )
    assert sim.closed is True


def test_cleanup_waits_for_public_viewer_model_to_clear_before_sim_close():
    order = []

    class OrderedSim(FakeSim):
        def close(self):
            order.append("sim.close")
            assert viewer.m is None
            super().close()

    class TransitioningViewer:
        def __init__(self):
            self.closed = False
            self.polls = 0

        @property
        def m(self):
            self.polls += 1
            return object() if self.polls < 2 else None

        def close(self):
            order.append("viewer.close")
            self.closed = True

    sim = OrderedSim()
    viewer = TransitioningViewer()
    sleeps = []
    mujoco_viewer._close_viewer_then_sim(
        viewer,
        sim,
        object(),
        object(),
        clock=iter((0.0, 0.1, 0.2)).__next__,
        sleep=sleeps.append,
        timeout=1.0,
    )
    assert order == ["viewer.close", "sim.close"]
    assert sleeps == [0.01]
    assert sim.closed is True


def test_cleanup_timeout_retains_unsafe_handles_and_does_not_close_sim():
    sim = FakeSim()

    class StuckViewer:
        m = object()

        def close(self):
            pass

    viewer = StuckViewer()
    retained_before = len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS)
    with pytest.raises(TimeoutError, match="did not finish"):
        mujoco_viewer._close_viewer_then_sim(
            viewer,
            sim,
            object(),
            object(),
            clock=iter((0.0, 2.0)).__next__,
            sleep=lambda _: None,
            timeout=1.0,
        )
    assert sim.closed is False
    assert len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS) == retained_before + 1


def test_close_error_with_live_public_model_retains_handles_without_sim_close():
    sim = FakeSim()

    class LiveViewer:
        m = object()

        def close(self):
            raise RuntimeError("close failed")

    viewer = LiveViewer()
    retained_before = len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS)
    with pytest.raises(RuntimeError, match="close failed"):
        mujoco_viewer._close_viewer_then_sim(viewer, sim, object(), object())
    assert sim.closed is False
    assert len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS) == retained_before + 1


def test_close_error_with_cleared_public_model_safely_closes_sim_then_reraises():
    sim = FakeSim()

    class ClearedViewer:
        m = None

        def close(self):
            raise RuntimeError("close failed")

    with pytest.raises(RuntimeError, match="close failed"):
        mujoco_viewer._close_viewer_then_sim(
            ClearedViewer(), sim, object(), object()
        )
    assert sim.closed is True


def test_close_error_with_unreadable_public_model_conservatively_retains_handles():
    sim = FakeSim()

    class UnreadableViewer:
        @property
        def m(self):
            raise ValueError("model unavailable")

        def close(self):
            raise RuntimeError("close failed")

    viewer = UnreadableViewer()
    retained_before = len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS)
    with pytest.raises(RuntimeError, match="close failed"):
        mujoco_viewer._close_viewer_then_sim(viewer, sim, object(), object())
    assert sim.closed is False
    assert len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS) == retained_before + 1
