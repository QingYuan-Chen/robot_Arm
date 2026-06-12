from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_motion_package_exports_core_modules() -> None:
    import rebotarm_motion.collision_precheck as collision_precheck
    import rebotarm_motion.replay_runtime_monitor as replay_runtime_monitor
    import rebotarm_motion.trajectory_safety_monitor as trajectory_safety_monitor
    import rebotarm_motion.trajectory_time_parameterization as trajectory_time_parameterization

    assert hasattr(collision_precheck, "CollisionPrechecker")
    assert hasattr(replay_runtime_monitor, "ReplayRuntimeMonitor")
    assert hasattr(trajectory_safety_monitor, "evaluate_replay_tracking")
    assert hasattr(trajectory_time_parameterization, "parameterize_teach_samples")


def test_interactive_control_keeps_motion_compatibility_imports() -> None:
    import rebotarm_interactive_control.collision_precheck as legacy_collision_precheck
    import rebotarm_interactive_control.replay_runtime_monitor as legacy_replay_runtime_monitor
    import rebotarm_motion.collision_precheck as motion_collision_precheck
    import rebotarm_motion.replay_runtime_monitor as motion_replay_runtime_monitor

    assert legacy_collision_precheck.CollisionPrechecker is motion_collision_precheck.CollisionPrechecker
    assert legacy_replay_runtime_monitor.ReplayRuntimeMonitor is motion_replay_runtime_monitor.ReplayRuntimeMonitor


def test_teach_package_exports_core_modules() -> None:
    import rebotarm_teach.teach_recording as teach_recording
    import rebotarm_teach.teach_replay_coordinator as teach_replay_coordinator
    import rebotarm_teach.teach_replay_settings as teach_replay_settings

    assert hasattr(teach_recording, "TeachSample")
    assert hasattr(teach_replay_coordinator, "TeachReplayCoordinator")
    assert hasattr(teach_replay_settings, "TeachReplaySettingsProvider")


def test_interactive_control_keeps_teach_compatibility_imports() -> None:
    import rebotarm_interactive_control.teach_recording as legacy_teach_recording
    import rebotarm_interactive_control.teach_replay_settings as legacy_teach_replay_settings
    import rebotarm_teach.teach_recording as teach_recording
    import rebotarm_teach.teach_replay_settings as teach_replay_settings

    assert legacy_teach_recording.TeachSample is teach_recording.TeachSample
    assert legacy_teach_replay_settings.TeachReplaySettingsProvider is teach_replay_settings.TeachReplaySettingsProvider


def test_teleop_package_exports_command_adapters() -> None:
    import rebotarm_teleop.teleop_core as teleop_core
    import rebotarm_teleop.web_execute as web_execute
    import rebotarm_teleop.web_teleop_client as web_teleop_client

    assert hasattr(teleop_core, "validate_web_keyboard_command")
    assert hasattr(web_execute, "validate_web_execute_request")
    assert hasattr(web_teleop_client, "WebTeleopClient")


def test_interactive_control_keeps_teleop_compatibility_imports() -> None:
    import rebotarm_interactive_control.teleop_core as legacy_teleop_core
    import rebotarm_interactive_control.web_execute as legacy_web_execute
    import rebotarm_teleop.teleop_core as teleop_core
    import rebotarm_teleop.web_execute as web_execute

    assert legacy_teleop_core.TeleopTargetPlanner is teleop_core.TeleopTargetPlanner
    assert legacy_web_execute.WebExecuteDecision is web_execute.WebExecuteDecision


def test_dashboard_package_exports_status_panel_modules() -> None:
    import rebotarm_dashboard.status_panel_api as status_panel_api
    import rebotarm_dashboard.status_panel_http as status_panel_http
    import rebotarm_dashboard.status_panel_state as status_panel_state

    assert hasattr(status_panel_api, "dispatch_post_request")
    assert hasattr(status_panel_http, "create_status_panel_server")
    assert hasattr(status_panel_state, "TeleopStatusStore")


def test_interactive_control_keeps_dashboard_compatibility_imports() -> None:
    import rebotarm_dashboard.status_panel_api as dashboard_api
    import rebotarm_interactive_control.status_panel_api as legacy_api

    assert legacy_api.dispatch_post_request is dashboard_api.dispatch_post_request


def test_layered_packages_do_not_depend_on_interactive_control_package() -> None:
    package_roots = [
        ROOT / "src/rebotarm_dashboard/rebotarm_dashboard",
        ROOT / "src/rebotarm_motion/rebotarm_motion",
        ROOT / "src/rebotarm_teach/rebotarm_teach",
        ROOT / "src/rebotarm_teleop/rebotarm_teleop",
    ]
    sources = [
        source
        for package_root in package_roots
        for source in package_root.rglob("*.py")
    ]
    offenders = {
        source.name: source.read_text(encoding="utf-8")
        for source in sources
        if "rebotarm_interactive_control" in source.read_text(encoding="utf-8")
    }

    assert offenders == {}


def test_primary_bringup_launches_dashboard_package_directly() -> None:
    launch_files = [
        ROOT / "src/rebotarm_bringup/launch/rebotarm_app.launch.py",
        ROOT / "src/rebotarm_bringup/launch/teleop_system.launch.py",
    ]

    for launch_file in launch_files:
        text = launch_file.read_text(encoding="utf-8")
        panel_index = text.index('executable="TeleopStatusPanelNode"')
        package_context = text[max(0, panel_index - 120):panel_index]
        assert 'package="rebotarm_dashboard"' in package_context


def test_teach_launches_use_teach_package_directly() -> None:
    launch_expectations = [
        (ROOT / "src/rebotarm_bringup/launch/teach_record.launch.py", "TeachRecorderNode"),
        (ROOT / "src/rebotarm_bringup/launch/teach_replay.launch.py", "TeachReplayNode"),
        (ROOT / "src/rebotarm_bringup/launch/teleop_system.launch.py", "TeachRecorderNode"),
    ]

    for launch_file, executable in launch_expectations:
        text = launch_file.read_text(encoding="utf-8")
        node_index = text.index(f'executable="{executable}"')
        package_context = text[max(0, node_index - 120):node_index]
        assert 'package="rebotarm_teach"' in package_context


def test_keyboard_launches_use_teleop_package_directly() -> None:
    launch_expectations = [
        ROOT / "src/rebotarm_bringup/launch/rebotarm_app.launch.py",
        ROOT / "src/rebotarm_bringup/launch/teleop_keyboard.launch.py",
    ]

    for launch_file in launch_expectations:
        text = launch_file.read_text(encoding="utf-8")
        node_index = text.index('executable="TeleopKeyboardNode"')
        package_context = text[max(0, node_index - 120):node_index]
        assert 'package="rebotarm_teleop"' in package_context


def test_rviz_drag_launch_does_not_start_legacy_preview_execution_nodes() -> None:
    text = (ROOT / "src/rebotarm_bringup/launch/interactive_system.launch.py").read_text(
        encoding="utf-8"
    )

    assert "PreviewNode" not in text
    assert "ExecutionNode" not in text
    assert "MarkerServerNode" not in text
    assert "interactive_control/execute_preview" not in text


def test_visual_gripper_launches_use_teleop_package_directly() -> None:
    for launch_file in (ROOT / "src/rebotarm_bringup/launch").glob("*.launch.py"):
        text = launch_file.read_text(encoding="utf-8")
        if 'executable="GripperVisualJointStateNode"' not in text:
            continue
        node_index = text.index('executable="GripperVisualJointStateNode"')
        package_context = text[max(0, node_index - 120):node_index]
        assert 'package="rebotarm_teleop"' in package_context


def test_interactive_control_python_package_is_compatibility_layer_only() -> None:
    source_root = ROOT / "src/rebotarm_interactive_control/rebotarm_interactive_control"
    concrete_modules = [
        source.name
        for source in source_root.glob("*.py")
        if source.name != "__init__.py"
        and "sys.modules[__name__]" not in source.read_text(encoding="utf-8")
    ]

    assert concrete_modules == []


def test_architecture_document_defines_package_responsibilities_and_rules() -> None:
    architecture = (ROOT / "docs/architecture.md").read_text(encoding="utf-8")
    context = (ROOT / "CONTEXT.md").read_text(encoding="utf-8")
    agents = (ROOT / "AGENTS.md").read_text(encoding="utf-8")

    required_terms = [
        "rebotarmcontroller",
        "rebotarm_motion",
        "rebotarm_teach",
        "rebotarm_teleop",
        "rebotarm_dashboard",
        "rebotarm_vision",
        "rebotarm_calibration",
        "rebotarm_interactive_control",
        "compatibility layer",
        "must not import rebotarm_interactive_control",
        "Hardware ownership",
        "Motion ownership",
        "Operator interaction ownership",
    ]
    for term in required_terms:
        assert term in architecture

    assert "Teach Replay" in context
    assert "Point-to-Point Execution" in context
    assert "docs/architecture.md" in agents
    assert "Do not add implementation logic to rebotarm_interactive_control" in agents


def test_rviz_moveit_drag_entrypoints_exist_without_custom_ee_target() -> None:
    feature_doc = (ROOT / "docs/rebotarm_feature_commands.md").read_text(
        encoding="utf-8"
    )
    rviz_config = (
        ROOT / "src/rebotarm_bringup/rviz/interactive_system.rviz"
    ).read_text(encoding="utf-8")
    real_launch = (
        ROOT / "src/rebotarm_bringup/launch/rviz_ee_drag_real.launch.py"
    ).read_text(encoding="utf-8")
    sim_launch = (
        ROOT / "src/rebotarm_bringup/launch/rviz_ee_drag_sim.launch.py"
    ).read_text(encoding="utf-8")

    assert "rviz_ee_drag_real.launch.py" in feature_doc
    assert "rviz_ee_drag_sim.launch.py" in feature_doc
    assert '"use_moveit_preview": "true"' in real_launch
    assert '"use_moveit_preview": "true"' in sim_launch
    assert "start_interaction_nodes" not in real_launch
    assert "start_interaction_nodes" not in sim_launch
    assert "interactive_control/ee_target" not in feature_doc
    assert "EndEffectorTarget" not in rviz_config
    assert "InteractiveMarkers" not in rviz_config
    assert "/rebotarm/interactive_control/ee_target/update" not in rviz_config
    assert "MarkerServerNode" not in (
        ROOT / "src/rebotarm_teleop/setup.py"
    ).read_text(encoding="utf-8")
    assert "InteractiveTargetNode" not in (
        ROOT / "src/rebotarm_teleop/setup.py"
    ).read_text(encoding="utf-8")


def test_legacy_custom_interactive_preview_entrypoints_are_removed() -> None:
    motion_setup = (ROOT / "src/rebotarm_motion/setup.py").read_text(encoding="utf-8")
    compatibility_setup = (
        ROOT / "src/rebotarm_interactive_control/setup.py"
    ).read_text(encoding="utf-8")

    for text in (motion_setup, compatibility_setup):
        console_lines = [line.strip().strip('",') for line in text.splitlines()]
        assert not any(line.startswith("PreviewNode =") for line in console_lines)
        assert not any(line.startswith("ExecutionNode =") for line in console_lines)
        assert "MarkerServerNode" not in text
        assert "InteractiveTargetNode" not in text

    assert not (ROOT / "src/rebotarm_motion/rebotarm_motion/preview_node.py").exists()
    assert not (ROOT / "src/rebotarm_motion/rebotarm_motion/execution_node.py").exists()
