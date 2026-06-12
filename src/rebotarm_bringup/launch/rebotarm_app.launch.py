import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bool_text(value: str) -> str:
    return "true" if str(value).strip().lower() in {"1", "true", "yes", "on"} else "false"


def _as_bool(value: str) -> bool:
    return _bool_text(value) == "true"


def _record_path(name: str, explicit_path: str) -> str:
    if explicit_path:
        return explicit_path
    safe_name = name.strip() or "teach_record"
    safe_name = os.path.basename(safe_name.replace("\\", "/")) or "teach_record"
    if not safe_name.endswith(".jsonl"):
        safe_name = f"{safe_name}.jsonl"
    return f"teleop_records/{safe_name}"


def _resolve_channel(channel: str) -> str:
    if channel and channel != "auto":
        return channel
    for candidate in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if os.path.exists(candidate):
            return candidate
    return "/dev/ttyACM0"


def _panel_node(
    *,
    teleop_config,
    arm_namespace: str,
    record_path: str,
    use_hardware: str,
    web_execute_enabled: str,
    execution_mode: str,
    panel_mode: str,
):
    return Node(
        package="rebotarm_dashboard",
        executable="TeleopStatusPanelNode",
        name="teleop_status_panel_node",
        output="screen",
        parameters=[
            teleop_config,
            {
                "arm_namespace": arm_namespace,
                "web_execute_enabled": _as_bool(web_execute_enabled),
                "record_path": record_path,
                "use_hardware": _as_bool(use_hardware),
                "execution_mode": execution_mode,
                "panel_mode": panel_mode,
            },
        ],
    )


def _keyboard_node(*, teleop_config, arm_namespace: str, keyboard_prefix: str):
    return Node(
        package="rebotarm_teleop",
        executable="TeleopKeyboardNode",
        name="teleop_keyboard_node",
        output="screen",
        prefix=keyboard_prefix,
        parameters=[teleop_config, {"arm_namespace": arm_namespace}],
    )


def _launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context).strip()
    explicit_record_path = LaunchConfiguration("record_path").perform(context).strip()
    record_path = _record_path(name, explicit_record_path)
    arm_namespace = LaunchConfiguration("arm_namespace").perform(context)
    channel = LaunchConfiguration("channel").perform(context)
    use_hardware = _bool_text(LaunchConfiguration("use_hardware").perform(context))
    use_rviz = _bool_text(LaunchConfiguration("use_rviz").perform(context))
    panel = _bool_text(LaunchConfiguration("panel").perform(context))
    web_execute_enabled = _bool_text(LaunchConfiguration("web_execute_enabled").perform(context))
    execution_mode = LaunchConfiguration("execution_mode").perform(context).strip().lower()
    keyboard_prefix = LaunchConfiguration("keyboard_prefix").perform(context)
    teleop_config = LaunchConfiguration("teleop_config")
    bringup_share = FindPackageShare("rebotarm_bringup")
    moveit_launch = PathJoinSubstitution([bringup_share, "launch", "moveit_hardware.launch.py"])
    web_rviz_config = PathJoinSubstitution([bringup_share, "rviz", "web_teleop_status.rviz"])
    resolved_channel = _resolve_channel(channel)

    if use_hardware != "true":
        raise RuntimeError(
            "rebotarm_app.launch.py is the full hardware app and requires use_hardware:=true"
        )

    actions = [
        LogInfo(msg="reBotArm app: starting full MoveIt + web teleop workbench"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "arm_namespace": arm_namespace,
                "channel": resolved_channel,
                "use_rviz": "false",
                "teach_record_path": record_path,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", web_rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ]

    if panel == "true":
        actions.append(
            _panel_node(
                teleop_config=teleop_config,
                arm_namespace=arm_namespace,
                record_path=record_path,
                use_hardware=use_hardware,
                web_execute_enabled=web_execute_enabled,
                execution_mode=execution_mode,
                panel_mode="control",
            )
        )

    actions.append(LogInfo(msg="teach recording/check/replay are controlled from the web Teach Trajectory card"))

    return actions


def generate_launch_description():
    interactive_share = FindPackageShare("rebotarm_interactive_control")
    teleop_config = PathJoinSubstitution(
        [interactive_share, "config", "teleop_control.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="teach_record"),
            DeclareLaunchArgument("record_path", default_value=""),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value="auto"),
            DeclareLaunchArgument("use_hardware", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("panel", default_value="true"),
            DeclareLaunchArgument("web_execute_enabled", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="execute"),
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'",
            ),
            DeclareLaunchArgument("teleop_config", default_value=teleop_config),
            OpaqueFunction(function=_launch_setup),
        ]
    )
