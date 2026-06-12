from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarm_interactive_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Phase-1 interactive control coordination for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "TeleopKeyboardNode = rebotarm_interactive_control.teleop_keyboard_node:main",
            "TeachRecorderNode = rebotarm_interactive_control.teach_recorder_node:main",
            "TeachReplayNode = rebotarm_interactive_control.teach_replay_node:main",
            "TeleopStatusPanelNode = rebotarm_interactive_control.teleop_status_panel_node:main",
            "GripperVisualJointStateNode = rebotarm_interactive_control.gripper_visual_joint_state_node:main",
        ],
    },
)
