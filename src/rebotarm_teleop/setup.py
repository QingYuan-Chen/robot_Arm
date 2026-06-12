from setuptools import find_packages, setup

package_name = "rebotarm_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Keyboard, web, and gripper teleoperation command adapters for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "TeleopKeyboardNode = rebotarm_teleop.teleop_keyboard_node:main",
            "GripperVisualJointStateNode = rebotarm_teleop.gripper_visual_joint_state_node:main",
        ],
    },
)
