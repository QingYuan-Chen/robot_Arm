import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rebotarm_simulation"


def install_resources(pattern):
    files_by_destination = {}
    for path in glob(pattern, recursive=True):
        destination = os.path.join("share", package_name, os.path.dirname(path))
        files_by_destination.setdefault(destination, []).append(path)
    return [
        (destination, sorted(files_by_destination[destination]))
        for destination in sorted(files_by_destination)
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]
    + install_resources("models/**/*.xml")
    + install_resources("models/**/*.[sS][tT][lL]")
    + install_resources("config/*.yaml")
    + install_resources("launch/*.launch.py"),
    install_requires=["setuptools", "mujoco>=3.3,<4", "numpy>=1.26", "PyYAML>=6"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="RViz/offline simulation utilities for reBotArm bringup tests.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_sim_trajectory_controller = rebotarm_simulation.sim_trajectory_controller_node:main",
            "rebotarm_mujoco_health = rebotarm_simulation.mujoco_health:main",
            "rebotarm_mujoco_acceptance = rebotarm_simulation.mujoco_acceptance:main",
            "rebotarm_mujoco_batch = rebotarm_simulation.mujoco_batch:main",
            "rebotarm_mujoco_contact_check = rebotarm_simulation.mujoco_contact_check:main",
            "rebotarm_mujoco_ros_acceptance = rebotarm_simulation.mujoco_ros_acceptance:main",
            "rebotarm_real2sim_acceptance = rebotarm_simulation.real2sim_acceptance:main",
            "rebotarm_real2sim_bridge = rebotarm_simulation.real2sim_ros_node:main",
            "rebotarm_real2sim_viewer = rebotarm_simulation.real2sim_viewer:main",
            "rebotarm_mujoco_moveit_acceptance = rebotarm_simulation.mujoco_moveit_acceptance:main",
            "rebotarm_mujoco_pick_batch = rebotarm_simulation.mujoco_pick_batch:main",
            "rebotarm_mujoco_cli = rebotarm_simulation.mujoco_cli:main",
            "rebotarm_mujoco_viewer = rebotarm_simulation.mujoco_viewer:main",
            "rebotarm_mujoco_node = rebotarm_simulation.mujoco_ros_node:main",
            "rebotarm_sim2real = rebotarm_simulation.sim2real_cli:main",
            "rebotarm_urdf_to_mjcf = rebotarm_simulation.urdf_to_mjcf:main",
        ],
    },
)
