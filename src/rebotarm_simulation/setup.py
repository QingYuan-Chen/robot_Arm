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


launch_files = glob("launch/*.launch.py")


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["assets/*.xml"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]
    + install_resources("models/**/*.xml")
    + install_resources("models/**/*.[sS][tT][lL]")
    + install_resources("config/*.yaml")
    + install_resources("launch/*.launch.py")
    + [(f"share/{package_name}/launch", sorted(launch_files))],
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
            "rebotarm_mujoco_cli = rebotarm_simulation.mujoco_cli:main",
            "rebotarm_mujoco = rebotarm_simulation.mujoco_cli:main",
            "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main",
            "rebotarm_mujoco_legacy_cli = rebotarm_simulation.mujoco_legacy_cli:main",
            # Retained for rollback comparisons; active launch uses the
            # upstream node directly from this package.
            "rebotarm_upstream_mujoco_node = rebotarm_simulation.upstream_backend:main",
            "rebotarm_mujoco_viewer = rebotarm_simulation.mujoco_viewer:main",
            "rebotarm_mujoco_node = rebotarm_simulation.mujoco_ros_node:main",
            "rebotarm_urdf_to_mjcf = rebotarm_simulation.urdf_to_mjcf:main",
        ],
    },
)
