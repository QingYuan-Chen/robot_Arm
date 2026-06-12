from setuptools import find_packages, setup

package_name = "rebotarm_simulation"

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
    description="RViz/offline simulation utilities for reBotArm bringup tests.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_sim_trajectory_controller = rebotarm_simulation.sim_trajectory_controller_node:main",
        ],
    },
)
