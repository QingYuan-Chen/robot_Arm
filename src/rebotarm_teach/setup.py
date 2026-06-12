from setuptools import find_packages, setup

package_name = "rebotarm_teach"

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
    description="Teach recording, prepared trajectory, and replay workflow utilities for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "TeachRecorderNode = rebotarm_teach.teach_recorder_node:main",
            "TeachReplayNode = rebotarm_teach.teach_replay_node:main",
        ],
    },
)
