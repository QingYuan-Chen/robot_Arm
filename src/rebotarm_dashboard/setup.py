from setuptools import find_packages, setup

package_name = "rebotarm_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "rebotarm_dashboard.status_panel_assets": ["index.html"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Web dashboard, SSE status, and API adapter for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "TeleopStatusPanelNode = rebotarm_dashboard.teleop_status_panel_node:main",
        ],
    },
)
