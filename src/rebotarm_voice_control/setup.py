from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarm_voice_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Task-level text and voice command control for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_text_input = rebotarm_voice_control.text_input_node:main",
            "rebotarm_llm_tool = rebotarm_voice_control.llm_tool_node:main",
            "rebotarm_tool_call = rebotarm_voice_control.tool_call_node:main",
            "rebotarm_realtime_event = rebotarm_voice_control.realtime_event_node:main",
            "rebotarm_realtime_gateway = rebotarm_voice_control.realtime_voice_gateway_node:main",
            "rebotarm_sim_move_relative_action = rebotarm_voice_control.sim_move_relative_action_node:main",
            "rebotarm_sim_executor = rebotarm_voice_control.sim_executor:main",
            "rebotarm_voice_file = rebotarm_voice_control.voice_file_node:main",
            "rebotarm_voice_control_node = rebotarm_voice_control.voice_control_node:main",
        ],
    },
)
