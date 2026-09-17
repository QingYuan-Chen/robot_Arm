"""语音控制包的 ament_python 安装脚本。

除安装 Python 包本体外，这里还决定三件事：
    1. 资源索引文件 resource/<包名>：让 ROS 2 能通过包名定位本包；
    2. data_files：把 package.xml、config/*.yaml 与 launch/*.launch.py 安装到
       share/<包名>/ 下——运行时各节点通过 <包根>/config 读取配置，因此少装
       任何一类都会导致启动即失败；
    3. console_scripts：对外暴露的命令行/可执行入口，名字是对外契约，
       被启动文件与测试直接引用，不可随意更改。

每个入口的职责：
    rebotarm_text_input               交互式中文文本指令（演练路由）
    rebotarm_llm_tool                 大模型文本工具调用入口
    rebotarm_tool_call                单条工具调用 JSON 的路由入口
    rebotarm_realtime_event           单条实时语音事件的路由入口
    rebotarm_realtime_gateway         实时事件流网关（JSONL 回放）
    rebotarm_sim_move_relative_action 仿真相对位移动作服务端
    rebotarm_sim_executor             仿真执行器
    rebotarm_voice_file               音频文件 → 转写 → 指令路由
    rebotarm_voice_control_node       常驻占位节点（演练模式）
"""

from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarm_voice_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament 资源索引：包被发现的前提。
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    # PyYAML 供 config_loader 读取配置；setuptools 为构建期自举依赖。
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
