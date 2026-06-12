from setuptools import find_packages, setup

package_name = "rebotarm_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/vision.launch.py"]),
        (
            f"share/{package_name}/config",
            [
                "config/camera.yaml",
                "config/flat_graspnet.yaml",
                "config/grasp_pose_policy.yaml",
                "config/graspnet_policy.yaml",
                "config/gripper_policy.yaml",
                "config/handeye.yaml",
                "config/retry_policy.yaml",
                "config/retreat_policy.yaml",
                "config/table_safety.yaml",
                "config/visual_ready.yaml",
                "config/visual_servo.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="ROS 2 Gemini2 vision node for reBotArm grasping.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_vision_node = rebotarm_vision.vision_node:main",
            "rebotarm_ordinary_grasp_node = rebotarm_vision.ordinary_grasp_node:main",
            "rebotarm_graspnet_baseline_node = rebotarm_vision.graspnet_baseline_node:main",
            "rebotarm_send_grasp_preview = rebotarm_vision.grasp_preview_sender_node:main",
            "rebotarm_visual_grasp_markers = rebotarm_vision.visual_grasp_marker_node:main",
            "rebotarm_visual_grasp_executor = rebotarm_vision.visual_grasp_executor_node:main",
            "rebotarm_grasp_candidate_ik_filter = rebotarm_vision.candidate_ik_filter_node:main",
            "rebotarm_grasp_tcp_frame = rebotarm_vision.grasp_tcp_frame_node:main",
            "rebotarm_visual_ready = rebotarm_vision.visual_ready_node:main",
            "rebotarm_visual_grasp_benchmark = rebotarm_vision.visual_grasp_benchmark:main",
            "rebotarm_hybrid_grasp_sim_benchmark = rebotarm_vision.hybrid_grasp_sim_benchmark:main",
            "rebotarm_tcp_calibration = rebotarm_vision.tcp_calibration_node:main",
            "rebotarm_debug_camera_preview = rebotarm_vision.debug_camera_preview:main",
            "rebotarm_grasp_depth_probe = rebotarm_vision.grasp_depth_probe_node:main",
        ],
    },
)
