from __future__ import annotations


def test_load_handeye_config_builds_static_tf_arguments(tmp_path):
    from rebotarm_vision.handeye_config import load_handeye_config

    config_path = tmp_path / "handeye.yaml"
    config_path.write_text(
        """
handeye:
  parent_frame: end_link
  child_frame: camera_depth_frame
  translation:
    x: 0.03
    y: -0.01
    z: 0.08
  rotation:
    x: 0.0
    y: 0.0
    z: 0.7071068
    w: 0.7071068
""",
        encoding="utf-8",
    )

    config = load_handeye_config(config_path)

    assert config.parent_frame == "end_link"
    assert config.child_frame == "camera_depth_frame"
    assert config.as_static_transform_arguments() == [
        "0.03",
        "-0.01",
        "0.08",
        "0.0",
        "0.0",
        "0.7071068",
        "0.7071068",
        "end_link",
        "camera_depth_frame",
    ]
