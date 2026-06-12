from __future__ import annotations


def test_camera_info_to_msg_uses_live_intrinsics_for_depth_frame():
    from rebotarm_vision.converters.image_msgs import camera_info_to_msg

    msg = camera_info_to_msg(
        {
            "width": 640,
            "height": 400,
            "fx": 519.4,
            "fy": 519.1,
            "cx": 320.6,
            "cy": 201.3,
            "distortion_model": "plumb_bob",
            "d": [0.1, 0.2, 0.3, 0.4, 0.5],
        },
        stamp=None,
        frame_id="camera_depth_frame",
    )

    assert msg.header.frame_id == "camera_depth_frame"
    assert msg.width == 640
    assert msg.height == 400
    assert list(msg.k) == [519.4, 0.0, 320.6, 0.0, 519.1, 201.3, 0.0, 0.0, 1.0]
    assert list(msg.p) == [519.4, 0.0, 320.6, 0.0, 0.0, 519.1, 201.3, 0.0, 0.0, 0.0, 1.0, 0.0]
    assert list(msg.d) == [0.1, 0.2, 0.3, 0.4, 0.5]
