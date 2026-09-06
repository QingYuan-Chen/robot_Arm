from types import SimpleNamespace

import numpy as np
import pytest

from rebotarm_vision.offline_yolo import (
    decode_image_to_bgr,
    filter_detection_array,
    parse_target_classes,
)
from sensor_msgs.msg import Image


def _image(*, encoding, width, height, step, data):
    message = Image()
    message.encoding = encoding
    message.width = width
    message.height = height
    message.step = step
    message.data = data
    return message


def test_parse_target_classes_accepts_launch_yaml_and_normalizes():
    assert parse_target_classes("['Cube', ' bottle ']") == ("cube", "bottle")
    assert parse_target_classes("bottle, cup") == ("bottle", "cup")
    assert parse_target_classes([]) == ()


def test_decode_rgb8_with_row_padding_returns_bgr():
    message = _image(
        encoding="rgb8",
        width=2,
        height=2,
        step=8,
        data=bytes(
            [
                1,
                2,
                3,
                4,
                5,
                6,
                99,
                99,
                7,
                8,
                9,
                10,
                11,
                12,
                98,
                98,
            ]
        ),
    )
    result = decode_image_to_bgr(message)
    assert result.flags["C_CONTIGUOUS"]
    np.testing.assert_array_equal(
        result,
        np.asarray([[[3, 2, 1], [6, 5, 4]], [[9, 8, 7], [12, 11, 10]]], dtype=np.uint8),
    )


def test_decode_rejects_unsupported_or_truncated_image():
    with pytest.raises(ValueError, match="supports rgb8"):
        decode_image_to_bgr(_image(encoding="mono8", width=1, height=1, step=1, data=b"\x00"))
    with pytest.raises(ValueError, match="needs at least"):
        decode_image_to_bgr(_image(encoding="rgb8", width=2, height=1, step=6, data=b"\x00" * 5))


def test_filter_detection_array_is_explicit_allowlist_and_fail_closed():
    detections = SimpleNamespace(header=SimpleNamespace(), detections=[])
    for class_name in ("Cube", "fire hydrant"):
        detection = SimpleNamespace(class_name=class_name)
        detections.detections.append(detection)

    filtered = filter_detection_array(detections, ["cube"])
    assert [item.class_name for item in filtered.detections] == ["Cube"]
    assert filter_detection_array(detections, []).detections == []
