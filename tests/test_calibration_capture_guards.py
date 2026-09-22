"""Exercise actual capture methods with a deterministic clock and ROS adapters."""
from types import SimpleNamespace as NS
import json
import pytest
import numpy as np


def node_module():
    from test_tcp_calibration import _install_ros_stubs_if_needed
    _install_ros_stubs_if_needed()
    from rebotarm_calibration import tcp_calibration_node
    return tcp_calibration_node


def test_no_camera_frames_times_out(monkeypatch):
    mod = node_module()
    tick = iter([0, 0.1, 0.2, 1.1])
    monkeypatch.setattr(mod.time, 'monotonic', lambda: next(tick))
    monkeypatch.setattr(mod.rclpy, 'ok', lambda: True, raising=False)
    monkeypatch.setattr(mod.rclpy, 'spin_once', lambda *a, **k: None, raising=False)
    node = NS(reference_mode='aruco', capture_timeout_sec=1, maximum_frames=45,
              reference_frames=30, latest_image=None, latest_camera_info=None,
              get_parameter=lambda _: NS(value=1))
    with pytest.raises(RuntimeError, match='timeout'):
        mod.TcpCalibrationNode.prepare_reference(node)


def test_stamp_rejects_old_future_zero():
    mod = node_module()
    node = NS(maximum_age_sec=.5, get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
    for sec in (0, 8, 11):
        with pytest.raises(ValueError, match='timestamp'):
            mod.TcpCalibrationNode._check_stamp(node, NS(sec=sec, nanosec=0))
    assert mod.TcpCalibrationNode._check_stamp(node, NS(sec=10, nanosec=0)) == 10_000_000_000


def test_invalid_tcp_limits_and_strict_failure_json():
    from rebotarm_calibration.tcp_calibration import analyze_tcp_samples, analyze_tcp_pivot_samples
    sample = {'end_link_position': [0, 0, 0], 'end_link_orientation_xyzw': [0, 0, 0, 1],
              'tcp_reference_position': [0, 0, .1]}
    for f in (analyze_tcp_samples, analyze_tcp_pivot_samples):
        for limit in (0, -1, float('inf'), float('nan')):
            with pytest.raises(ValueError):
                f([sample]*5, maximum_rms_residual_m=limit)
    result = analyze_tcp_pivot_samples([sample]*5)
    assert not result['passed']
    assert result['condition_number'] is None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('fault', ['frame', 'size', 'skew', 'distortion', 'quality'])
def test_reference_rejects_mismatched_or_poor_frames(monkeypatch, fault):
    mod = node_module()
    stamp = NS(sec=10, nanosec=0)
    image = NS(header=NS(stamp=stamp, frame_id='optical'), width=640, height=480)
    info = NS(header=NS(stamp=stamp, frame_id='optical'), width=640, height=480,
              k=np.eye(3).ravel(), d=[], distortion_model='plumb_bob')
    if fault == 'frame': info.header.frame_id = 'other'
    if fault == 'size': info.width = 320
    if fault == 'skew': info.header.stamp = NS(sec=9, nanosec=0)
    if fault == 'distortion': info.distortion_model = 'equidistant'
    monkeypatch.setattr(mod.rclpy, 'spin_once', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(mod.rclpy, 'ok', lambda: True, raising=False)
    monkeypatch.setattr(mod, 'detect_aruco_pose', lambda *a, **k: {
        'camera_to_marker': {'translation': [0, 0, .4]}, 'reprojection_rmse_px': 10, 'area_px2': 1000})
    node = NS(reference_mode='aruco', capture_timeout_sec=1, maximum_frames=1,
              reference_frames=1, latest_image=image, latest_camera_info=info,
              camera_frame='optical', aruco_maximum_info_skew_sec=.1,
              aruco_maximum_reprojection_rmse_px=1,
              get_parameter=lambda _: NS(value=1),
              _check_stamp=lambda s: s.sec * 10**9,
              bridge=NS(imgmsg_to_cv2=lambda *a, **k: None),
              get_logger=lambda: NS(warn=lambda _: None))
    with pytest.raises(RuntimeError, match='gate failed'):
        mod.TcpCalibrationNode.prepare_reference(node)


def test_sample_requires_progressing_stable_tf(monkeypatch):
    mod = node_module()
    monkeypatch.setattr(mod.rclpy, 'spin_once', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(mod.rclpy, 'ok', lambda: True, raising=False)
    stamps = iter([1_000_000_000, 1_000_000_000, 1_200_000_000, 1_400_000_000])
    node = NS(capture_timeout_sec=1, base_frame='base', end_link_frame='end',
              stability_translation_m=.001, stability_rotation_deg=.5, stability_window_sec=.4,
              reference_mode='pivot', samples=[], _check_stamp=lambda s: s)
    node._lookup = lambda *a: NS(header=NS(stamp=next(stamps)), transform=NS(
        translation=NS(x=0, y=0, z=0), rotation=NS(x=0, y=0, z=0, w=1)))
    sample = mod.TcpCalibrationNode.capture_sample(node)
    assert sample['tf_stamp_ns'] == 1_400_000_000
    assert len(node.samples) == 1
