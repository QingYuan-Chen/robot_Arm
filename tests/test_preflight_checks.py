from rebotarm_calibration.preflight_checks import PreflightChecks, PreflightFailure


def test_failure_does_not_mark_unreached_checks_passed():
    checks = PreflightChecks()
    checks.start('image')
    checks.passed('fresh')
    checks.start('camera_info')
    checks.failed('missing')
    error = PreflightFailure('missing', checks)
    assert error.checks['image']['state'] == 'passed'
    assert error.checks['camera_info']['state'] == 'failed'
    assert error.checks['aruco']['state'] == 'pending'
    checks.reset()
    assert checks.items['image']['state'] == 'pending'


def test_tcp_marks_camera_checks_not_applicable():
    checks = PreflightChecks(tcp=True)
    assert all(checks.items[key]['state'] == 'skipped' for key in ('image', 'camera_info', 'aruco'))
    assert checks.items['tf']['state'] == 'pending'
