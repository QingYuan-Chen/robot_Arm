import numpy as np
from rebotarm_calibration.stability_window import StabilityWindow


def test_continuous_window_rejects_gap_and_clock_regression():
    window=StabilityWindow(.4,.2,.001,.5);pose=np.eye(4)
    assert not window.add(1_000_000_000,pose)
    assert not window.add(1_100_000_000,pose)
    assert not window.add(2_000_000_000,pose)  # no spanning a dropped stream
    assert len(window.values)==1
    assert not window.add(1_500_000_000,pose)
    assert len(window.values)==1
    for stamp in [1_600_000_000,1_700_000_000,1_800_000_000]:
        assert not window.add(stamp,pose)
    assert window.add(1_900_000_000,pose)


def test_motion_and_rejected_observation_reset_window():
    window=StabilityWindow(.4,.2,.001,.5);pose=np.eye(4)
    window.add(1_000_000_000,pose);window.add(1_100_000_000,pose)
    shifted=pose.copy();shifted[0,3]=.01
    assert not window.add(1_200_000_000,shifted)
    assert len(window.values)==1
    window.reset()
    assert not window.add(1_300_000_000,shifted)
