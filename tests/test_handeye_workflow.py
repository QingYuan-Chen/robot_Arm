from copy import deepcopy
import json
import cv2
import numpy as np
import pytest
from rebotarm_calibration.handeye_residual import matrix_transform
from rebotarm_calibration.handeye_workflow import atomic_write_json, solve_dataset


def dataset():
    rng = np.random.default_rng(42)
    handeye = np.eye(4)
    handeye[:3, :3] = cv2.Rodrigues(np.array([.2, -.1, .3]))[0]
    handeye[:3, 3] = [-.08, .01, .04]
    marker = np.eye(4)
    marker[:3, 3] = [.2, -.3, .4]
    samples = []
    for index in range(16):
        end = np.eye(4)
        end[:3, :3] = cv2.Rodrigues(rng.uniform(-.5, .5, 3))[0]
        end[:3, 3] = rng.uniform(-.2, .2, 3)
        samples.append({'sample_id': str(index), 'base_to_end': matrix_transform(end),
                        'camera_to_marker': matrix_transform(np.linalg.inv(end @ handeye) @ marker)})
    return {'schema_version': 1, 'metadata': dict(base_frame='base', end_link_frame='end',
            camera_frame='optical', marker_frame='marker'),
            'training_samples': samples[:10], 'validation_samples': samples[10:]}


def test_exact_dataset_passes_without_acceptance():
    report = solve_dataset(dataset())
    assert report['passed']
    assert not report['accepted'] and not report['deployed']
    assert len(report['methods']) == 5
    assert report['methods'][report['selected_method']]['validation']['reference_source'] == 'provided'
    json.dumps(report, allow_nan=False)


def test_final_validation_requires_new_poses_after_selection_change():
    payload=dataset()
    for index,sample in enumerate(payload['training_samples']+payload['validation_samples']):
        sample['sample_index']=index
    payload['final_validation_after_index']=15
    report=solve_dataset(payload)
    assert not report['passed']
    assert report['final_validation_sample_count']==0
    assert report['methods'][report['selected_method']]['full_validation']['sample_count']==6
    # The unchanged sample poses are valid for this solver test; move the cutoff
    # before their recorded acquisition indices to model an independent cohort.
    payload['final_validation_after_index']=9
    report=solve_dataset(payload)
    assert report['passed']
    assert report['final_validation_sample_count']==6


def test_holdout_bias_fails_and_does_not_change_selection():
    payload = dataset()
    selected = solve_dataset(payload)['selected_method']
    for sample in payload['validation_samples']:
        sample['camera_to_marker']['translation'][0] += .1
    report = solve_dataset(payload)
    assert not report['passed']
    assert report['selected_method'] == selected


def test_duplicate_split_and_missing_frames_rejected():
    payload = dataset()
    payload['validation_samples'][0] = deepcopy(payload['training_samples'][0])
    with pytest.raises(ValueError, match='unique'):
        solve_dataset(payload)
    payload = dataset()
    del payload['metadata']['camera_frame']
    with pytest.raises(ValueError, match='camera_frame'):
        solve_dataset(payload)


def test_atomic_write_preserves_previous_on_failure(tmp_path, monkeypatch):
    path = tmp_path / 'report.json'
    atomic_write_json(path, {'old': True})
    with pytest.raises(ValueError):
        atomic_write_json(path, {'bad': float('nan')})
    import rebotarm_calibration.handeye_workflow as module
    def fail(*args):
        raise OSError('simulated failure')
    monkeypatch.setattr(module.os, 'replace', fail)
    with pytest.raises(OSError):
        atomic_write_json(path, {'new': True})
    assert json.loads(path.read_text()) == {'old': True}
    assert list(tmp_path.iterdir()) == [path]
