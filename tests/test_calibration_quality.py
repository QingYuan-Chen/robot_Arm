import numpy as np
from test_handeye_workflow import dataset
from rebotarm_calibration.calibration_quality import sample_coverage, bootstrap_diagnostic
from rebotarm_calibration.handeye_solver import solve_eye_in_hand


def test_coverage_empty_and_diverse():
    assert not sample_coverage([])['ready']
    samples = dataset()['training_samples']
    assert sample_coverage(samples)['ready']
    assert not sample_coverage([samples[0]] * 6)['ready']


def test_bootstrap_reproducible_and_honest_about_failure():
    samples = dataset()['training_samples']
    reference = solve_eye_in_hand(samples, method='PARK')
    first = bootstrap_diagnostic(samples, 'PARK', reference, iterations=16)
    assert first == bootstrap_diagnostic(samples, 'PARK', reference, iterations=16)
    assert first['available']
    assert max(first['translation_std_m']) < 1e-8
    bad = bootstrap_diagnostic([samples[0]]*5, 'PARK', reference, iterations=16)
    assert not bad['available'] and bad['failed_draws'] == 16


def test_residual_diagnostics_marks_outlier_without_removing_samples():
    from copy import deepcopy
    from rebotarm_calibration.calibration_quality import residual_diagnostics
    samples=[{'label':str(i),'position_residual_m':.001,'rotation_residual_deg':.1} for i in range(6)]
    samples[-1]['position_residual_m']=.03
    before=deepcopy(samples)
    report=residual_diagnostics(samples)
    assert samples==before
    assert [s['label'] for s in report['flagged_samples']]==['5']
    assert 'position_residual_m:absolute_limit' in report['flagged_samples'][0]['reasons']
    assert 'position_residual_m:relative_outlier' in report['flagged_samples'][0]['reasons']
    for sample in samples:sample['position_residual_m']=.03
    assert len(residual_diagnostics(samples)['flagged_samples'])==6
