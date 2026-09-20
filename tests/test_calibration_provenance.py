from rebotarm_calibration.provenance import runtime_provenance


def test_effective_settings_and_sources_are_recorded_without_storage_path():
    result=runtime_provenance({'maximum_age_sec':.5,'session_directory':'/private/output'})
    assert result['capture_settings']=={'maximum_age_sec':.5}
    assert len(result['source_sha256']['handeye_capture_node.py'])==64
    assert result['python_version'] and result['opencv_version']


def test_dataset_hash_covers_capture_evidence_not_session_state():
    from copy import deepcopy
    from test_handeye_workflow import dataset
    from rebotarm_calibration.provenance import dataset_fingerprint
    payload=dataset()
    original=dataset_fingerprint(payload)
    payload.update(state='accepted',revision=123,report={'passed':True},requests={'one':'hash'})
    assert dataset_fingerprint(payload)==original
    for key,value in [('camera_info',{'k':[520]}),('tf_stamp_ns',100),('provenance',{'version':'changed'})]:
        changed=deepcopy(payload)
        changed['training_samples'][0][key]=value
        assert dataset_fingerprint(changed)!=original
