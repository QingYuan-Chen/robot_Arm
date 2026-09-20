import pytest
from rebotarm_calibration.session_store import SessionStore
from test_handeye_workflow import dataset


def test_durable_workflow_retry_accept_and_restart(tmp_path):
    source = dataset()
    store = SessionStore(tmp_path)
    data = store.create(source['metadata'])
    sid = data['session_id']
    for split in ('training', 'validation'):
        for sample in source[split + '_samples']:
            revision = data['revision']
            payload = {'split': split, 'sample': sample}
            data = store.mutate(sid, request_id=sample['sample_id'], revision=revision,
                                operation='capture', payload=payload)
            assert store.mutate(sid, request_id=sample['sample_id'], revision=revision,
                                operation='capture', payload=payload) == data
    store = SessionStore(tmp_path)
    data = store.mutate(sid, request_id='solve', revision=data['revision'], operation='solve')
    assert data['report']['passed']
    with pytest.raises(ValueError, match='confirmation'):
        store.mutate(sid, request_id='accept', revision=data['revision'], operation='accept')
    accepted = store.mutate(sid, request_id='accept', revision=data['revision'], operation='accept',
                            payload={'confirmed': True, 'operator': 'test'})
    assert accepted['state'] == 'accepted'
    assert accepted['report']['deployed'] is False
    with pytest.raises(ValueError, match='terminal'):
        store.mutate(sid, request_id='again', revision=accepted['revision'], operation='solve')


def test_stale_requests_and_bad_paths_do_not_mutate(tmp_path):
    store = SessionStore(tmp_path)
    data = store.create(dataset()['metadata'])
    sid = data['session_id']
    with pytest.raises(ValueError, match='stale'):
        store.mutate(sid, request_id='bad', revision=1, operation='abort', payload={'reason':'test'})
    with pytest.raises(ValueError):
        store.read('../outside')
    assert store.read(sid) == data
    with pytest.raises(ValueError):
        store.mutate(sid, request_id='solve', revision=0, operation='solve')
    assert store.read(sid) == data


@pytest.mark.parametrize('changed', ['intrinsics', 'settings', 'missing'])
def test_capture_conditions_frozen_across_splits_and_restart(tmp_path, changed):
    from copy import deepcopy
    source=dataset();store=SessionStore(tmp_path)
    session=store.create(source['metadata']);sid=session['session_id']
    sample=deepcopy(source['training_samples'][0])
    sample['camera_info']={'k':[520.,0.,320.,0.,520.,240.,0.,0.,1.],'width':640,'height':480}
    sample['provenance']={'capture_settings':{'maximum_age_sec':.5}}
    session=store.mutate(sid,request_id='first',revision=0,operation='capture',payload={'split':'training','sample':sample})
    next_sample=deepcopy(sample)
    if changed=='intrinsics':next_sample['camera_info']['k'][0]=521.
    elif changed=='settings':next_sample['provenance']['capture_settings']['maximum_age_sec']=1.
    else:next_sample.pop('provenance')
    store=SessionStore(tmp_path)
    with pytest.raises(ValueError,match='conditions changed'):
        store.mutate(sid,request_id='second',revision=1,operation='capture',payload={'split':'validation','sample':next_sample})
    assert store.read(sid)==session
    next_sample=deepcopy(sample)
    next_sample['base_to_end']=source['training_samples'][1]['base_to_end']
    assert store.mutate(sid,request_id='second',revision=1,operation='capture',payload={'split':'validation','sample':next_sample})['revision']==2


def test_failed_report_cannot_be_accepted_and_reopen_invalidates_report(tmp_path):
    from copy import deepcopy
    source=dataset();store=SessionStore(tmp_path)
    current=store.create(source['metadata']);sid=current['session_id']
    for split in ('training','validation'):
        for sample in source[split+'_samples']:
            sample=deepcopy(sample)
            if split=='validation':sample['camera_to_marker']['translation'][0]+=.1
            current=store.mutate(sid,request_id=sample['sample_id'],revision=current['revision'],
                                 operation='capture',payload={'split':split,'sample':sample})
    current=store.mutate(sid,request_id='solve',revision=current['revision'],operation='solve')
    assert not current['report']['passed']
    with pytest.raises(ValueError,match='passing report'):
        store.mutate(sid,request_id='accept',revision=current['revision'],operation='accept',
                     payload={'confirmed':True,'operator':'test'})
    assert store.read(sid)==current
    current=store.mutate(sid,request_id='reopen',revision=current['revision'],operation='reopen')
    assert current['state']=='active' and current['report'] is None
    with pytest.raises(ValueError,match='passing report'):
        store.mutate(sid,request_id='accept',revision=current['revision'],operation='accept',
                     payload={'confirmed':True,'operator':'test'})


def test_returned_session_is_detached(tmp_path):
    store=SessionStore(tmp_path)
    source=dataset();current=store.create(source['metadata']);sid=current['session_id']
    current['metadata']['base_frame']='corrupted'
    assert store.read(sid)['metadata']['base_frame']==source['metadata']['base_frame']
    current=store.mutate(sid,request_id='first',revision=0,operation='capture',
                         payload={'split':'training','sample':source['training_samples'][0]})
    current['training_samples'][0]['base_to_end']['translation'][0]=999
    assert store.read(sid)['training_samples'][0]['base_to_end']['translation'][0]!=999


def test_audit_indices_and_retries_are_stable(tmp_path):
    from datetime import datetime
    store=SessionStore(tmp_path);source=dataset()
    current=store.create(source['metadata']);sid=current['session_id']
    assert datetime.fromisoformat(current['created_at']).tzinfo is not None
    first={'split':'training','sample':source['training_samples'][0]}
    current=store.mutate(sid,request_id='first',revision=0,operation='capture',payload=first)
    repeated=store.mutate(sid,request_id='first',revision=0,operation='capture',payload=first)
    assert repeated==current and len(current['audit'])==1
    current=store.mutate(sid,request_id='second',revision=1,operation='capture',
        payload={'split':'validation','sample':source['validation_samples'][0]})
    assert current['training_samples'][0]['sample_index']==0
    assert current['validation_samples'][0]['sample_index']==1
    assert [a['revision'] for a in current['audit']]==[1,2]
    assert current['audit'][-1]['at']==current['updated_at']
    assert SessionStore(tmp_path).read(sid)==current
