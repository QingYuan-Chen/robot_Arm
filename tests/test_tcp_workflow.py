import numpy as np
from test_handeye_workflow import dataset
from rebotarm_calibration.handeye_residual import transform_matrix,matrix_transform
from rebotarm_calibration.tcp_workflow import solve_tcp_dataset
from rebotarm_calibration.session_store import SessionStore


def test_tcp_holdout_and_persistent_workflow(tmp_path):
    data=dataset();offset=np.array([-.04,.002,.003]);pivot=np.array([.3,-.2,.2])
    for split in ('training_samples','validation_samples'):
        for sample in data[split]:
            pose=transform_matrix(sample['base_to_end']);pose[:3,3]=pivot-pose[:3,:3]@offset
            sample['base_to_end']=matrix_transform(pose)
    report=solve_tcp_dataset(data)
    assert report['passed']
    assert np.allclose(report['tcp_offset_xyz'],offset)
    store=SessionStore(tmp_path);session=store.create({'mode':'tcp','base_frame':'base','end_link_frame':'end'})
    for split in ('training','validation'):
        for sample in data[split+'_samples']:
            session=store.mutate(session['session_id'],request_id=sample['sample_id'],revision=session['revision'],operation='capture',payload={'split':split,'sample':{'base_to_end':sample['base_to_end']}})
    solved=store.mutate(session['session_id'],request_id='solve',revision=session['revision'],operation='solve')
    assert solved['report']['passed'] and solved['report']['kind']=='tcp_pivot'
    for sample in data['validation_samples']:sample['base_to_end']['translation'][0]+=.02
    assert not solve_tcp_dataset(data)['passed']
