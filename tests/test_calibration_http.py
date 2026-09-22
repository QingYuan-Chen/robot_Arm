import json
import threading
from types import SimpleNamespace as NS
from urllib.request import Request, urlopen
from rebotarm_dashboard.status_panel_http import create_status_panel_server
from rebotarm_dashboard.status_panel_state import TeleopStatusStore


def test_page_and_calibration_success_http_status(tmp_path):
    received=[]
    def command(payload):
        received.append(payload)
        return {'success':True,'reason_code':'OK','session':{'revision':0}}
    server=create_status_panel_server(host='127.0.0.1',port=0,
        node=NS(_handle_calibration_command=command),store=TeleopStatusStore(),
        html_page='dashboard',urdf_path=tmp_path/'robot.urdf',mesh_dir=tmp_path,sse_interval_sec=.1)
    thread=threading.Thread(target=server.serve_forever);thread.start()
    base=f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base+'/calibration') as response:
            page=response.read().decode()
            assert response.status==200 and 'gravity-start' in page and 'EventSource' in page
        request=Request(base+'/api/calibration/command',data=b'{"command":"status"}',
                        headers={'Content-Type':'application/json'})
        with urlopen(request) as response:
            assert response.status==200 and json.load(response)['success']
        assert received==[{'command':'status'}]
        with urlopen(base+'/api/calibration/export?session_id='+'a'*32) as response:
            assert response.headers['Content-Disposition']=='attachment; filename="'+('a'*32)+'.json"'
            assert response.headers['Cache-Control']=='no-store'
            assert json.load(response)=={'revision':0}
        from urllib.error import HTTPError
        import pytest
        with pytest.raises(HTTPError) as failure:
            urlopen(base+'/api/calibration/export?session_id=../private')
        assert failure.value.code==400
        assert len(received)==2

    finally:
        server.shutdown();server.server_close();thread.join()


def test_multiple_sse_clients_disconnect_and_resume(tmp_path):
    store=TeleopStatusStore()
    server=create_status_panel_server(host='127.0.0.1',port=0,node=NS(),store=store,
        html_page='test',urdf_path=tmp_path/'unused',mesh_dir=tmp_path,sse_interval_sec=.02)
    thread=threading.Thread(target=server.serve_forever);thread.start()
    base=f'http://127.0.0.1:{server.server_port}'
    clients=[]
    def event(client):
        for _ in range(12):
            line=client.readline()
            if line.startswith(b'data: '):return json.loads(line[6:])
        raise AssertionError('missing SSE data')
    try:
        store.update_teleop_status('calibration',{'session_id':'one','busy':True,'command':'capture'})
        clients=[urlopen(base+'/events',timeout=3) for _ in range(2)]
        for client in clients:
            data=event(client)['teleop']['calibration']
            assert data['session_id']=='one' and data['busy']
            assert 'samples' not in data
        clients[0].close()
        store.update_teleop_status('calibration',{'session_id':'one','busy':False,'revision':1,'success':True})
        for _ in range(20):
            data=event(clients[1])['teleop']['calibration']
            if not data['busy']:break
        assert data['revision']==1
        with urlopen(base+'/events',timeout=3) as reconnected:
            assert event(reconnected)['teleop']['calibration']['revision']==1
        with urlopen(base+'/api/status',timeout=3) as response:
            assert json.load(response)['teleop']['calibration']['success']
    finally:
        for client in clients:client.close()
        server.shutdown();server.server_close();thread.join()
