"""Execute shipped JS with DOM/transport doubles, including deferred requests."""
from pathlib import Path
import shutil
import subprocess
import pytest


def test_stop_remains_available_while_capture_waits(tmp_path):
    node = shutil.which('node')
    if not node:
        pytest.skip('Node.js unavailable')
    page = (Path(__file__).resolve().parents[1] / 'src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/calibration.html').read_text()
    script = page.split('<script>')[1].split('</script>')[0]
    harness = r'''
const assert=require('node:assert/strict');
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,{id,value:'',disabled:false,append(){},textContent:'',dataset:{}});return elements.get(id);}
for(const id of ['create','restore','capture','gravity-stop','gravity-start','enable-arm','download','abort','accept'])element(id);
let clickedDownload=null;global.document={getElementById:element,createElement:()=>({style:{},append(){},click(){clickedDownload=this;}}),querySelectorAll:selector=>selector==='button'?[...elements.values()]:[]};
global.localStorage={getItem:()=>null,setItem(){}};
global.crypto={randomUUID:()=> '12345678-1234-1234-1234-123456789abc'};
global.EventSource=class{addEventListener(){}};
global.confirm=()=>true;
const nativeFetch=global.fetch;let resolveCapture;const calls=[];
global.fetch=(path,options)=>{calls.push([path,JSON.parse(options.body)]);if(path==='/api/calibration/command')return new Promise(resolve=>resolveCapture=resolve);return Promise.resolve({json:async()=>({accepted:true,message:'hold'})});};
'''
    assertions = r'''
(async()=>{
 const pending=command('capture',{split:'training'});
 assert.equal(element('gravity-stop').disabled,false);
 assert.equal(element('capture').disabled,true);
 await element('gravity-stop').onclick();
 assert.equal(calls[1][0],'/api/calibration/gravity');
 assert.equal(calls[1][1].command,'stop');
 resolveCapture({json:async()=>({success:false,message:'no camera'})});
 await pending;
 assert.equal(element('capture').disabled,false);
 assert.equal(calls.length,2);
 const fixture={schema_version:1,session_id:'a'.repeat(32),revision:4,state:'solved',metadata:{camera_frame:'optical'},training_samples:[{sample_id:'t',base_to_end:{translation:[1,2,3]}}],validation_samples:[{sample_id:'v'}],report:{passed:false,accepted:false,deployed:false,methods:{PARK:{passed:false,error:'test rejection'}}}};
 show(fixture);
 element('download').onclick();
 assert.equal(clickedDownload.download,fixture.session_id+'.json');
 const blobResponse=await nativeFetch(clickedDownload.href);
 assert.deepEqual(await blobResponse.json(),fixture);

})().catch(e=>{console.error(e);process.exitCode=1});
'''
    file = tmp_path / 'page.cjs'
    file.write_text(harness + script + assertions)
    result = subprocess.run([node, str(file)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
