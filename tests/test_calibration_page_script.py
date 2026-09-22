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
function element(id){if(!elements.has(id))elements.set(id,{id,value:'',disabled:false,replaceChildren(){this.children=[];},append(...children){this.children=(this.children||[]).concat(children);},textContent:'',dataset:{}});return elements.get(id);}
for(const id of ['create','restore','capture','gravity-stop','gravity-start','enable-arm','safe-home','disable-arm','apply-exclusions','abort','accept'])element(id);
global.document={getElementById:element,createElement:()=>({style:{},replaceChildren(){this.children=[];},append(...children){this.children=(this.children||[]).concat(children);},set id(value){this._id=value;elements.set(value,this);},get id(){return this._id;}}),querySelectorAll:selector=>selector==='button'?[...elements.values()]:selector==='.outlier-choice:checked'?(element('outlier-options').children||[]).flatMap(label=>label.children||[]).filter(input=>input.checked):[]};
element('outlier-options').replaceChildren=function(){this.children=[];};
global.localStorage={getItem:()=>null,setItem(){}};
global.crypto={randomUUID:()=> '12345678-1234-1234-1234-123456789abc'};
global.EventSource=class{addEventListener(){}};
global.setInterval=()=>0;
global.confirm=()=>true;
let resolveCapture;const calls=[];
global.fetch=(path,options)=>{calls.push([path,JSON.parse(options.body)]);if(path==='/api/calibration/command')return new Promise(resolve=>resolveCapture=resolve);return Promise.resolve({json:async()=>({accepted:true,message:'hold'})});};
'''
    assertions = r'''
(async()=>{
 streamFresh=true;feedbackFresh=true;armStatus={enabled:true,state_machine:'IDLE'};session={state:'active'};
 checklist({image:{state:'passed',detail:'fresh'},camera_info:{state:'failed',detail:'missing'}});
 assert.equal(element('preflight-list').children.length,5);
 assert.equal(element('preflight-list').children[0].className,'preflight-row passed');
 assert.equal(element('preflight-list').children[1].className,'preflight-row failed');
 assert.equal(element('preflight-list').children[2].className,'preflight-row pending');
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
 const fixture={schema_version:1,session_id:'a'.repeat(32),revision:4,state:'solved',metadata:{camera_frame:'optical',marker_frame:'board_1',marker_length_m:0.08,marker_id:7,dictionary:'DICT_5X5_100'},training_samples:[{sample_id:'t',base_to_end:{translation:[1,2,3]}}],validation_samples:[{sample_id:'v'}],report:{passed:false,accepted:false,deployed:false,methods:{PARK:{passed:false,error:'test rejection'}}}};
 show(fixture);
 assert.equal(element('mode').disabled,true);
 assert.equal(element('marker_length_m').disabled,true);
 assert.ok(element('count').textContent.includes('有效'));
 for(const id of ['marker_frame','marker_length_m','marker_id','dictionary'])assert.ok(elements.has(id),id+' input missing');
 assert.equal(element('marker_frame').value,'board_1');
 assert.equal(element('marker_length_m').value,0.08);
 assert.equal(element('marker_id').value,7);
 assert.equal(element('dictionary').value,'DICT_5X5_100');
 await element('safe-home').onclick();
 assert.equal(calls.at(-1)[0],'/api/arm_safe_home');
 await element('disable-arm').onclick();
 assert.equal(calls.at(-1)[0],'/api/arm_disable');
 assert.equal(elements.has('download'),false);
 element('marker_frame').value='board_2';element('marker_length_m').value='0.12';element('marker_id').value='9';element('dictionary').value='DICT_6X6_250';
 element('create').onclick();
 const created=calls.at(-1)[1];
 assert.equal(created.command,'create');
 assert.deepEqual([created.payload.metadata.marker_frame,created.payload.metadata.marker_length_m,created.payload.metadata.marker_id,created.payload.metadata.dictionary],['board_2',0.12,9,'DICT_6X6_250']);
 resolveCapture({json:async()=>({success:false,message:'test'})});
 await new Promise(resolve=>setImmediate(resolve));
 const flagged={...fixture,state:'solved',report:{passed:false,accepted:false,deployed:false,selected_method:'PARK',methods:{PARK:{training:{diagnostics:{flagged_samples:[{label:'t',reasons:['relative_outlier']}]}},validation:{diagnostics:{flagged_samples:[]}}}}}};
 show(flagged);
 assert.equal(element('exclusion-panel').hidden,false);
 assert.equal(element('outlier-options').children.length,1);
 element('outlier-options').children[0].children[0].checked=true;
 element('exclusion-reason').value='bad observation';
 element('apply-exclusions').onclick();
 const exclusion=calls.at(-1)[1];
 assert.equal(exclusion.command,'exclude_samples');
 assert.deepEqual(exclusion.payload,{sample_ids:['t'],reason:'bad observation',evidence:''});
 resolveCapture({json:async()=>({success:false,message:'test'})});
 await new Promise(resolve=>setImmediate(resolve));
 const countBefore=calls.length;
 armBusy=true;
 updateControls();
 assert.equal(element('capture').disabled,true);
 await element('safe-home').onclick();
 assert.equal(calls.length,countBefore);
 armBusy=false;feedbackFresh=false;updateControls();
 assert.equal(element('safe-home').disabled,true);
 assert.equal(element('gravity-stop').disabled,false);
 await element('safe-home').onclick();
 assert.equal(calls.length,countBefore);
 feedbackFresh=true;armUnknown=true;updateControls();
 assert.equal(element('capture').disabled,true);
 await element('capture').onclick();
 assert.equal(calls.length,countBefore);
 armUnknown=false;
 element('copy-session').onclick();
 assert.equal(session,null);
 assert.equal(element('marker_length_m').disabled,false);

})().catch(e=>{console.error(e);process.exitCode=1});
'''
    file = tmp_path / 'page.cjs'
    file.write_text(harness + script + assertions)
    result = subprocess.run([node, str(file)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
