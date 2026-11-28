import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';
import {selectConfiguration,validateSelectionInput,validateSelectionResult,selectionStatus} from '../server/configuration-selection.mjs';
import {resultHTML} from '../dist/configuration-selection-page.mjs';
const env={ARM_SELECTION_AGENT_COMMAND:process.execPath,ARM_SELECTION_AGENT_ARGS:JSON.stringify([fileURLToPath(new URL('./fixtures/selection-agent.mjs',import.meta.url))])};

test('scenario input keeps unknowns and validates numeric units without example defaults',()=>{
  const input=validateSelectionInput({scenario:'  工位取放  ',workpiece_mass_kg:0});
  assert.equal(input.scenario,'工位取放');assert.equal(input.workpiece_mass_kg,0);
  assert.equal(input.tool_mass_kg,null);assert.equal(input.reach_mm,null);assert.equal(input.installation,null);
  assert.throws(()=>validateSelectionInput({}),/场景/);
  for(const value of [-1,Infinity,'3'])assert.throws(()=>validateSelectionInput({task:'取放',tool_mass_kg:value}));
});
test('real bridge protocol loads bundled skill, returns traceable results, and renders escaped content',async()=>{
  assert.equal((await selectionStatus({ARM_SELECTION_PROVIDER:'external'})).configured,false);
  assert.equal((await selectionStatus(env)).configured,true);
  await assert.rejects(selectConfiguration({scenario:'test'},{env:{ARM_SELECTION_PROVIDER:'external'}}),/尚未配置/);
  const data=await selectConfiguration({scenario:'<img src=x onerror=alert(1)>'},{env});
  assert.equal(data.result.outcome,'needs_requirements');
  assert.equal(data.input.reach_mm,null);
  assert.equal(data.execution.sources.length,3);
  assert.ok(data.execution.sources.every(source=>/^[a-f0-9]{64}$/.test(source.sha256)));
  assert.match(resultHTML(data),/&lt;img/);assert.doesNotMatch(resultHTML(data),/<img/);
  await assert.rejects(selectConfiguration({scenario:'bad-output'},{env}),/契约/);
  await assert.rejects(selectConfiguration({scenario:'slow'},{env:{...env,ARM_SELECTION_TIMEOUT_MS:'100'}}),/SKILL_TIMEOUT/);
  const controller=new AbortController();
  const pending=selectConfiguration({scenario:'slow'},{env,signal:controller.signal});
  setTimeout(()=>controller.abort(Error('selection-cancelled')),100);
  await assert.rejects(pending,/selection-cancelled/);
});
test('candidate output requires complete configuration, valid constraints and scoped evidence',()=>{
  const candidate={id:'C1',status:'conditional',configuration:Object.fromEntries(['arm_dof','axis_layout','elbow','wrist','drive_layout','sensing_control','deployment'].map(key=>[key,null])),fit_reasons:[],tradeoffs:[],pending_validations:['核对工具姿态'],constraint_checks:[{requirement_id:'R1',result:'unknown'}],
    evidence:[{source_id:'G01',locator:'知识库某章节',evidence_type:'engineering_inference',claim_kind:'inference',supports:'条件路线',scope:'待验证',supporting_source_ids:['E01'],assumptions:[],pending_validations:[]}]};
  const result={schema_name:'arm-configuration-selection',schema_version:'1.0',outcome:'shortlist',conclusion:'条件候选',requirements:[{id:'R1',name:'task',value:'test',kind:'task',origin:'user_input'}],candidates:[candidate],open_questions:[],validation_plan:[]};
  validateSelectionResult(result);
  const noEvidence=structuredClone(result);noEvidence.candidates[0].evidence=[];
  assert.throws(()=>validateSelectionResult(noEvidence),/来源/);
  const wrongReference=structuredClone(result);wrongReference.candidates[0].constraint_checks[0].requirement_id='missing';
  assert.throws(()=>validateSelectionResult(wrongReference),/引用/);
});

test('configuration HTTP routes serve frontend, health, validation and Agent results',async()=>{
  const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('URL=http://127.0.0.1:'+process.env.PORT);});});`;
  const server=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env:{...process.env,...env},stdio:['ignore','pipe','pipe']});
  try{
    const url=await new Promise((resolve,reject)=>{let output='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);server.stdout.on('data',chunk=>{output+=chunk;const match=output.match(/URL=(http:\/\/\S+)/);if(match){clearTimeout(timer);resolve(match[1]);}});server.on('error',reject);server.on('exit',code=>{clearTimeout(timer);reject(Error('server exit '+code));});});
    const html=await (await fetch(url)).text();assert.match(html,/data-tab="configuration"/);
    assert.equal((await fetch(url+'/configuration-selection-page.mjs')).status,200);
    assert.equal((await (await fetch(url+'/api/configuration-selection/health')).json()).configured,true);
    const post=body=>fetch(url+'/api/configuration-selection',{method:'POST',headers:{'content-type':'application/json'},body});
    assert.equal((await post('broken')).status,400);
    assert.equal((await post('{}')).status,422);
    const response=await post(JSON.stringify({scenario:'工位取放'}));assert.equal(response.status,200);
    assert.equal((await response.json()).result.outcome,'needs_requirements');
    assert.equal((await post(JSON.stringify({scenario:'bad-output'}))).status,502);
  }finally{server.kill('SIGTERM');}
});

test('page submits scene data, preserves unknowns and displays results on the same page',async()=>{
  const {ConfigurationSelectionPage}=await import('../dist/configuration-selection-page.mjs');
  const data=await selectConfiguration({scenario:'UI 场景'},{env});
  const original={document:globalThis.document,location:globalThis.location,fetch:globalThis.fetch};
  const controls=[{name:'scenario',value:'UI 场景'},{name:'task',value:''},...['workpiece_mass_kg','tool_mass_kg','reach_mm','cycle_time_s'].map(name=>({name,value:''}))];
  const nodes={};
  const node=id=>({id,elements:id==='selectionForm'?controls:undefined,textContent:'',get innerHTML(){return this.html??'';},set innerHTML(value){this.html=value;if(id==='content')for(const match of value.matchAll(/id="([^"]+)"/g))nodes[match[1]]=node(match[1]);}});
  nodes.content=node('content');let submitted;
  globalThis.document={getElementById:id=>nodes[id]??null};
  globalThis.location={hash:'#configuration'};
  globalThis.fetch=async(url,options)=>{
    if(!options)return {ok:true,json:async()=>({configured:true})};
    submitted=JSON.parse(options.body);return {ok:true,json:async()=>data};
  };
  try{
    nodes.content.innerHTML=ConfigurationSelectionPage.html();
    ConfigurationSelectionPage.bind();
    await new Promise(resolve=>setImmediate(resolve));
    await nodes.selectionForm.onsubmit({preventDefault(){}});
    assert.equal(submitted.scenario,'UI 场景');
    assert.equal(submitted.workpiece_mass_kg,null);assert.equal(submitted.reach_mm,null);
    assert.equal(globalThis.location.hash,'#configuration');
    assert.match(nodes.content.innerHTML,/需求与未知项/);
    assert.match(nodes.content.innerHTML,/选择结论与下一步/);
    assert.match(nodes.content.innerHTML,/selectionJSON/);
  }finally{
    for(const [key,value] of Object.entries(original)){if(value===undefined)delete globalThis[key];else globalThis[key]=value;}
  }
});
