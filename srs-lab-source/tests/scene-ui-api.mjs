import test from 'node:test';
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {mkdtemp,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {SceneSimulationPage,jointExportsHTML} from '../dist/scene-simulation-page.mjs';

test('scene exports list every joint, standalone HTML and separate CSVs without truncating to the first charts',()=>{
 const modules=Array.from({length:7},(_,i)=>({joint:'J'+(i+1),joint_type:'hinge',state_samples:123,section_samples:123,status:'complete',peaks:{effort:i},state_csv:`data/joint_modules/${i}_state.csv`,actuator_csv:`data/joint_modules/${i}_actuators.csv`,charts:[{signal:'position',png:`figures/${i}_position.png`}]}));
 const job={id:'00000000-0000-0000-0000-000000000001',result:{run_path:'runs/current',joint_exports:{status:'exported',frames:123,html:'reports/report.html',wide_csv:'data/joint_states_wide.csv',summary_csv:'reports/joint_module_summary.csv',dictionary:'reports/data_dictionary.json',modules}}};
 const html=jointExportsHTML(job);
 assert.match(html,/下载全关节 HTML 总报告/);
 for(let i=1;i<=7;i++){assert.match(html,new RegExp(`J${i} 模组状态`));assert.match(html,new RegExp(`J${i} · 状态曲线`));}
 assert.equal((html.match(/_state.csv/g)??[]).length,7);
 const legacy=jointExportsHTML({id:job.id,result:{run_path:'runs/current',load_coverage:{recorded_joints:['J1','J2']}}});assert.match(legacy,/整理全关节 HTML 与 CSV/);assert.match(legacy,/无需再次调用 Agent/);
});

test('scene HTTP API serves the new page and asynchronous Agent plan without external services',async()=>{
 const root=await mkdtemp(path.join(tmpdir(),'arm-scene-http-'));
 const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('URL=http://127.0.0.1:'+process.env.PORT);});});`;
 const server=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env:{...process.env,ARM_SCENE_STORAGE:root,ARM_SCENE_AGENT_COMMAND:process.execPath,ARM_SCENE_AGENT_ARGS:JSON.stringify([fileURLToPath(new URL('./fixtures/scene-agent.mjs',import.meta.url))])},stdio:['ignore','pipe','pipe']});
 try{
  const url=await new Promise((resolve,reject)=>{let out='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);server.stdout.on('data',chunk=>{out+=chunk;const m=out.match(/URL=(http:\/\/\S+)/);if(m){clearTimeout(timer);resolve(m[1]);}});server.on('error',reject);server.on('exit',code=>{clearTimeout(timer);reject(Error('exit '+code));});});
  assert.equal((await fetch(url+'/scene-simulation-page.mjs')).status,200);
  assert.equal((await fetch(url+'/generated-model-view.mjs')).status,200);
  assert.deepEqual((await (await fetch(url+'/api/generated-models')).json()).models,[]);
  assert.equal((await fetch(url+'/api/generated-models/00000000-0000-0000-0000-000000000000/viewer')).status,404);
  assert.equal((await (await fetch(url+'/api/scene-simulation/health')).json()).configured,true);
  assert.deepEqual((await (await fetch(url+'/api/scene-simulation/models')).json()).models,[]);
  const post=(route,body)=>fetch(url+route,{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(body)});
  assert.equal((await post('/api/scene-simulation/plan',{})).status,422);
  const response=await post('/api/scene-simulation/plan',{description:'只描述自由运动，尚无模型'});assert.equal(response.status,202);
  const {job}=await response.json();let snapshot;
  for(let i=0;i<60;i++){snapshot=(await (await fetch(url+'/api/scene-simulation/jobs/'+job.id)).json()).job;if(snapshot.status!=='planning')break;await new Promise(resolve=>setTimeout(resolve,20));}
  assert.equal(snapshot.status,'needs_input');assert.ok(snapshot.plan.missing_inputs.length>0);
  assert.equal((await post('/api/scene-simulation/jobs/'+job.id+'/run',{plan_sha256:snapshot.plan_sha256})).status,422);
  assert.notEqual((await fetch(url+'/api/scene-simulation/jobs/'+job.id+'/artifact?path=../../../../README.md')).status,200);
  assert.equal((await fetch(url+'/api/scene-simulation/jobs/00000000-0000-0000-0000-000000000000')).status,404);
 }finally{server.kill('SIGTERM');await rm(root,{recursive:true,force:true});}
});

test('scene page sends description and unknowns then shows the same-page plan state',async()=>{
 const original={document:globalThis.document,location:globalThis.location,fetch:globalThis.fetch,window:globalThis.window};
 const controls=[{name:'description',value:'保持初始姿态'},{name:'model_id',value:''},{name:'cycles',value:''},{name:'max_time_s',value:''}];
 const nodes={};
 const node=id=>({id,elements:id==='sceneForm'?controls:undefined,textContent:'',get innerHTML(){return this.html??'';},set innerHTML(value){this.html=value;if(id==='content')for(const m of value.matchAll(/id="([^"]+)"/g))nodes[m[1]]=node(m[1]);}});
 nodes.content=node('content');let submitted;
 globalThis.document={getElementById:id=>nodes[id]??null};globalThis.location={hash:'#sorting'};globalThis.window={ResultViewer:{closeViewer(){}}};
 globalThis.fetch=async(url,options)=>{
  if(!options)return {ok:true,json:async()=>url.endsWith('/models')?{models:[]}:{configured:true,message:'本地测试'} };
  submitted=JSON.parse(options.body);return {ok:true,json:async()=>({ok:true,job:{id:'00000000-0000-0000-0000-000000000001',status:'needs_input',stage:'请补充模型',events:[],plan:null}})};
 };
 try{
  nodes.content.innerHTML=SceneSimulationPage.html();SceneSimulationPage.bind({go(){}});await new Promise(resolve=>setImmediate(resolve));
  assert.match(nodes.content.innerHTML,/场景描述与动作顺序/);assert.doesNotMatch(nodes.content.innerHTML,/红色放左盘/);
  await nodes.sceneForm.onsubmit({preventDefault(){}});
  assert.equal(submitted.description,'保持初始姿态');assert.equal(submitted.assumption_policy,'concept');assert.equal(submitted.auto_run,true);assert.equal(submitted.previous_job_id,null);assert.equal(submitted.cycles,null);assert.equal(submitted.max_time_s,null);
  assert.equal(location.hash,'#sorting');assert.match(nodes.content.innerHTML,/需要补充信息/);
 }finally{for(const [key,value] of Object.entries(original)){if(value===undefined)delete globalThis[key];else globalThis[key]=value;}}
});
