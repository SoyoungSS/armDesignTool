import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {spawn} from 'node:child_process';
import {analyze} from '../../arm-kinematics-dynamics/scripts/lib/analyze.mjs';
const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('URL=http://127.0.0.1:'+process.env.PORT);});});`;
const child=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env:process.env,stdio:['ignore','pipe','pipe']});
try{
 const url=await new Promise((resolve,reject)=>{let text='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);child.stdout.on('data',b=>{text+=b;const m=text.match(/URL=(http:\/\/\S+)/);if(m){clearTimeout(timer);resolve(m[1]);}});child.on('error',e=>{clearTimeout(timer);reject(e);});child.on('exit',code=>{clearTimeout(timer);reject(Error('server exit '+code));});});
 await test('motion routes serve separate model requests and browser modules',async()=>{
  for(const path of ['/motion-worksheet.json','/motion-fr3.json','/motion-worksheet-dynamics.json','/motion-export.mjs','/motion-client.mjs','/motion-page.mjs','/api/motion/health'])assert.equal((await fetch(url+path)).status,200,path);
  assert.equal((await fetch(url+'/motion-secret.json')).status,404);
 });
 await test('real motion CLI HTTP bridge equals independent API for both geometry-only-capable examples',async()=>{
  for(const name of ['worksheet','franka-fr3','worksheet-dynamics']){const request=JSON.parse(await readFile(new URL(`../../arm-kinematics-dynamics/assets/examples/${name==='worksheet-dynamics'?name:name+'-motion'}.json`,import.meta.url),'utf8'));
   const response=await fetch(url+'/api/motion',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(request)});assert.equal(response.status,200);const data=await response.json();assert.equal(data.execution.execution_mode,'skill_cli');assert.deepEqual(data.result,analyze(request));assert.equal(data.result.scope.dynamics_computed,name==='worksheet-dynamics');
  }
 });
 await test('motion invalid input and JSON report errors without fallback trajectories',async()=>{
  assert.equal((await fetch(url+'/api/motion',{method:'POST',body:'{}'})).status,415);
  for(const body of ['{}','{broken']){const response=await fetch(url+'/api/motion',{method:'POST',headers:{'content-type':'application/json'},body});assert.equal(response.status,422);const data=await response.json();assert.equal(data.error,'INVALID_INPUT');assert.equal(data.result,undefined);}
 });
 await test('random generation through the CLI bridge returns reusable analysis requests',async()=>{
  const template=await (await fetch(url+'/motion-fr3.json')).json();
  const post=async request=>fetch(url+'/api/motion',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(request)});
  const generation={schema_name:'arm-random-motion-request',schema_version:'1.0',template,
    settings:{seed:42,trajectory_count:3,move_count:3,max_step_fraction:.1,min_speed_fraction:.4,max_speed_fraction:.8,min_hold_s:0,max_hold_s:.5}};
  const generated=await post(generation);
  assert.equal(generated.status,200);
  const batch=await generated.json();
  assert.equal(batch.execution.execution_mode,'skill_cli');
  assert.equal(batch.result.trajectories.length,3);
  const response=await post(batch.result.trajectories[1].request);
  assert.equal(response.status,200);
  const data=await response.json();
  assert.equal(data.result.summary.constraint_status,'verified');
  generation.settings.seed=-1;
  assert.equal((await post(generation)).status,422);
 });
}finally{child.kill('SIGTERM');}
