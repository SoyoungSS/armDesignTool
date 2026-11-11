import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {spawn} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {analyze as statics} from '../../arm-statics-parametric/scripts/lib/statics.mjs';
import {analyze as workspace} from '../../arm-workspace-parametric/scripts/lib/workspace.mjs';
const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('URL=http://127.0.0.1:'+process.env.PORT);});});`;
const server=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env:{...process.env,STATICS_SKILL_DIR:fileURLToPath(new URL('../../arm-statics-parametric',import.meta.url)),WORKSPACE_SKILL_DIR:fileURLToPath(new URL('../../arm-workspace-parametric',import.meta.url))},stdio:['ignore','pipe','pipe']});
try{
  const url=await new Promise((resolve,reject)=>{let output='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);server.stdout.on('data',b=>{output+=b;const match=output.match(/URL=(http:\/\/\S+)/);if(match){clearTimeout(timer);resolve(match[1]);}});server.on('error',e=>{clearTimeout(timer);reject(e);});server.on('exit',code=>{clearTimeout(timer);reject(Error('server exit '+code));});});
  await test('common parameter requests reach both real CLIs through unchanged web API',async()=>{
    for(const [skill,endpoint,analyze] of [['arm-statics-parametric','statics',statics],['arm-workspace-parametric','workspace',workspace]]){
      const request=JSON.parse(await readFile(new URL(`../../${skill}/assets/examples/design-seven-axis.json`,import.meta.url),'utf8'));
      const response=await fetch(url+'/api/'+endpoint,{method:'POST',headers:{'content-type':'application/jsonc'},body:'// common parameter request\n'+JSON.stringify(request)});
      assert.equal(response.status,200);const body=await response.json();assert.deepEqual(body.result,analyze(request));assert.equal(body.execution.execution_mode,'skill_cli');
    }
  });
  await test('shared browser geometry resolves through compatibility re-export and allowlist',async()=>{
    const entry=new URL('/workspace-kinematics.mjs',url),text=await (await fetch(entry)).text();const relative=text.match(/from '([^']+)'/)[1];
    const response=await fetch(new URL(relative,entry));assert.equal(response.status,200);assert.match(await response.text(),/export function forward/);
    assert.equal((await fetch(url+'/shared/contracts/schemas.mjs')).status,404);
    assert.equal((await fetch(url+'/shared/kinematics/other.mjs')).status,404);
  });
  await test('edited common model flows through all three CLIs into the module table',async()=>{
    const template=JSON.parse(await readFile(new URL('../../arm-kinematics-dynamics/assets/examples/worksheet-dynamics.json',import.meta.url)));
    const model=template.design_model;
    model.name='Edited 1.5 kg design';
    model.payload.mass_kg=1.5;
    delete model.joints[0].components[0].inertia_kg_m2;
    const post=(design_model,prepare_only)=>fetch(url+'/api/module-specs',{
      method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({design_model,prepare_only}),
    });
    const prepared=await post(model,true);
    assert.equal(prepared.status,200);
    const {requests}=await prepared.json();
    for(const request of Object.values(requests)){
      assert.deepEqual(request.design_model,requests.motion.design_model);
      assert.equal(request.design_model.payload.mass_kg,1.5);
    }
    assert.ok(requests.motion.design_model.joints[0].components[0].inertia_kg_m2);
    const response=await post(model,false);
    assert.equal(response.status,200);
    const {specifications,results,execution}=await response.json();
    assert.equal(specifications.model_name,model.name);
    assert.equal(specifications.joints.length,7);
    for(const [skill,result] of Object.entries(results)){
      assert.equal(execution[skill].execution_mode,'skill_cli');
      assert.equal(result.design_model_sha256,specifications.design_model_sha256);
    }
    assert.equal(specifications.workspace.sample_count,results.workspace.summary.point_count);
    assert.equal(specifications.joints[0].holding_effort_abs,results.statics.summary[0].required_output);
    assert.equal(specifications.joints[0].dynamic_peak_abs,results.motion.summary.dynamic_joints[0].effort_peak_abs);
    delete model.joints[0].components;
    assert.equal((await post(model,true)).status,422);
  });
}finally{server.kill('SIGTERM');}
