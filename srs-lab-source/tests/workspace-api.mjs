import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile,mkdtemp,writeFile,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {pathToFileURL,fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {createRequire} from 'node:module';
import {analyze} from '../../arm-workspace-parametric/scripts/lib/workspace.mjs';
import {forward} from '../../arm-workspace-parametric/scripts/lib/kinematics.mjs';
const E=createRequire(import.meta.url)('../dist/engine.js');
const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-9,`${a} != ${b}`);
const root=new URL('../../arm-workspace-parametric/',import.meta.url);
const dir=await mkdtemp(join(tmpdir(),'workspace-client-test-'));
for(const name of ['kinematics','exports','input-text'])await writeFile(join(dir,`workspace-${name}.mjs`),await readFile(new URL(`scripts/lib/${name}.mjs`,root)));
await writeFile(join(dir,'workspace-client.mjs'),await readFile(new URL('../dist/workspace-client.mjs',import.meta.url)));
const client=await import(pathToFileURL(join(dir,'workspace-client.mjs')));

try{
  await test('workspace client maps all web templates/custom axes and installation to identical FK; replays q',()=>{
    for(const variant of E.configurations()){
      const c=E.defaults(variant.id);c.installation={gravity_world_m_s2:[0,0,-9.8],base_orientation_in_world:{rpy_rad:[.3,-.4,.8]}};c.samples=100;
      c.jointAxes=E.model(c).axes.map(a=>a.map(x=>-x));
      const input=client.fromWebConfig(c,E.model(c),E.installationInfo(c)),result=analyze(input);
      for(let i=0;i<result.points.length;i+=7){const p=result.points[i],f=E.fk(c,result.joint_names.map(j=>p.q[j]));f.p.forEach((v,k)=>close(v,p.position_base_m[k]));const preview=client.previewPose(result,i);preview.position_base_m.forEach((v,k)=>close(v,p.position_base_m[k]));preview.position_world_m.forEach((v,k)=>close(v,p.position_world_m[k]));}
      assert.equal(client.parseInput('// comment\n'+client.formatInput(input)).mode,'joint_sampling');
    }
  });
  await test('workspace client replays position-only fallback when all requested orientations fail',()=>{
    const c={...E.defaults('WORKSHEET-7R'),samples:100},input=client.fromWebConfig(c,E.model(c),E.installationInfo(c));
    const position=forward(input.model,[0,0,0,0,0,0,0]).position_m;
    delete input.sampling;input.mode='pose_reachability';input.reachability={points_m:[position],orientations_rpy_rad:[[Math.PI,0,0]],ik:{attempts:1,max_iterations:1}};
    const result=analyze(input);assert.equal(result.points[0].spatial_status,'reachable');assert.equal(result.points[0].solutions.length,0);
    const preview=client.previewPose(result,0);preview.position_base_m.forEach((v,k)=>close(v,position[k]));
    assert.equal(client.pointVisible(result.points[0],'spatial'),true);
    assert.equal(client.pointVisible(result.points[0],'sampled_dexterous'),false);
  });
  const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('URL=http://127.0.0.1:'+process.env.PORT);});});`;
  const env={...process.env};delete env.STATICS_SKILL_DIR;delete env.WORKSPACE_SKILL_DIR;
  const server=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env,stdio:['ignore','pipe','pipe']});
  try{
    const url=await new Promise((resolve,reject)=>{let text='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);server.stdout.on('data',b=>{text+=b;const m=text.match(/URL=(http:\/\/\S+)/);if(m){clearTimeout(timer);resolve(m[1]);}});server.once('error',e=>{clearTimeout(timer);reject(e);});});
    await test('workspace server health/assets work independently of statics env configuration',async()=>{
      assert.equal((await fetch(url+'/api/workspace/health')).status,200);
      assert.equal((await fetch(url+'/api/statics/health')).status,503);
      for(const asset of ['/workspace-client.mjs','/workspace-scene.js','/workspace-kinematics.mjs','/workspace-exports.mjs','/workspace-input-text.mjs','/workspace-example.jsonc','/workspace-reachability.jsonc','/workspace-classification.jsonc','/workspace-dexbot-right.json','/workspace-dexbot-left.json'])assert.equal((await fetch(url+asset)).status,200,asset);
      assert.equal((await fetch(url+'/workspace-../../server/index.mjs')).status,404);
    });
    await test('workspace real CLI result and provenance equal direct analysis for FK and IK',async()=>{
      const c={...E.defaults('WORKSHEET-7R'),samples:100};
      const input=client.fromWebConfig(c,E.model(c),E.installationInfo(c));
      for(const mode of ['joint_sampling','pose_reachability']){
        const request=structuredClone(input);request.mode=mode;
        if(mode==='pose_reachability'){delete request.sampling;request.reachability={points_m:[forward(request.model,[0,0,0,0,0,0,0]).position_m,[10,0,0]],ik:{attempts:2,max_iterations:30}};}
        const response=await fetch(url+'/api/workspace',{method:'POST',headers:{'content-type':'application/jsonc'},body:'/* test */'+JSON.stringify(request)});assert.equal(response.status,200);
        const data=await response.json();assert.deepEqual(data.result,analyze(request));assert.equal(data.execution.execution_mode,'skill_cli');assert.equal(data.execution.skill_name,'arm-workspace-parametric');assert.ok(data.execution.raw_input_sha256);assert.ok(data.execution.request_id);
      }
    });
    await test('workspace errors do not return fallback point clouds',async()=>{
      assert.equal((await fetch(url+'/api/workspace',{method:'POST',body:'{}'})).status,415);
      for(const body of ['{}','/* bad */ {']){const response=await fetch(url+'/api/workspace',{method:'POST',headers:{'content-type':'application/jsonc'},body});assert.equal(response.status,502);const data=await response.json();assert.equal(data.ok,false);assert.equal(data.result,undefined);}
    });
  }finally{server.kill();await once(server,'exit');}
}finally{await rm(dir,{recursive:true,force:true});}
