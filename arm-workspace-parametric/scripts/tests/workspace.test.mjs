import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile,mkdtemp,rm,readdir,writeFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';
import {analyze,normalizeInput,spherePoints} from '../lib/workspace.mjs';
import {forward,rpy,mv,add,orientationError,jacobian} from '../lib/kinematics.mjs';
import {parseRequestText} from '../lib/input-text.mjs';
import {projectionSVG,pointsCSV} from '../lib/exports.mjs';
const root=fileURLToPath(new URL('../../',import.meta.url));
const example=async name=>parseRequestText(await readFile(join(root,'assets/examples',name),'utf8'));
const close=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
const one=()=>({schema_version:'1.0',mode:'joint_sampling',model:{name:'rod',joints:[{name:'J1',type:'revolute',origin:{xyz_m:[0,0,0],rpy_rad:[0,0,0]},axis:[0,0,1],limits:[-Math.PI,Math.PI]}],tip:{xyz_m:[1,0,0],rpy_rad:[0,0,0]}},sampling:{method:'random',samples:100,seed:42}});
function ikInput(model,points,orientations){return {schema_version:'1.0',mode:'pose_reachability',model,reachability:{points_m:points,...(orientations?{orientations_rpy_rad:orientations}:{}),seed:42,ik:{attempts:6,max_iterations:200,position_tolerance_m:1e-5,orientation_tolerance_rad:1e-4,damping:.01}}};}

test('FK points preserve circle radius, joint limits, replay q and deterministic output',()=>{
  const input=one(),snapshot=structuredClone(input),a=analyze(input);assert.deepEqual(input,snapshot);assert.equal(JSON.stringify(a),JSON.stringify(analyze(input)));
  for(const p of a.points){close(Math.hypot(...p.position_base_m),1);close(p.position_base_m[2],0);assert.ok(p.q.J1>=-Math.PI&&p.q.J1<=Math.PI);assert.deepEqual(forward(a.normalized_input.model,[p.q.J1]).position_m,p.position_base_m);}
  assert.equal(a.points[0].q.J1,0);assert.equal(a.scope.orientation_checked,false);assert.equal(a.summary.sampled_orientation_success_rate,null);
});

test('grid includes actual limit endpoints and named fixed joints without repeated grids',()=>{
  const input=one();input.sampling={method:'grid',steps_per_joint:5};const a=analyze(input);assert.equal(a.points.length,5);close(a.points[0].q.J1,-Math.PI);close(a.points.at(-1).q.J1,Math.PI);
  input.sampling.fixed_joints={J1:.3};const b=analyze(input);assert.equal(b.points.length,1);close(b.points[0].q.J1,.3);
});

test('Dexbot left/right URDF snapshots match 20 independent Python 4x4 FK references',async()=>{
  const references=JSON.parse(await readFile(join(root,'scripts/tests/fixtures/dexbot-fk-reference.json'),'utf8'));
  const models=Object.fromEntries(await Promise.all(['left','right'].map(async side=>[side,(await example(`dexbot-${side}-arm.json`)).model])));
  for(const ref of references.poses){const actual=forward(models[ref.arm],ref.q);ref.position_m.forEach((v,k)=>close(v,actual.position_m[k]));ref.rotation.forEach((row,i)=>row.forEach((v,j)=>close(v,actual.rotation[i][j])));}
  close(forward(models.right,Array(7).fill(0)).position_m[1],-.17);close(forward(models.left,Array(7).fill(0)).position_m[2],-.6);
});

test('installation preserves base points and applies the declared world transform exactly once',async()=>{
  const input=await example('worksheet-workspace.jsonc');input.sampling.samples=30;const original=analyze(input);
  input.base_in_world={xyz_m:[.1,-.2,.3],rpy_rad:[.3,-.7,1.1]};const out=analyze(input),R=rpy(input.base_in_world.rpy_rad);
  out.points.forEach((p,i)=>{assert.deepEqual(p.position_base_m,original.points[i].position_base_m);add(input.base_in_world.xyz_m,mv(R,p.position_base_m)).forEach((v,k)=>close(v,p.position_world_m[k]));});
});

test('analytic Jacobian agrees with finite differences for arbitrary revolute/prismatic/fixed installations',()=>{
  const input=one();input.model.joints.push({name:'P',type:'prismatic',origin:{xyz_m:[.1,.2,.3],rpy_rad:[.2,-.3,.4]},axis:[1,2,3],limits:[-.5,.5]},{name:'fixed',type:'fixed',origin:{xyz_m:[.2,.1,-.1],rpy_rad:[.1,.2,.3]}});
  const n=normalizeInput(input),q=[.3,.2],f=forward(n.model,q),J=jacobian(f),h=1e-6;
  q.forEach((_,j)=>{const a=q.slice(),b=q.slice();a[j]+=h;b[j]-=h;const A=forward(n.model,a),B=forward(n.model,b);A.position_m.forEach((v,k)=>close((v-B.position_m[k])/(2*h),J[j][k],1e-8));});
});

test('SO(3) error handles 180 degrees without a false zero error',()=>{close(Math.hypot(...orientationError(rpy([Math.PI,0,0]),rpy([0,0,0]))),Math.PI);close(Math.hypot(...orientationError(rpy([0,0,Math.PI]),rpy([0,0,0]))),Math.PI);});

test('IK verifies both position and orientation, stores bounded replayable success solutions',()=>{
  const input=ikInput(one().model,[[Math.cos(.5),Math.sin(.5),0]],[[0,0,.5]]),out=analyze(input);
  assert.equal(out.summary.ik_success_count,1);const s=out.points[0].solutions[0],f=forward(out.normalized_input.model,[s.q.J1]);close(f.position_m[0],Math.cos(.5),1e-5);assert.ok(s.orientation_error_rad<=1e-4);assert.equal(out.scope.orientation_checked,true);
  input.reachability.orientations_rpy_rad=[[Math.PI,0,0]];assert.equal(analyze(input).summary.ik_success_count,0);
});

test('position-only prismatic IK succeeds while a far target stays unresolved with null bounds',()=>{
  const input=one();input.model.joints[0].type='prismatic';input.model.joints[0].axis=[1,0,0];input.model.joints[0].limits=[0,1];input.model.tip.xyz_m=[0,0,0];
  const request=ikInput(input.model,[[.4,0,0],[10,0,0]]),out=analyze(request);assert.equal(out.summary.ik_success_count,1);assert.equal(out.points[1].q,null);assert.equal(out.scope.orientation_checked,false);close(out.points[0].q.J1,.4,1e-5);
  request.reachability.points_m=[[10,0,0]];const failed=analyze(request);assert.equal(failed.summary.bounds_base_m,null);assert.equal(failed.summary.sampled_orientation_success_rate,0);assert.ok(projectionSVG(failed).includes('<svg'));assert.equal(failed.summary.sampled_voxel_volume_m3,0);
});

test('seven-axis worksheet 6D IK recovers a verifiable pose distinct from its original redundant solution',async()=>{
  const input=await example('worksheet-workspace.jsonc'),f=forward(input.model,[.4,-.6,.3,.8,-.4,.2,.1]),R=f.rotation;
  const angles=[Math.atan2(R[2][1],R[2][2]),Math.asin(-R[2][0]),Math.atan2(R[1][0],R[0][0])];
  const request=ikInput(input.model,[f.position_m],[angles]);request.reachability.ik.attempts=8;
  const result=analyze(request);assert.equal(result.summary.ik_success_count,1);
  const solution=result.points[0].solutions[0],pose=forward(input.model,result.joint_names.map(j=>solution.q[j]));
  f.position_m.forEach((v,k)=>close(v,pose.position_m[k],1e-5));assert.ok(Math.hypot(...orientationError(f.rotation,pose.rotation))<1e-4);
});

test('sphere sampling deduplicates center/poles and sections use declared total thickness',()=>{
  const sphere={center_m:[.1,.2,.3],radius_m:1,radial_steps:3,theta_steps:8,phi_steps:5},points=spherePoints(sphere);
  assert.equal(points.filter(p=>p.every((v,k)=>v===sphere.center_m[k])).length,1);assert.equal(new Set(points.map(p=>p.map(v=>v.toFixed(9)).join(','))).size,points.length);
  points.forEach(p=>assert.ok(Math.hypot(...p.map((v,k)=>v-sphere.center_m[k]))<=1+1e-12));
  const input=one();input.sections={center_m:[0,0,0],thickness_m:.02,voxel_size_m:.1};const out=analyze(input);assert.equal(out.sections.xy.length,100);assert.equal(out.summary.sampled_voxel_volume_m3,out.summary.occupied_voxels*.1**3);assert.ok(pointsCSV(out).includes('q_json'));
});

test('reject malformed inputs, unsafe budgets, mixed modes and unknown fields before compute',()=>{
  const variants=[r=>r.model.joints[0].axis=[0,0,0],r=>r.model.joints[0].limits=[1,0],r=>r.model.joints[0].origin.rpy_rad=[0,NaN,0],r=>r.sampling.seed=-1,r=>r.sampling.samples=50001,r=>r.sampling.samples=true,r=>r.extra=1,r=>r.reachability={},r=>r.sampling.fixed_joints={missing:0},r=>r.sampling={method:'grid',steps_per_joint:100,samples:10},r=>r.model.joints.push(structuredClone(r.model.joints[0]))];
  for(const mutate of variants){const input=one();mutate(input);assert.throws(()=>analyze(input),e=>e.code==='INVALID_INPUT');}
  const input=one();for(let i=2;i<8;i++)input.model.joints.push({...structuredClone(input.model.joints[0]),name:'J'+i});input.sampling={method:'grid',steps_per_joint:10};assert.throws(()=>analyze(input),/budget/);
  const request=ikInput(input.model,Array.from({length:2000},()=>[0,0,0]),Array.from({length:64},()=>[0,0,0]));assert.throws(()=>analyze(request),/budget/);
});

test('CLI JSONC exports deterministic replayable data and preserves existing result on failure',async()=>{
  const dir=await mkdtemp(join(tmpdir(),'workspace-cli-test-'));
  try{const file=join(root,'assets/examples/worksheet-workspace.jsonc'),output=join(dir,'workspace_result.json'),run=()=>spawnSync(process.execPath,[join(root,'scripts/run.mjs'),'--input',file,'--output',output,'--report-dir',dir],{encoding:'utf8'});
    const first=run();assert.equal(first.status,0,first.stderr);const text=await readFile(output,'utf8'),data=JSON.parse(text);assert.deepEqual(data,analyze(await example('worksheet-workspace.jsonc')));assert.equal(run().status,0);assert.equal(await readFile(output,'utf8'),text);
    assert.equal((await readdir(dir)).filter(f=>f.endsWith('.svg')).length,6);
    const bad=join(dir,'bad.json');await writeFile(bad,'{}');assert.notEqual(spawnSync(process.execPath,[join(root,'scripts/run.mjs'),'--input',bad,'--output',output]).status,0);assert.equal(await readFile(output,'utf8'),text);
  }finally{await rm(dir,{recursive:true,force:true});}
});
