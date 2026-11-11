import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile,mkdtemp,writeFile,rm} from 'node:fs/promises';
import {spawnSync} from 'node:child_process';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {fileURLToPath} from 'node:url';
import {analyze} from '../lib/analyze.mjs';
import {singularValues,singularity} from '../../../shared/kinematics/singularity.mjs';
const close=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
const identity={xyz_m:[0,0,0],rpy_rad:[0,0,0]},source={source:'analytic test fixture',assumptions:[]};
function fixture(){return {schema_name:'arm-motion-request',schema_version:'1.0',design_model:{schema_name:'arm-design-model',schema_version:'1.0',name:'rod',joints:[{id:'j1',name:'J1',type:'revolute',origin:identity,axis:[0,0,1],limits:[-4,4]}],tip:{xyz_m:[1,0,0],rpy_rad:[0,0,0]},provenance:source},motion_limits:{j1:{velocity_max:1,acceleration_max:2}},trajectory:{kind:'waypoints',start_q:[0],sample_dt_s:.05,timing_policy:'strict',segments:[{id:'move',kind:'move',duration_s:2,q_target:[1]}]},singularity:{task:'position'}};}
const example=async name=>JSON.parse(await readFile(new URL(`../../assets/examples/${name}-motion.json`,import.meta.url),'utf8'));
test('Jacobi SVD resolves known spectra, exact rank loss and ill-conditioned values without Gram squaring',()=>{
  assert.deepEqual(singularValues([[3,0,0],[0,2,0]]),[3,2]);
  const values=singularValues([[1,1],[1,1]]);close(values[0],2);assert.ok(values[1]<1e-12);
  const small=singularValues([[1,0],[0,1e-10]]);close(small[1],1e-10,1e-20);
  const rotated=singularValues([[3/Math.sqrt(2),3/Math.sqrt(2)],[-2/Math.sqrt(2),2/Math.sqrt(2)]]);close(rotated[0],3);close(rotated[1],2);
});
test('quintic analytic endpoints, extrema and strict constraints are independent of coarse output sampling',()=>{
  const input=fixture();input.trajectory.sample_dt_s=1;const r=analyze(input),first=r.samples[0],last=r.samples.at(-1),mid=r.samples.find(s=>s.time_s===1);
  close(first.q[0],0);close(last.q[0],1);close(last.dq[0],0);close(last.ddq[0],0);close(mid.q[0],.5);close(mid.dq[0],.9375);close(r.summary.joint_peaks[0].acceleration_max,10/Math.sqrt(3)/4);
  assert.equal(r.summary.trajectory_feasible,true);assert.equal(r.summary.task_underactuated,true);assert.equal(r.scope.dynamics_computed,false);
  input.motion_limits.j1.acceleration_max=1;const fail=analyze(input);assert.equal(fail.summary.trajectory_feasible,false);assert.ok(fail.events.some(e=>e.quantity==='acceleration_max'&&e.verification==='analytic_segment_extrema'));
});
test('stretch obeys all joint bounds including jerk and preserves requested duration separately',()=>{
  const input=fixture();input.trajectory.timing_policy='stretch';input.trajectory.segments[0].duration_s=.1;input.motion_limits.j1={velocity_max:.2,acceleration_max:.5,jerk_max:1};
  const r=analyze(input);assert.equal(r.summary.trajectory_feasible,true);close(r.trajectory.segments[0].requested_duration_s,.1);assert.ok(r.summary.duration_s>=9.375-1e-10);assert.ok(r.events.some(e=>e.type==='duration_stretched'));
});
test('hold/wait preserve position and exact zero derivatives with unique monotonic timestamps',()=>{
  const input=fixture();input.trajectory.segments.push({id:'hold',kind:'hold',duration_s:1},{id:'wait',kind:'wait',duration_s:.5});const r=analyze(input);
  close(r.summary.duration_s,3.5);assert.equal(new Set(r.samples.map(s=>s.time_s)).size,r.samples.length);
  r.samples.filter(s=>s.time_s>2).forEach(s=>{close(s.q[0],1);close(s.dq[0],0);close(s.ddq[0],0);});
});
test('external q-only records do not invent derivatives; explicit estimation works on nonuniform quadratic data',()=>{
  const input=fixture();input.trajectory={kind:'samples',joint_ids:['j1'],samples:[0,.2,.7,1].map(t=>({time_s:t,q:[t*t]}))};let r=analyze(input);
  assert.equal(r.samples[0].dq,null);assert.equal(r.samples[0].tcp_base.linear_velocity_m_s,null);assert.equal(r.summary.trajectory_feasible,null);
  input.trajectory.derivatives='estimate';r=analyze(input);r.samples.forEach(s=>{close(s.dq[0],2*s.time_s,1e-8);close(s.ddq[0],2,1e-8);});assert.equal(r.summary.constraint_status,'violated');
});
test('TCP offset, world transform and angular/linear accelerations use geometric Jacobian with Jdot',()=>{
  const input=fixture();input.tcp_offset={xyz_m:[1,0,0],rpy_rad:[0,0,0]};input.design_model.base_in_world={xyz_m:[2,3,4],rpy_rad:[0,0,Math.PI/2]};
  input.trajectory={kind:'samples',joint_ids:['j1'],samples:[0,.2,.4].map(t=>({time_s:t,q:[t],dq:[1],ddq:[0]}))};
  const r=analyze(input),s=r.samples[1];close(s.tcp_base.position_m[0],2*Math.cos(.2));close(s.tcp_base.linear_velocity_m_s[1],2*Math.cos(.2));close(s.tcp_base.linear_acceleration_m_s2[0],-2*Math.cos(.2),1e-7);close(s.tcp_base.angular_velocity_rad_s[2],1);close(s.tcp_world.position_m[0],2-2*Math.sin(.2));
  assert.equal(r.summary.trajectory_feasible,null);assert.equal(r.summary.constraint_status,'samples_passed');
});
test('two-link straight posture loses rank; bent posture restores available rank, independently of underactuation',()=>{
  const input=fixture();input.design_model.joints.push({...structuredClone(input.design_model.joints[0]),id:'j2',name:'J2',origin:{xyz_m:[1,0,0],rpy_rad:[0,0,0]}});delete input.motion_limits;
  input.trajectory={kind:'samples',joint_ids:['j2','j1'],samples:[{time_s:0,q:[0,0]},{time_s:1,q:[Math.PI/2,0]}]};const r=analyze(input);
  assert.equal(r.samples[0].singularity.status,'singular');assert.equal(r.samples[0].singularity.condition_number_infinite,true);assert.equal(r.samples[0].singularity.condition_number,null);
  assert.equal(r.samples[1].singularity.effective_rank,2);assert.equal(r.samples[1].singularity.task_underactuated,true);assert.equal(r.samples[1].singularity.status,'regular');assert.ok(r.events.some(e=>e.type==='singularity_risk'));
});
test('position and full-pose tasks differ; characteristic length normalization removes uniform geometric scale',async()=>{
  const input=await example('franka-fr3');const r=analyze(input);const scaled=structuredClone(input);scaled.design_model.joints.forEach(j=>j.origin.xyz_m=j.origin.xyz_m.map(x=>x*10));scaled.design_model.tip.xyz_m=scaled.design_model.tip.xyz_m.map(x=>x*10);scaled.singularity.characteristic_length_m*=10;
  const other=analyze(scaled);r.samples[0].singularity.singular_values.forEach((v,i)=>close(v,other.samples[0].singularity.singular_values[i],1e-8));
  input.singularity.task='position';assert.equal(analyze(input).samples[0].singularity.task_dimension,3);
});
test('both public models run deterministically with no mass/inertia prerequisites',async()=>{
  for(const name of ['worksheet','franka-fr3']){const input=await example(name),snapshot=structuredClone(input),a=analyze(input),b=analyze(input);assert.deepEqual(a,b);assert.deepEqual(input,snapshot);assert.equal(a.joint_names.length,7);assert.ok(a.samples.every(s=>s.q.every(Number.isFinite)));assert.equal(a.summary.trajectory_feasible,true);}
});
test('invalid inputs reject before compute; imported position violations remain diagnostic failures',()=>{
  for(const mutate of [r=>r.trajectory.start_q=[10],r=>r.trajectory.segments[0].duration_s=0,r=>r.trajectory.segments[0].kind='hold',r=>r.singularity.extra=1,r=>r.motion_limits.j1.velocity_max=0,r=>r.trajectory.sample_dt_s=.00001,r=>r.trajectory.segments[0].q_target=[NaN]]){const input=fixture();mutate(input);assert.throws(()=>analyze(input),e=>e.code==='INVALID_INPUT');}
  const input=fixture();input.trajectory={kind:'samples',joint_ids:['j1'],samples:[{time_s:0,q:[0]},{time_s:1,q:[10]}]};assert.equal(analyze(input).summary.trajectory_feasible,false);
  input.trajectory.samples[1].time_s=0;assert.throws(()=>analyze(input),/increase/);
});
test('CLI preserves old successful output on failure and portable bundle runs without web or original repo',async()=>{
  const temp=await mkdtemp(join(tmpdir(),'arm-motion-cli-')),root=fileURLToPath(new URL('../../../',import.meta.url));
  try{
    const bundle=join(temp,'bundle'),pack=spawnSync(process.execPath,[join(root,'shared/scripts/package-skill.mjs'),'arm-kinematics-dynamics',bundle],{cwd:tmpdir(),encoding:'utf8'});assert.equal(pack.status,0,pack.stderr);
    const entry=join(bundle,'arm-kinematics-dynamics/scripts/run.mjs'),out=join(temp,'result.json'),file=join(bundle,'arm-kinematics-dynamics/assets/examples/franka-fr3-motion.json');
    const run=spawnSync(process.execPath,[entry,'--input',file,'--output',out,'--report-dir',join(temp,'report')],{cwd:tmpdir(),encoding:'utf8'});assert.equal(run.status,0,run.stderr);const saved=await readFile(out,'utf8');assert.equal(JSON.parse(saved).summary.sample_count,257);
    await writeFile(join(temp,'bad.json'),'{}');const bad=spawnSync(process.execPath,[entry,'--input',join(temp,'bad.json'),'--output',out],{encoding:'utf8'});assert.notEqual(bad.status,0);assert.equal(JSON.parse(bad.stderr).error.code,'INVALID_INPUT');assert.equal(await readFile(out,'utf8'),saved);assert.ok((await readFile(join(temp,'report/trajectory.csv'),'utf8')).includes('fr3_joint1'));
  }finally{await rm(temp,{recursive:true,force:true});}
});

test('worksheet zero-waypoint demo produces singular intervals without falsely failing joint motion limits',async()=>{
  const input=await example('worksheet');input.trajectory.segments[0].q_target=Array(7).fill(0);const r=analyze(input);
  assert.ok(r.summary.singular_sample_count>0);assert.ok(r.events.some(e=>e.type==='singularity_risk'&&e.status==='singular'));assert.equal(r.summary.trajectory_feasible,true);
});
