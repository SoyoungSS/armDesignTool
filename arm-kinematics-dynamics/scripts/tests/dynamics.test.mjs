import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {analyze} from '../lib/analyze.mjs';
import {prepareDynamics,inverseDynamics,summarizeDynamics} from '../lib/dynamics.mjs';
import {analyze as statics} from '../../../arm-statics-parametric/scripts/lib/statics.mjs';

const source={source:'analytic test',assumptions:[]};
const pose=(xyz=[0,0,0])=>({xyz_m:xyz,rpy_rad:[0,0,0]});
const inertia=value=>[[value,0,0],[0,value,0],[0,0,value]];
const body=(id,x,mass=2)=>({
  id,name:id,type:'revolute',origin:pose([x,0,0]),axis:[0,0,1],limits:[-4,4],
  components:[{name:'link',mass_kg:mass,com_m:[.5,0,0],inertia_kg_m2:inertia(.2),provenance:source}],
});
const model=()=>({
  schema_name:'arm-design-model',schema_version:'1.0',name:'analytic planar',
  joints:[body('a',0),body('b',1)],tip:pose([1,0,0]),
  gravity_world_m_s2:[0,-9.81,0],provenance:source,
});
const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-9,JSON.stringify({actual:a,expected:b}));

test('two-link inverse dynamics matches closed-form mass, Coriolis and gravity terms',()=>{
  const m=model(),[q1,q2]=[.3,-.8],[v1,v2]=[.7,-.4],[a1,a2]=[.5,.2];
  const loads=inverseDynamics(prepareDynamics(m,{}),[q1,q2],[v1,v2],[a1,a2]);
  const M11=3.4+2*Math.cos(q2),M12=.7+Math.cos(q2),M22=.7,h=Math.sin(q2);
  const G2=9.81*Math.cos(q1+q2),G1=3*9.81*Math.cos(q1)+G2;
  close(loads[0].effort,M11*a1+M12*a2-h*(2*v1*v2+v2*v2)+G1);
  close(loads[1].effort,M12*a1+M22*a2+h*v1*v1+G2);
});

test('static limit matches existing statics with mounting, fixed bodies and offset payload',()=>{
  const m=model();
  m.base_in_world={xyz_m:[1,2,3],rpy_rad:[.5,-.3,.8]};
  m.gravity_world_m_s2=[0,0,-9.81];
  m.joints.splice(1,0,{id:'fixed',name:'fixed',type:'fixed',origin:{xyz_m:[.1,.2,.1],rpy_rad:[.1,.2,.3]},components:[{
    name:'adapter',mass_kg:1,com_m:[.2,0,.1],inertia_kg_m2:inertia(.1),provenance:source,
  }]});
  m.payload={mass_kg:3,com_in_tip_m:[.1,.2,.3],provenance:source};
  const q=[.3,-.8],actual=inverseDynamics(prepareDynamics(m,{payload_inertia_kg_m2:inertia(.1)}),q,[0,0],[0,0]);
  const result=statics({schema_name:'arm-analysis-request',schema_version:'1.0',design_model:m,
    options:{poses:[{name:'check',q:{a:q[0],b:q[1]}}]}});
  actual.forEach((load,i)=>close(load.effort,result.poses[0].holding_load[i]));
});

test('rotating prismatic body includes centripetal and Coriolis acceleration',()=>{
  const m=model();
  m.gravity_world_m_s2=[0,0,0];
  m.joints[0].components=[];
  m.joints[1]={id:'slide',name:'slide',type:'prismatic',origin:pose(),axis:[1,0,0],limits:[0,4],
    components:[{name:'mass',mass_kg:2,com_m:[0,0,0],inertia_kg_m2:inertia(.2),provenance:source}]};
  const loads=inverseDynamics(prepareDynamics(m,{}),[.3,1.2],[.7,.4],[.5,.6]);
  close(loads[1].effort,2*(.6-1.2*.7**2));
  close(loads[0].effort,(.2+2*1.2**2)*.5+2*2*1.2*.4*.7);
  close(loads[1].radial_force_N,2*(1.2*.5+2*.7*.4));
});

test('external force sign, application at physical tip, and payload inertia',()=>{
  const m=model();
  m.joints=[body('a',0,0)];
  m.gravity_world_m_s2=[0,0,0];
  m.joints[0].components=[];
  const d={external_wrench_world:{force_N:[0,3,0],moment_Nm:[0,0,2]}};
  close(inverseDynamics(prepareDynamics(m,d),[0],[0],[0])[0].effort,-5);
  m.payload={mass_kg:2,com_in_tip_m:[.5,0,0],provenance:source};
  assert.throws(()=>prepareDynamics(m,{}),/payload.*inertia/);
  close(inverseDynamics(prepareDynamics(m,{payload_inertia_kg_m2:inertia(.3)}),[0],[0],[2])[0].effort,9.6);
});

test('time weighted RMS integrates squared loads using nonuniform sample intervals',()=>{
  const samples=[{time_s:0,q:[0],dq:[0],dynamic_loads:[{effort:0,axial_force_N:0,radial_force_N:0,bending_moment_Nm:0}]},
    {time_s:1,q:[1],dq:[1],dynamic_loads:[{effort:2,axial_force_N:0,radial_force_N:0,bending_moment_Nm:0}]},
    {time_s:4,q:[2],dq:[0],dynamic_loads:[{effort:0,axial_force_N:0,radial_force_N:0,bending_moment_Nm:0}]}];
  const [s]=summarizeDynamics(samples,[{id:'a',type:'revolute'}]);
  close(s.effort_rms,Math.sqrt(2));
  close(s.peak_time_s,1);
});

test('analysis preserves geometry-only use and rejects missing dynamics inputs',async()=>{
  const input=JSON.parse(await readFile(new URL('../../assets/examples/worksheet-motion.json',import.meta.url)));
  assert.equal(analyze(input).scope.dynamics_computed,false);
  input.dynamics={enabled:true};
  assert.throws(()=>analyze(input),/inertia/);
  input.design_model=model();
  delete input.motion_limits;
  input.trajectory={kind:'samples',joint_ids:['a','b'],samples:[{time_s:0,q:[0,0]},{time_s:1,q:[0,0]}]};
  assert.throws(()=>analyze(input),/requires dq and ddq/);
  delete input.motion_limits;
  input.trajectory.samples.forEach(s=>Object.assign(s,{dq:[0,0],ddq:[0,0]}));
  const r=analyze(input);
  assert.equal(r.scope.dynamics_computed,true);
  assert.equal(r.summary.dynamic_joints.length,2);
  close(r.summary.dynamic_joints[0].effort_rms,39.24);
  const baseline=r.samples[0].effort;
  input.tcp_offset={xyz_m:[10,0,0],rpy_rad:[0,0,1]};
  assert.deepEqual(analyze(input).samples[0].effort,baseline);
});

test('spatial chain mass symmetry and kinetic energy balance',()=>{
  const m=model();
  m.gravity_world_m_s2=[0,0,0];
  m.joints.push(body('c',.6));
  m.joints[0].axis=[1,2,3];
  m.joints[1].axis=[0,1,0];
  m.joints[1].origin.rpy_rad=[.2,-.5,.3];
  m.joints[2].axis=[1,0,0];
  m.joints[2].origin.rpy_rad=[-.3,.4,.1];
  const prepared=prepareDynamics(m,{}),q=[.2,-.6,.9],v=[.8,.3,-.4],z=[0,0,0];
  const mass=position=>[0,1,2].map(j=>inverseDynamics(prepared,position,z,z.map((_,i)=>i===j?1:0)).map(load=>load.effort));
  const M=mass(q);
  for(let i=0;i<3;i++)for(let j=0;j<3;j++)close(M[i][j],M[j][i]);
  const h=1e-5,A=mass(q.map((value,i)=>value+h*v[i])),B=mass(q.map((value,i)=>value-h*v[i]));
  let energyRate=0;
  for(let i=0;i<3;i++)for(let j=0;j<3;j++)energyRate+=.5*v[i]*v[j]*(A[i][j]-B[i][j])/(2*h);
  const power=inverseDynamics(prepared,q,v,z).reduce((sum,load,i)=>sum+load.effort*v[i],0);
  close(power,energyRate);
});
