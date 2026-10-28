import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {analyze,toStandardInput,reportMarkdown} from '../lib/statics.mjs';

const close=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
// Independent closed-form Rz Ry Rx, used to rotate the physical chain in W.
export function matrix([r,p,y]) {
  const cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];
}
const mv=(R,v)=>R.map(row=>row.reduce((s,x,i)=>s+x*v[i],0));
const installation=rpy=>({gravity_world_m_s2:[0,0,-9.81],base_orientation_in_world:{rpy_rad:rpy}});
const rod=()=>({schema_version:'1.0',input_mode:'simple',simple:{name:'rod',segments:[{name:'J1',axis:[0,1,0],length_m:1,limits_rad:[-Math.PI,Math.PI],link_mass_kg:1}]},payload:{mass_kg:0,com_in_tip_m:[0,0,0]},poses:[{name:'zero',q:{J1:0}}],design_factor:1});

test('installation: upright, inverted and side-mounted rod analytical loads',()=>{
  for(const [rpy,expected] of [[[0,0,0],-4.905],[[Math.PI,0,0],4.905],[[0,Math.PI/2,0],0]]){
    const r=analyze({...rod(),installation:installation(rpy)});close(r.poses[0].holding_load[0],expected);
    close(Math.hypot(...r.normalized_input.gravity_m_s2),9.81);
  }
});

test('installation: arbitrary world rotation equals base gravity transform, including installed root and local loads',async()=>{
  const example=JSON.parse(await readFile(new URL('../../assets/examples/project-right-arm.json',import.meta.url),'utf8'));
  const source=toStandardInput(example);delete source.search;
  source.model.joints[0].origin={xyz_m:[.1,-.2,.3],rpy_rad:[.2,.1,-.4]};
  source.attachments=[{name:'camera',frame:source.model.joints[2].name,mass_kg:.3,com_m:[.02,-.04,.01]}];
  source.wrenches=[{name:'force',frame:'tip',point_m:[.1,.02,.03],force_N:[3,-4,5],moment_Nm:[.2,.3,-.1]}];
  for(const angles of [[0,0,0],[Math.PI,0,0],[0,Math.PI/2,0],[.37,-.81,1.17]]){
    const base=structuredClone(source);delete base.gravity_m_s2;base.installation=installation(angles);
    const world=structuredClone(source);world.gravity_m_s2=[0,0,-9.81];
    world.model.joints.unshift({name:'world_mount',type:'fixed',origin:{xyz_m:[0,0,0],rpy_rad:angles},components:[]});
    const a=analyze(base),b=analyze(world),R=matrix(angles);
    a.poses.forEach((pose,k)=>{
      pose.holding_load.forEach((v,j)=>close(v,b.poses[k].holding_load[j]));
      mv(R,pose.tip_pose_in_base.position_m).forEach((v,j)=>close(v,b.poses[k].tip_pose_in_base.position_m[j]));
      for(const category of Object.keys(pose.components))pose.components[category].forEach((v,j)=>close(v,b.poses[k].components[category][j]));
    });
    const round=toStandardInput(base);assert.ok(round.installation);assert.equal('gravity_m_s2' in round,false);
    assert.deepEqual(analyze(round).summary,a.summary);assert.deepEqual(analyze(base),a);
    assert.match(reportMarkdown(a),/世界重力及基座安装姿态/);
  }
});

test('installation: prismatic force transforms and search extrema re-evaluate',async()=>{
  const p=JSON.parse(await readFile(new URL('../../assets/examples/prismatic.json',import.meta.url),'utf8'));
  delete p.gravity_m_s2;p.installation=installation([Math.PI,0,0]);
  const out=analyze(p),old=structuredClone(p);delete old.installation;old.gravity_m_s2=[0,0,9.81];
  out.poses[0].holding_load.forEach((v,i)=>close(v,analyze(old).poses[0].holding_load[i]));
  const r={...rod(),installation:installation([.2,.8,-.3]),search:{samples:100,seed:42}};
  const a=analyze(r),one=structuredClone(r);delete one.search;one.poses=[{name:'critical',q:a.summary[0].critical_pose.q}];
  close(Math.abs(analyze(one).poses[0].holding_load[0]),a.summary[0].max_abs_checked);
});

test('installation: legacy input physics unchanged and invalid/ambiguous frames rejected',()=>{
  const direct={...rod(),gravity_m_s2:[0,0,-9.81]},mounted={...rod(),installation:installation([0,0,0])};
  assert.deepEqual(analyze(direct).poses,analyze(mounted).poses);
  for(const request of [rod(),{...direct,installation:installation([0,0,0])},{...rod(),installation:{}},{...rod(),installation:{...installation([0,0,0]),unknown:1}},{...rod(),installation:installation([0,NaN,0])},{...rod(),installation:installation([0,0])},{...rod(),installation:{gravity_world_m_s2:[0,0,-9.81],base_orientation_in_world:{rpy_rad:[0,0,0],yaw:1}}}])assert.throws(()=>analyze(request),e=>e.code==='INVALID_INPUT');
  const zero={...rod(),installation:{...installation([.3,.4,.5]),gravity_world_m_s2:[0,0,0]}};
  close(analyze(zero).summary[0].max_abs_checked,0);
});
