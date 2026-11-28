import test from 'node:test';
import assert from 'node:assert/strict';
import {analyze} from '../lib/workspace.mjs';
import {analyzeDexterity} from '../lib/dexterity.mjs';
import {forward} from '../lib/kinematics.mjs';
import {voxelColor,voxelSection,voxelSectionSVG,pointsCSV,reportMarkdown} from '../lib/exports.mjs';

const pose={xyz_m:[0,0,0],rpy_rad:[0,0,0]};
const basis=[[1,0,0],[0,1,0],[0,0,1]];
function input(){
  const joints=basis.map((axis,i)=>({name:`p${i}`,type:'prismatic',origin:pose,axis,limits:[-.3,.3]}));
  for(const [i,axis] of basis.slice().reverse().entries())joints.push({name:`r${i}`,type:'revolute',origin:pose,axis,limits:[-Math.PI,Math.PI]});
  return {schema_version:'1.0',mode:'dexterity_voxels',model:{name:'position and wrist fixture',joints,tip:pose},
    base_in_world:{xyz_m:[1,2,3],rpy_rad:[0,Math.PI/2,0]},
    dexterity:{voxel_size_m:.1,orientations_per_voxel:4,bounds_base_m:{min:[-.1,-.1,-.1],max:[.1,.1,.1]}}};
}

test('dexterity checks uniform shared orientations at grid centers and keeps replayable IK',()=>{
  const request=input(),result=analyze(request);
  assert.deepEqual(analyze(request),result);
  assert.deepEqual(analyze({...request,dexterity:{...request.dexterity,time_budget_s:.01}}).points,result.points);
  assert.equal(result.dexterity.completed_voxels,8);
  assert.equal(result.summary.numerical_ik_solve_count,40);
  assert.equal(result.summary.position_check_count,8);
  assert.equal(result.summary.ik_query_count,32);
  assert.equal(result.summary.sampled_orientation_success_rate,result.summary.ik_success_count/32);
  const quaternions=result.dexterity.orientations_wxyz;
  assert.equal(new Set(quaternions.map(JSON.stringify)).size,4);
  assert.ok(quaternions.every(q=>Math.abs(Math.hypot(...q)-1)<1e-12));
  assert.ok(result.points.some(p=>p.reachability>0));
  for(const point of result.points){
    assert.deepEqual(point.position_base_m,point.voxel_index.map(i=>(i+.5)*.1));
    assert.equal(point.reachability,point.orientation_successes/4);
    if(point.q){
      const f=forward(result.normalized_input.model,result.joint_names.map(name=>point.q[name]));
      assert.ok(Math.hypot(...f.position_m.map((v,i)=>v-point.position_base_m[i]))<=.001);
    }
  }
  for(const plane of ['xy','yz','xz'])for(const layer of [-1,0]){
    const section=voxelSection(result,plane,layer);
    assert.equal(section.points.length,4);
    assert.ok(section.points.every(p=>p.voxel_index[section.axes[2]]===layer));
    assert.match(voxelSectionSVG(section),new RegExp(`${plane.toUpperCase()} 剖面`));
  }
});

test('voxel position-only witnesses survive failed orientation solves and are exported separately',()=>{
  const request=input();request.model.joints=request.model.joints.slice(0,3);
  const result=analyze(request);
  assert.equal(result.summary.spatial_reachable_point_count,8);
  assert.equal(result.summary.ik_success_count,0);
  assert.equal(result.summary.workspace_counts.spatial_reachable,8);
  assert.equal(result.summary.unresolved_point_count,0);
  for(const p of result.points){
    assert.equal(p.reachability,0);assert.equal(p.category,'spatial_reachable');assert.ok(p.position_solution);
    const fk=forward(result.normalized_input.model,result.joint_names.map(name=>p.q[name]));
    assert.ok(Math.hypot(...fk.position_m.map((x,i)=>x-p.position_base_m[i]))<=.001);
  }
  assert.match(pointsCSV(result),/position_solution_json/);
  assert.match(reportMarkdown(result),/另有 8 次位置检查/);
  assert.notEqual(voxelColor(result.points[0]),voxelColor({reachability:0,category:'unresolved'}));
});

test('a successful orientation also witnesses position when the independent position solve fails',()=>{
  const scan=analyzeDexterity({options:{voxel_size_m:.1,orientations_per_voxel:2},cells:[{index:[0,0,0],position:[.05,.05,.05]}],world:p=>p,orientationRandom:()=>.25,
    solve:(p,rotation)=>rotation?{success:true,q:{j:1}}:{success:false}});
  assert.equal(scan.points[0].spatial_status,'reachable');assert.ok(scan.points[0].position_solution);assert.equal(scan.calls,3);
});

test('dexterity completes all planned checks even after a legacy time budget elapses',()=>{
  const scan=analyzeDexterity({options:{voxel_size_m:.1,orientations_per_voxel:2,time_budget_s:.01},
    cells:[{index:[0,0,0],position:[.05,.05,.05]},{index:[1,0,0],position:[.15,.05,.05]}],
    world:p=>p,orientationRandom:()=>.25,solve:()=>{
      Atomics.wait(new Int32Array(new SharedArrayBuffer(4)),0,0,25);
      return {success:true,q:{joint:0}};
    }});
  assert.equal(scan.detail.completed,true);
  assert.equal(scan.detail.time_budget_s,null);
  assert.equal(scan.calls,6);
  assert.ok(scan.points.every(p=>p.orientation_tested===2&&p.reachability===1));
  assert.notEqual(voxelColor({reachability:null}),voxelColor({reachability:0}));
});
