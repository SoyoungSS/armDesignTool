import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {analyze,conservativeReachBound} from '../lib/workspace.mjs';
import {forward} from '../lib/kinematics.mjs';
import {pointColor,pointVisible,projectionSVG,pointsCSV,reportMarkdown} from '../lib/exports.mjs';
import {parseRequestText} from '../lib/input-text.mjs';

// Analytic planar test rig: x/y slides + yaw, with a 1 m tool offset.
// Its position workspace is larger than its two-orientation dexterous workspace.
function request(){
  const origin={xyz_m:[0,0,0],rpy_rad:[0,0,0]};
  return {schema_version:'1.0',mode:'pose_reachability',model:{name:'planar classification rig',joints:[
    {name:'PX',type:'prismatic',origin,axis:[1,0,0],limits:[0,2]},
    {name:'PY',type:'prismatic',origin,axis:[0,1,0],limits:[-.6,.6]},
    {name:'RZ',type:'revolute',origin,axis:[0,0,1],limits:[-Math.PI,Math.PI]}
  ],tip:{xyz_m:[1,0,0],rpy_rad:[0,0,0]}},reachability:{points_m:[[1,0,0],[2,0,0],[.5,1,0],[10,0,0],[1,0,1]],orientations_rpy_rad:[[0,0,0],[0,0,Math.PI]],seed:42,ik:{attempts:12,max_iterations:300,position_tolerance_m:1e-5,orientation_tolerance_rad:1e-4,damping:.01}}};
}

test('separates spatial, sampled dexterous, partial, proven unreachable and unresolved points',()=>{
  const result=analyze(request());
  assert.deepEqual(result.points.map(p=>p.category),['sampled_dexterous','partially_dexterous','spatial_reachable','unreachable','unresolved']);
  assert.equal(result.summary.spatial_reachable_point_count,3);
  assert.equal(result.summary.sampled_dexterous_point_count,1);
  assert.equal(result.summary.proven_unreachable_point_count,1);
  assert.equal(result.summary.unresolved_point_count,1);
  assert.equal(result.points[2].orientation_successes,0);
  assert.ok(result.points[2].position_solution.q);
  assert.equal(result.points[3].unreachable_reason,'outside_conservative_reach_bound');
  assert.equal(result.points[4].unreachable_reason,null);
  assert.equal(result.scope.dexterous_workspace_proven,false);
  for(const p of result.points.filter(p=>p.q)){
    const f=forward(result.normalized_input.model,result.joint_names.map(j=>p.q[j]));
    assert.ok(Math.hypot(...f.position_m.map((v,k)=>v-p.position_base_m[k]))<=1e-5);
  }
});

test('one orientation or periodic duplicates never count as sampled dexterous workspace',()=>{
  for(const angles of [[[0,0,0]],[[0,0,0],[0,0,2*Math.PI]]]){
    const input=request();input.reachability.points_m=[[1,0,0]];input.reachability.orientations_rpy_rad=angles;
    const result=analyze(input);assert.equal(result.points[0].category,'spatial_reachable');
    assert.equal(result.summary.distinct_orientation_count,1);assert.equal(result.summary.sampled_dexterous_point_count,null);assert.equal(result.scope.dexterity_evaluated,false);
  }
});

test('position-only mode is not a dexterity test; outer bound includes prismatic travel and root offset',()=>{
  const input=request();delete input.reachability.orientations_rpy_rad;
  input.model.joints[0].origin={xyz_m:[10,-3,2],rpy_rad:[.2,.3,.4]};
  const bound=conservativeReachBound(input.model);assert.deepEqual(bound.center_base_m,[10,-3,2]);assert.equal(bound.radius_m,3.6);
  input.mode='joint_sampling';delete input.reachability;input.sampling={method:'random',samples:100,seed:42};
  const sampled=analyze(input);assert.equal(sampled.summary.sampled_dexterous_point_count,null);
  assert.equal(sampled.summary.workspace_counts.spatial_reachable,100);
  for(const p of sampled.points)assert.ok(Math.hypot(...p.position_base_m.map((v,k)=>v-bound.center_base_m[k]))<=bound.radius_m+1e-9);
});

test('classification colors, spatial-union filters, sections and exports share the same categories',()=>{
  const result=analyze(request());assert.equal(new Set(result.points.map(pointColor)).size,5);
  assert.equal(result.points.filter(p=>pointVisible(p,'spatial')).length,3);
  assert.equal(result.points.filter(p=>pointVisible(p,'sampled_dexterous')).length,1);
  assert.ok(result.sections.xy.includes(3),'proven unreachable points must remain in section plots');
  const svg=projectionSVG(result,'xy',false,'unreachable');assert.ok(svg.includes(pointColor(result.points[3])));assert.equal((svg.match(/<circle /g)||[]).length,1);
  const csv=pointsCSV(result);assert.ok(csv.includes('position_solution_json'));assert.ok(csv.includes('unreachable'));assert.ok(csv.includes('spatial_reachable'));
  const report=reportMarkdown(result);assert.ok(report.includes('连续全姿态灵巧工作空间未证明'));assert.ok(report.includes('界内IK失败保持待定'));
});

test('annotated humanoid example distinguishes sampled dexterity, position only, unreachable and unresolved',async()=>{
  const input=parseRequestText(await readFile(new URL('../../assets/examples/worksheet-classification.jsonc',import.meta.url),'utf8'));
  const result=analyze(input);
  assert.deepEqual(result.points.map(p=>p.category),['sampled_dexterous','spatial_reachable','unreachable','unresolved']);
  assert.equal(result.summary.spatial_reachable_point_count,2);
  assert.equal(result.summary.sampled_dexterous_point_count,1);
  assert.equal(result.summary.proven_unreachable_point_count,1);
  assert.equal(result.summary.unresolved_point_count,1);
});
