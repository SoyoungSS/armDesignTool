import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {analyze,normalizeInput} from '../lib/workspace.mjs';
import {forward} from '../lib/kinematics.mjs';
const raw=await readFile(new URL('../../assets/examples/franka-fr3.workspace.json',import.meta.url));
const input=JSON.parse(raw);
test('FR3 source snapshot is unchanged and includes fixed flange and nonzero joint limits',async()=>{
  const source=JSON.parse(await readFile(new URL('../../references/franka-fr3-source.json',import.meta.url)));
  assert.equal(createHash('sha256').update(raw).digest('hex'),source.sha256);
  assert.equal(input.model.joints.filter(j=>j.type==='revolute').length,7);assert.equal(input.model.joints.at(-1).type,'fixed');
  assert.equal(input.sampling.samples,30000);assert.equal(input.model.joints[3].limits[1],-.1518);assert.equal(input.model.joints[5].limits[0],.5445);
  assert.ok(input.model.joints.every(j=>!('components'in j)));
});
test('FR3 FK matches independent homogeneous-transform calculation; all 30k poses respect limits',()=>{
  const q=[0,-.5,0,-1.5,0,1.5,0],f=forward(input.model,q);
  const expected=[.3279778071661409,0,.8750542514671218];
  f.position_m.forEach((x,i)=>assert.ok(Math.abs(x-expected[i])<1e-12));
  const r=analyze(input);assert.equal(r.points.length,30000);assert.deepEqual(r.joint_names,input.model.joints.slice(0,7).map(j=>j.name));
  for(const p of r.points)for(const j of input.model.joints.slice(0,7))assert.ok(p.q[j.name]>=j.limits[0]&&p.q[j.name]<=j.limits[1]);
});
test('FR3 target position IK replays on original chain rather than worksheet geometry',()=>{
  const q=[0,-.5,0,-1.5,0,1.5,0],target=forward(input.model,q).position_m;
  const {sampling,...request}=input;request.mode='pose_reachability';request.reachability={points_m:[target],seed:42,ik:{attempts:8,max_iterations:200}};
  const r=analyze(request);assert.equal(r.points[0].spatial_status,'reachable');
  const solution=r.points[0].position_solution,f=forward(normalizeInput(request).model,r.joint_names.map(n=>solution.q[n]));
  assert.ok(Math.hypot(...f.position_m.map((x,i)=>x-target[i]))<.001);
});
