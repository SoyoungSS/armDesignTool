import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {analyze} from '../lib/analyze.mjs';
import {generateRandomTrajectories} from '../lib/random-trajectory.mjs';

const example=async name=>JSON.parse(await readFile(new URL(`../../assets/examples/${name}-motion.json`,import.meta.url)));
const request=template=>({
  schema_name:'arm-random-motion-request',
  schema_version:'1.0',
  template,
  settings:{
    seed:42,trajectory_count:3,move_count:3,max_step_fraction:.1,
    min_speed_fraction:.4,max_speed_fraction:.8,min_hold_s:0,max_hold_s:.5,
  },
});

for(const name of ['worksheet','franka-fr3']){
  test(`${name} random trajectories are reproducible, bounded, and analyzable`,async()=>{
    const input=request(await example(name));
    const snapshot=structuredClone(input);
    const batch=generateRandomTrajectories(input);
    assert.deepEqual(batch,generateRandomTrajectories(input));
    assert.deepEqual(input,snapshot);
    const other=generateRandomTrajectories({...input,settings:{...input.settings,seed:0}});
    assert.notDeepEqual(batch.trajectories[0].request.trajectory,other.trajectories[0].request.trajectory);
    const joints=input.template.design_model.joints.filter(j=>j.type!=='fixed');
    for(const item of batch.trajectories){
      const trajectory=item.request.trajectory;
      assert.deepEqual(trajectory.start_q,input.template.trajectory.start_q);
      assert.equal(trajectory.segments.filter(s=>s.kind==='move').length,input.settings.move_count);
      let previous=trajectory.start_q;
      for(const segment of trajectory.segments){
        if(segment.kind==='hold'){
          assert.ok(segment.duration_s>0&&segment.duration_s<=input.settings.max_hold_s);
          continue;
        }
        for(const [index,joint] of joints.entries()){
          const [low,high]=joint.limits;
          const value=segment.q_target[index];
          assert.ok(value>=low&&value<=high);
          assert.ok(Math.abs(value-previous[index])<=(high-low)*input.settings.max_step_fraction+1e-12);
        }
        previous=segment.q_target;
      }
      const result=analyze(item.request);
      assert.equal(result.summary.constraint_status,'verified');
      assert.equal(result.summary.duration_s,item.duration_s);
      assert.ok(result.samples.some(s=>s.dq.some(x=>x!==0)));
      assert.ok(result.samples.some(s=>s.ddq.some(x=>x!==0)));
      assert.ok(result.samples.at(-1).dq.every(x=>x===0));
    }
  });
}

test('uses imported start in model order and supports a start at the position limits',async()=>{
  const template=await example('worksheet');
  const active=template.design_model.joints.filter(j=>j.type!=='fixed');
  const q=active.map(j=>j.limits[1]);
  template.trajectory={
    kind:'samples',joint_ids:active.map(j=>j.id).reverse(),
    samples:[{time_s:0,q:q.slice().reverse()},{time_s:1,q:q.slice().reverse()}],
  };
  const input=request(template);
  Object.assign(input.settings,{min_hold_s:0,max_hold_s:0});
  const trajectory=generateRandomTrajectories(input).trajectories[0].request.trajectory;
  assert.deepEqual(trajectory.start_q,q);
  assert.equal(trajectory.segments.length,3);
  assert.equal(analyze({...template,trajectory}).summary.constraint_status,'verified');
});

test('rejects invalid settings, missing limits and trajectories over the sample budget',async()=>{
  const input=request(await example('worksheet'));
  for(const settings of [{seed:.1},{move_count:21},{min_speed_fraction:1},{min_hold_s:1}]){
    assert.throws(()=>generateRandomTrajectories({...input,settings:{...input.settings,...settings}}),{code:'INVALID_INPUT'});
  }
  delete input.template.motion_limits;
  assert.throws(()=>generateRandomTrajectories(input),/explicit per-joint limits/);
  input.template.motion_limits=(await example('worksheet')).motion_limits;
  input.template.trajectory.sample_dt_s=.0001;
  input.settings.min_hold_s=input.settings.max_hold_s=3;
  assert.throws(()=>generateRandomTrajectories(input),/20,000 samples/);
});
