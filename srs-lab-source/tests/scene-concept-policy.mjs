import test from 'node:test';
import assert from 'node:assert/strict';
import {validateSceneInput,validateScenePlan} from '../server/scene-simulation.mjs';
import {conceptPolicy} from '../server/scene-policy.mjs';
const plan=()=>({schema_name:'arm-scene-plan',schema_version:'1.0',summary:'搬运',strategy:'',ready_to_run:false,phases:[],success_criteria:[],physics:[],assumptions:[],missing_inputs:['未知工具质量和节拍'],limitations:[],sources_read:[],cycles:null,max_time_s:null});

test('incomplete concept scene becomes an explicit runnable load proxy with recorded defaults',()=>{
 const input=validateSceneInput({description:'搬运小物体',auto_run:true});const result=validateScenePlan(plan(),input,{metadata:{}});
 assert.equal(input.assumption_policy,'concept');assert.equal(input.auto_run,true);
 assert.equal(result.ready_to_run,true);assert.equal(result.cycles,1);assert.equal(result.max_time_s,10);assert.equal(result.simulation_scope,'load_proxy');
 assert.deepEqual(result.missing_inputs,[]);assert.deepEqual(result.follow_up_questions,['未知工具质量和节拍']);assert.ok(result.assumptions.length>=3);assert.ok(result.phases.length);assert.ok(result.success_criteria.length);
});
test('strict mode still waits, explicit user limits are retained, and real blocks are not bypassed',()=>{
 const strict=validateSceneInput({description:'搬运',assumption_policy:'strict',auto_run:true});assert.equal(strict.auto_run,false);assert.equal(validateScenePlan(plan(),strict,{}).ready_to_run,false);
 const input=validateSceneInput({description:'搬运',cycles:2,max_time_s:20});assert.throws(()=>validateScenePlan({...plan(),cycles:3,max_time_s:20},input,{}),/轮数/);
 const blocked=validateScenePlan({...plan(),blocking_issues:['模型不可编译']},validateSceneInput({description:'搬运'}),{});assert.equal(blocked.ready_to_run,false);assert.deepEqual(blocked.missing_inputs,['模型不可编译']);
 const missingModel=validateScenePlan(plan(),validateSceneInput({description:'搬运'}),null);assert.equal(missingModel.ready_to_run,false);assert.ok(missingModel.missing_inputs.some(x=>x.includes('模型')));
 assert.equal(conceptPolicy(validateSceneInput({description:'动作'}),{ARM_SCENE_MAX_SIM_SECONDS:'5'}).defaults.max_time_s,5);
});
