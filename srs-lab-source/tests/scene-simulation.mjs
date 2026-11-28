import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,readFile,writeFile,rm,mkdir,symlink} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {validateSceneInput,validateScenePlan,createScenePlan,getSceneJob,runSceneJob,sceneJobSnapshot,cancelSceneJob,sceneArtifact,scenePython,refreshSceneExports} from '../server/scene-simulation.mjs';
import {generatedModelList,generatedModelViewer} from '../server/generated-models.mjs';
import {registerSceneModel,listSceneModels,containedFile} from '../server/scene-storage.mjs';
const fixture=fileURLToPath(new URL('./fixtures/scene-agent.mjs',import.meta.url));
const external={ARM_SCENE_AGENT_COMMAND:process.execPath,ARM_SCENE_AGENT_ARGS:JSON.stringify([fixture])};
const waitJob=async(id,env)=>{await (await getSceneJob(id,env)).promise;return sceneJobSnapshot(id,env);};
test('scene inputs preserve unknowns and reject changed cycles',()=>{
 const input=validateSceneInput({description:'保持姿态'});assert.equal(input.cycles,null);assert.equal(input.max_time_s,null);assert.equal(input.model_id,null);assert.throws(()=>validateSceneInput({description:'x',cycles:1.2}));
 const plan={schema_name:'arm-scene-plan',schema_version:'1.0',summary:'test',strategy:'test',ready_to_run:true,phases:[],success_criteria:[],physics:[],assumptions:[],missing_inputs:[],limitations:[],sources_read:[],cycles:null,max_time_s:null};
 validateScenePlan(plan,{...input,assumption_policy:'strict'},null);assert.equal(plan.ready_to_run,false);assert.ok(plan.missing_inputs.length>=3);assert.throws(()=>validateScenePlan({...plan,cycles:3},{...input,cycles:1},{}),/轮数/);
});
test('Agent planning requires missing information and supports cancellation',async()=>{
 const root=await mkdtemp(path.join(tmpdir(),'arm-scene-plan-')),env={...external,ARM_SCENE_STORAGE:root};
 try{const first=await createScenePlan({description:'只知道需要搬运'},env);const result=await waitJob(first.id,env);assert.equal(result.status,'needs_input');assert.ok(result.plan.missing_inputs.some(x=>x.includes('模型')));
 const slow=await createScenePlan({description:'slow'},env);await cancelSceneJob(slow.id,env);assert.equal((await waitJob(slow.id,env)).status,'cancelled');}finally{await rm(root,{recursive:true,force:true});}
});
test('artifacts reject traversal and symlink escape',async()=>{
 const root=await mkdtemp(path.join(tmpdir(),'arm-scene-paths-'));try{await mkdir(path.join(root,'project'));await writeFile(path.join(root,'secret'),'private');await symlink(path.join(root,'secret'),path.join(root,'project','link'));await assert.rejects(containedFile(path.join(root,'project'),'../secret'));await assert.rejects(containedFile(path.join(root,'project'),'link'));}finally{await rm(root,{recursive:true,force:true});}
});
test('approved scene runs the real Skill, loads and replay',{timeout:120000},async t=>{
 const python=scenePython();if(spawnSync(python,['-c','import mujoco,matplotlib,numpy'],{encoding:'utf8'}).status!==0){t.skip('MuJoCo task Python missing');return;}
 const root=await mkdtemp(path.join(tmpdir(),'arm-scene-physics-')),env={...external,ARM_SCENE_STORAGE:path.join(root,'store'),ARM_SCENE_PYTHON:python};
 try{
 const raw={schema_version:'1.0',model:{name:'ScenePhysicsTest',joints:[{name:'J1',type:'revolute',origin:{xyz_m:[0,0,.1],rpy_rad:[0,0,0]},axis:[0,0,1],limits:[-1,1],components:[{name:'link',mass_kg:1,com_m:[0,0,.05],inertia_frame:'link',inertia_kg_m2:[[.01,0,0],[0,.01,0],[0,0,.01]]}]}],tip:{xyz_m:[0,0,.15],rpy_rad:[0,0,0]}},physics:{fixed_base_massless:true},drives:{J1:{effort:10,velocity:1}},export:{mode:'strict',targets:['mujoco']}};
 const input=path.join(root,'model.json'),output=path.join(root,'generated');await writeFile(input,JSON.stringify(raw));
 const generator=fileURLToPath(new URL('../../arm-sim-model-generator/scripts/generate.py',import.meta.url));const generated=spawnSync(python,[generator,'generate','--input',input,'--output',output],{encoding:'utf8',timeout:30000});assert.equal(generated.status,0,generated.stderr+generated.stdout);
 const attempt=JSON.parse(await readFile(path.join(output,'latest_attempt.json'))),model=await registerSceneModel(attempt.run_directory,{name:'one joint test'},env);assert.ok(model.id);assert.equal((await listSceneModels(env)).length,1);
 const viewerMetadata=await generatedModelList(env);assert.equal(viewerMetadata[0].id,model.id);
 const metadataPath=path.join(env.ARM_SCENE_STORAGE,'models',model.id,'metadata.json');const savedMetadata=JSON.parse(await readFile(metadataPath));savedMetadata.mujoco_status='failed';await writeFile(metadataPath,JSON.stringify(savedMetadata));
 const view=await generatedModelViewer(model.id,env);assert.equal(view.frames.length,2);assert.equal(view.joints.length,1);assert.ok(view.geoms.length>0);assert.equal(view.export_status,'failed');assert.deepEqual(await generatedModelViewer(model.id,env),view);
 const job=await createScenePlan({description:'保持初始姿态，无工具物体，单次试验',model_id:model.id,cycles:1,max_time_s:.4},env),plan=await waitJob(job.id,env);assert.equal(plan.status,'ready');await assert.rejects(runSceneJob(job.id,{plan_sha256:'wrong'},env),/审阅/);
 await runSceneJob(job.id,{plan_sha256:plan.plan_sha256},env);const finished=await waitJob(job.id,env);assert.equal(finished.status,'completed',JSON.stringify(finished));assert.equal(finished.result.measured.status,'passed');assert.ok(finished.result.preparation_notes.includes('准备阶段尚未执行仿真'));assert.ok(!finished.result.limitations.some(note=>note.includes('尚未执行仿真')));assert.equal(finished.result.measured.evaluation.cycles_completed,1);assert.ok(finished.result.measured.samples>1);assert.equal(finished.result.load_spectrum.status,'exported');assert.equal(finished.result.replay_available,true,finished.result.replay_error);
 const automatic=await createScenePlan({description:'repair-once',auto_run:true}, {...env,ARM_SCENE_MAX_SIM_SECONDS:'.4'});
 const automaticResult=await waitJob(automatic.id,env);assert.equal(automaticResult.status,'completed',JSON.stringify(automaticResult));assert.equal(automaticResult.input.model_id,model.id);assert.equal(automaticResult.plan.cycles,1);assert.equal(automaticResult.plan.max_time_s,.4);assert.equal(automaticResult.result.assumption_policy,'concept');assert.ok(automaticResult.events.some(event=>event.message.includes('自动修复一次')));
 const executedContractPath=await sceneArtifact(automatic.id,automaticResult.result.run_path+'/input/task_contract.json',env);const executedContract=JSON.parse(await readFile(executedContractPath));assert.equal(executedContract.assumption_authorization.kind,'delegated_concept_defaults');assert.equal(executedContract.confirmation.evidence.authorization_type,'delegated_concept_defaults');
 const revision=await createScenePlan({description:'速度减半',previous_job_id:automatic.id},env);await waitJob(revision.id,env);const revisionJob=await getSceneJob(revision.id,env);assert.equal(revisionJob.previous.id,automatic.id);assert.equal(revisionJob.previous.input.description,'repair-once');
 const replay=JSON.parse(await readFile(await sceneArtifact(job.id,'viewer.json',env)));assert.ok(replay.frames.length>1);assert.ok(replay.frames.every(frame=>frame.phase==='hold'));
 const csv=finished.result.artifacts.find(a=>a.path===finished.result.run_path+'/data/joints.csv');assert.ok(csv);assert.match(await readFile(await sceneArtifact(job.id,csv.path,env),'utf8'),/J1/);
 assert.equal(finished.result.joint_exports.status,'exported');assert.equal(finished.result.joint_exports.modules.length,1);
 const entry=finished.result.joint_exports.modules[0];assert.equal(entry.joint,'J1');assert.equal(entry.state_samples,finished.result.measured.samples);
 const html=await readFile(await sceneArtifact(job.id,finished.result.run_path+'/'+finished.result.joint_exports.html,env),'utf8');assert.match(html,/data:image\/svg\+xml;base64,/);assert.match(html,/data-joint="J1"/);
 const perJoint=await readFile(await sceneArtifact(job.id,finished.result.run_path+'/'+entry.state_csv,env),'utf8');assert.match(perJoint,/position_deg/);assert.match(perJoint,/force_x_N/);
 const refreshed=await refreshSceneExports(job.id,env);assert.deepEqual(refreshed.result.measured,finished.result.measured);assert.equal(refreshed.status,'completed');assert.equal(refreshed.result.joint_exports.status,'exported');
 }finally{await rm(root,{recursive:true,force:true});}
});
