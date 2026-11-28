import {readFile,writeFile,mkdir,cp,readdir,stat,realpath} from 'node:fs/promises';
import {existsSync} from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {randomUUID,createHash} from 'node:crypto';
import {executeAgentTask,runProcess} from './execution-adapter.mjs';
import {codexReadiness} from './codex-runtime.mjs';
import {sceneRoot,sceneModel,writeJSON,containedFile,validId,listSceneModels} from './scene-storage.mjs';
import {isConcept,conceptPolicy,completeConceptPlan} from './scene-policy.mjs';
const skillDefault=fileURLToPath(new URL('../../arm-mujoco-task-sim',import.meta.url));
const bridge=fileURLToPath(new URL('./adapters/codex-scene-bridge.mjs',import.meta.url));
const runner=fileURLToPath(new URL('./run-scene-task.py',import.meta.url));
const viewer=fileURLToPath(new URL('./prepare-viewer.py',import.meta.url));
const core=['SKILL.md','references/intake.md','references/adapter.md','references/outputs.md','references/load-spectrum.md','references/collision.md','references/concept-trial.md'];
const jobs=new Map();
const error=(message,code='INVALID_INPUT')=>Object.assign(Error(message),{code});
const digest=value=>createHash('sha256').update(JSON.stringify(value)).digest('hex');
const active=new Set(['planning','preparing','checking','running','exporting']);
const strings=x=>Array.isArray(x)&&x.every(item=>typeof item==='string');
const json=file=>readFile(file,'utf8').then(JSON.parse);
function agentEnv(env){return {...env,ARM_EXECUTION_MODE:'agent',ARM_AGENT_COMMAND:env.ARM_SCENE_AGENT_COMMAND||env.ARM_AGENT_COMMAND||process.execPath,
  ARM_AGENT_ARGS:env.ARM_SCENE_AGENT_ARGS??env.ARM_AGENT_ARGS??JSON.stringify([bridge])};}
export const scenePython=(env=process.env)=>{
  if(env.ARM_SCENE_PYTHON||env.ARM_SIM_PYTHON)return env.ARM_SCENE_PYTHON||env.ARM_SIM_PYTHON;
  for(const relative of ['../../.runtime/scene-python/bin/python','../../.venv-arm-sim/bin/python']){
    const command=fileURLToPath(new URL(relative,import.meta.url));if(existsSync(command))return command;
  }
  return 'python3';
};
export async function sceneStatus(env=process.env){
  let status;
  if(env.ARM_SCENE_AGENT_COMMAND||env.ARM_AGENT_COMMAND)status={configured:true,provider:'external'};
  else status=await codexReadiness(env);
  try{await readFile(path.join(env.ARM_SCENE_SKILL_DIR||skillDefault,'SKILL.md'));}catch{return {configured:false,message:'服务器未部署场景仿真 Skill。'};}
  return {...status,message:status.configured?'场景仿真 Agent 已连接；支持按常规假设自动试算并导出载荷谱。':status.message?.replaceAll('构型选择','场景仿真')};
}
export function validateSceneInput(raw,env=process.env){
  if(!raw||typeof raw!=='object'||Array.isArray(raw))throw error('请输入场景描述。');
  const input={};
  for(const key of ['description','success_criteria','tool_environment','strategy','randomization','additional_notes']){
    if(raw[key]!=null&&typeof raw[key]!=='string')throw error(key+' 应为文本');
    input[key]=raw[key]?.trim()||null;if(input[key]?.length>12000)throw error('场景文本过长，请控制在 12000 字以内。');
  }
  if(!input.description)throw error('请填写要仿真的场景与动作。');
  input.model_id=raw.model_id||null;if(input.model_id&&!validId(input.model_id))throw error('模型编号无效。');
  for(const key of ['cycles','max_time_s']){
    const value=raw[key];input[key]=value??null;
    if(value!=null&&(typeof value!=='number'||!Number.isFinite(value)||value<=0||(key==='cycles'&&!Number.isInteger(value))))throw error(key+' 应为正数，轮数须为整数');
  }
  if(input.cycles>1000)throw error('轮数超过当前网页任务限制。');
  if(input.max_time_s>Number(env.ARM_SCENE_MAX_SIM_SECONDS||120))throw error('仿真时长超过服务器当前限制。');
  input.assumption_policy=raw.assumption_policy??'concept';
  if(!['concept','strict'].includes(input.assumption_policy))throw error('未知的试算模式。');
  input.auto_run=input.assumption_policy==='concept'&&raw.auto_run===true;
  input.previous_job_id=raw.previous_job_id||null;if(input.previous_job_id&&!validId(input.previous_job_id))throw error('上一任务编号无效。');
  input.mode='headless';input.presentation='browser_replay';
  return input;
}
export function validateScenePlan(plan,input,model,env=process.env){
  if(!plan||plan.schema_name!=='arm-scene-plan'||plan.schema_version!=='1.0'||typeof plan.summary!=='string'||typeof plan.strategy!=='string'||typeof plan.ready_to_run!=='boolean'||!Array.isArray(plan.phases))throw error('Agent 场景方案格式无效。','INVALID_AGENT_RESULT');
  for(const key of ['success_criteria','physics','assumptions','missing_inputs','limitations','sources_read'])if(!strings(plan[key]))throw error('方案列表格式无效：'+key,'INVALID_AGENT_RESULT');
  for(const phase of plan.phases)if(!phase||['name','action','completion','on_failure'].some(key=>typeof phase[key]!=='string'))throw error('阶段定义不完整。','INVALID_AGENT_RESULT');
  for(const key of ['follow_up_questions','blocking_issues']){plan[key]??=[];if(!strings(plan[key]))throw error('补充建议或执行阻塞格式无效。','INVALID_AGENT_RESULT');}
  plan.simulation_scope??=isConcept(input)&&!plan.ready_to_run&&plan.missing_inputs.length?'load_proxy':'task';if(!['task','load_proxy'].includes(plan.simulation_scope))throw error('未知的仿真范围。','INVALID_AGENT_RESULT');
  if(isConcept(input))completeConceptPlan(plan,input,model,env);
  if(!model)plan.missing_inputs.push('请选择已生成的完整 MuJoCo 模型。');
  if(!Number.isInteger(plan.cycles)||plan.cycles<1||plan.cycles>1000)plan.missing_inputs.push('请明确仿真轮数或单次任务范围。');
  if(typeof plan.max_time_s!=='number'||!Number.isFinite(plan.max_time_s)||plan.max_time_s<=0)plan.missing_inputs.push('请明确本次最大仿真时长。');
  if(plan.max_time_s>Number(env.ARM_SCENE_MAX_SIM_SECONDS||120))plan.missing_inputs.push('计划时长超过当前服务器限制，请缩短或由管理员调整。');
  if(input.cycles!==null&&plan.cycles!==input.cycles)throw error('Agent 不可替换用户指定的轮数。','INVALID_AGENT_RESULT');
  if(input.max_time_s!==null&&plan.max_time_s!==input.max_time_s)throw error('Agent 不可替换用户指定的仿真时长。','INVALID_AGENT_RESULT');
  if(!plan.success_criteria.length||!plan.phases.length)plan.missing_inputs.push('补充任务阶段和可测量成功判据。');
  plan.missing_inputs=[...new Set(plan.missing_inputs)];
  plan.assumptions=[...new Set(plan.assumptions)];
  plan.ready_to_run=plan.ready_to_run&&plan.missing_inputs.length===0;
  return plan;
}
function publicJob(job){return {id:job.id,status:job.status,stage:job.stage,input:job.input,model:job.model?.metadata??null,plan:job.plan??null,
  plan_sha256:job.plan_sha256??null,result:job.result??null,error:job.error??null,created_at:job.created_at,updated_at:job.updated_at,events:job.events??[]};}
async function save(job){job.updated_at=new Date().toISOString();job.writeQueue=(job.writeQueue??Promise.resolve()).then(()=>writeJSON(path.join(job.directory,'job.json'),publicJob(job)));await job.writeQueue;}
async function stage(job,status,label){job.status=status;job.stage=label;job.events.push({at:new Date().toISOString(),message:label});await save(job);}
export async function getSceneJob(id,env=process.env){
  if(!validId(id))throw error('任务编号无效。');
  if(!jobs.has(id)){
    let record;try{record=await json(path.join(sceneRoot(env),'jobs',id,'job.json'));}catch{throw error('场景任务不存在。','NOT_FOUND');}
    const model=record.input.model_id?await sceneModel(record.input.model_id,env):null;
    const job={...record,model,directory:path.join(sceneRoot(env),'jobs',id),env};
    if(active.has(record.status)){job.status='interrupted';job.error='服务器曾重启，此任务已中断，请重新提交。';await save(job);}
    jobs.set(id,job);
  }
  return jobs.get(id);
}
export async function sceneJobSnapshot(id,env=process.env){return publicJob(await getSceneJob(id,env));}
function assertAvailable(){if([...jobs.values()].some(job=>active.has(job.status)))throw error('已有场景任务正在执行，请稍后重试。','BUSY');}
async function runAgent(job,operation){
  const env=job.env,skill=path.resolve(env.ARM_SCENE_SKILL_DIR||skillDefault),project=path.join(job.directory,'project');
  await mkdir(project,{recursive:true});
  const inputPath=path.join(job.directory,operation==='scene.plan'?'request.json':'approved-task.json'),outputPath=path.join(job.directory,operation==='scene.plan'?'plan.json':'prepared.json');
  const sources=await Promise.all(core.map(async file=>({path:file,text:await readFile(path.join(skill,file),'utf8')})));
  if(operation==='scene.plan')await writeJSON(inputPath,{input:job.input,model:job.model?.metadata??null,previous_task:job.previous??null,assumption_policy:conceptPolicy(job.input,env,job.previous?.plan)});
  await executeAgentTask({operation,skill:'arm-mujoco-task-sim',cwd:project,inputPath,outputPath,timeoutMs:Number(env.ARM_SCENE_AGENT_TIMEOUT_MS||300000),signal:job.controller.signal,
    context:{assumption_policy:job.input.assumption_policy,supplied_sources:sources,paths:{skill,project,model:job.model?.directory??null,python:scenePython(env),inspection:path.join(project,'model_inspection.json'),approval:path.join(job.directory,'approval.json')},
      instructions:operation==='scene.plan'?
        '按 Skill 生成可以实现的场景方案。input.assumption_policy=concept 时已委托常规补齐并优先试算：读取 concept-trial.md，轮数/节拍/工具/环境/摩擦/判据/控制策略不完整时自行选常规值并写入 assumptions/physics，不把一般缺项当 blocking_issues。默认一轮、低速平滑、5s左右周期和约1s稳定、10s超时上限；按实际限速调整。无法精细模拟接触时先构建标明 load_proxy 的载荷代理，说明原任务哪些动作未验证。missing_inputs 仅作后续建议，blocking_issues 只放模型不可读/无法编译/拓扑等真正执行阻塞。不要因上游部分姿态碰撞未通过就立即放弃，可在限位内规划安全场景起点并记录。保留用户明确值。previous_task 是上一场景，上述描述可以是“速度减半、物体改1kg”等增量修改，未修改项沿用。strict 模式仍需关键参数齐备并等待审阅。来源写真实相对路径；mode=headless+浏览器回放。':
        '按 approved-task.json 的场景方案实现 task_contract.json、scene、assets、plan/strategy.json、code/task.py，读取 inspection、concept-trial.md、scene-strategy.md 与相关适配器。概念模式已授权补齐常规参数，不因普通物理缺项返回 ready=false；用实际具体值写 assumptions、资产清单和合同，未精细表达的工具/接触可改为明确的 load_proxy 并返回 simulation_scope。若 repair_feedback 存在，依据真实诊断修复一次：只改项目场景/代码/受委托假设，不删除碰撞、不改原模型、不突破用户硬约束。robot_joints="all"，mode="headless"，output_root="runs"，cycles/max_time_s 必须等于给定计划；保留原驱动限幅、安装与全关节测点，已有负载不重复添加。原 source 和 Skill 只读；任务场景可在限位内选可行初姿态并记录，用户明确要求的起点除外。不要执行 run_task/preflight/analyze 或伪造报告；控制器不得覆写 qpos/qvel/time 或调用 mj_step。evaluate 和正负例判据须来自物理状态，不写死成功；不能以单纯到达超时判成功。confirmation由服务器绑定真实请求与概念委托，不伪造数值确认。缺乏真实夹爪时可用经标注的等效载荷代理；仅真正不可实现时 ready=false、blocking问题。优先复用现有工作负载适配器。'
},
  },agentEnv(env));
  const info=await stat(outputPath);if(info.size>1024*1024)throw error('Agent 返回过大。','INVALID_AGENT_RESULT');
  return json(outputPath);
}
async function failJob(job,err){
  try{job.result??={measured:null,limitations:['执行未完成，请查看错误与实际诊断文件。'],artifacts:await artifacts(path.join(job.directory,'project'))};}catch{}
  job.error=job.controller?.signal.aborted?'已取消当前场景任务。':String(err.message).slice(0,2000);await stage(job,job.controller?.signal.aborted?'cancelled':'failed',job.error);}
export async function createScenePlan(raw,env=process.env){
  assertAvailable();const input=validateSceneInput(raw,env),health=await sceneStatus(env);
  if(!health.configured)throw error(health.message,'AGENT_NOT_CONFIGURED');
  if(!input.model_id&&isConcept(input)){const latest=(await listSceneModels(env))[0];if(latest){input.model_id=latest.id;input.model_selection='latest_generated';}}
  const model=input.model_id?await sceneModel(input.model_id,env):null;
  let previous=null;if(input.previous_job_id){try{const prior=await getSceneJob(input.previous_job_id,env);previous={id:prior.id,input:prior.input,plan:prior.plan,result_summary:prior.result?.measured??null};}catch{input.previous_context_unavailable=true;}}
  assertAvailable();
  const job={id:randomUUID(),input,model,previous,env,created_at:new Date().toISOString(),events:[],controller:new AbortController()};
  job.directory=path.join(sceneRoot(env),'jobs',job.id);jobs.set(job.id,job);
  await stage(job,'planning','Agent 正在读取场景仿真 Skill、核对模型并生成方案');
  job.promise=(async()=>{
    try{const plan=await runAgent(job,'scene.plan');job.controller.signal.throwIfAborted();job.plan=validateScenePlan(plan,input,model,env);job.plan_sha256=digest(job.plan);if(job.plan.ready_to_run&&input.auto_run){await startSceneRun(job,'delegated_concept_defaults');await job.promise;}else await stage(job,job.plan.ready_to_run?'ready':'needs_input',job.plan.ready_to_run?'场景方案已生成，请审阅后运行':'缺少可运行模型或存在执行阻塞');}
    catch(err){await failJob(job,err);}
  })();return publicJob(job);
}
async function pythonStep(job,args,label,timeoutMs=180000){
  await stage(job,job.status,label);
  const logDirectory=path.join(job.directory,'project','logs');await mkdir(logDirectory,{recursive:true});
  const log=path.join(logDirectory,String(job.events.length).padStart(3,'0')+'.log');
  const cache=path.join(sceneRoot(job.env),'cache','matplotlib');await mkdir(cache,{recursive:true});
  try{
    const result=await runProcess(scenePython(job.env),args,{cwd:path.join(job.directory,'project'),timeoutMs,signal:job.controller.signal,
      env:{...job.env,MPLCONFIGDIR:cache,PYTHONDONTWRITEBYTECODE:'1'},maxBuffer:4*1024*1024});
    await writeFile(log,result.stdout+'\n'+result.stderr);return result;
  }catch(error){await writeFile(log,String(error.stdout??'')+'\n'+String(error.stderr??error.message));throw error;}

}
async function readIf(file){try{return await json(file);}catch{return null;}}
async function artifacts(root,prefix=''){
  const result=[];
  for(const entry of await readdir(path.join(root,prefix),{withFileTypes:true})){
    const relative=path.posix.join(prefix,entry.name);if(entry.isSymbolicLink())continue;
    if(entry.isDirectory()){if(!['source','runner_tools','__pycache__'].includes(entry.name))result.push(...await artifacts(root,relative));}
    else if(entry.isFile())result.push({path:relative,size:(await stat(path.join(root,relative))).size});
  }
  return result;
}
export async function runSceneJob(id,approval,env=process.env){
  assertAvailable();const job=await getSceneJob(id,env);
  if(job.status!=='ready'||!job.plan?.ready_to_run||approval?.plan_sha256!==job.plan_sha256)throw error('请先审阅当前完整方案，再点击按方案运行。');
  return startSceneRun(job,'reviewed_plan');
}
async function startSceneRun(job,authorization){
  const env=job.env;
  job.controller=new AbortController();job.error=null;
  const project=path.join(job.directory,'project'),skill=path.resolve(env.ARM_SCENE_SKILL_DIR||skillDefault);
  const evidence={action:authorization==='delegated_concept_defaults'?'收到允许常规补齐并自动执行的概念试算请求':'收到确认当前方案并启动运行的 API 请求',authorization_type:authorization,at:new Date().toISOString(),plan_sha256:job.plan_sha256,input:job.input,approved_plan:job.plan,previous_task:job.previous??null,assumption_policy:conceptPolicy(job.input,env,job.previous?.plan),mode:'headless',presentation:'browser_replay'};
  await stage(job,'preparing','检查模型与环境，Agent 正在生成场景和任务适配器');
  await writeJSON(path.join(job.directory,'approval.json'),evidence);
  await writeJSON(path.join(job.directory,'approved-task.json'),evidence);
  job.promise=(async()=>{
    try{
      await cp(job.model.directory,path.join(project,'source'),{recursive:true});
      await pythonStep(job,[path.join(skill,'scripts/environment.py'),'probe','--mode','headless'],'检查服务器 MuJoCo 与载荷分析环境');
      await pythonStep(job,[path.join(skill,'scripts/inspect_generator.py'),path.join(project,'source'),'--out',path.join(project,'model_inspection.json')],'检查完整生成模型、驱动与载荷测点');
      let prepared,assumptions;
      for(let attempt=0;;attempt++){
        try{
      await stage(job,'preparing','Agent 正在生成本次场景、合同和控制器');
      prepared=await runAgent(job,'scene.prepare');
      if(!prepared||typeof prepared.ready!=='boolean'||!strings(prepared.missing_inputs)||!strings(prepared.limitations))throw error('Agent 未返回有效的场景准备状态。','INVALID_AGENT_RESULT');
      if(!prepared.ready||prepared.missing_inputs.length){if(isConcept(job.input))throw Error('场景准备仍未落实常规补齐：'+(prepared.summary??'')+' '+prepared.missing_inputs.join('；'));job.plan.missing_inputs=prepared.missing_inputs;job.plan.ready_to_run=false;await stage(job,'needs_input',prepared.summary||'场景实现仍需补充信息');return;}
      const contract=await json(path.join(project,'task_contract.json'));
      if(contract.mode!=='headless'||contract.cycles!==job.plan.cycles||contract.max_time_s!==job.plan.max_time_s||contract.output_root!=='runs'||contract.robot_joints!=='all'||contract.schema_version!=='1.1')throw error('任务合同与已批准的运行范围不一致。','INVALID_AGENT_RESULT');
      for(const key of ['scene','adapter','asset_manifest','strategy_file'])await containedFile(project,contract[key]);
      assumptions=[...new Set([...job.plan.assumptions,...(prepared.assumptions??[]),...(contract.assumptions??[])].map(value=>typeof value==='string'?value:JSON.stringify(value)))];
      if(prepared.simulation_scope&&!['task','load_proxy'].includes(prepared.simulation_scope))throw error('准备阶段仿真范围无效。','INVALID_AGENT_RESULT');
      contract.simulation_scope=prepared.simulation_scope??job.plan.simulation_scope;
      contract.assumption_policy=job.input.assumption_policy??'strict';contract.assumptions=assumptions;
      if(isConcept(job.input))contract.assumption_authorization={kind:'delegated_concept_defaults',evidence,scope:conceptPolicy(job.input,env).authorization};
      await writeJSON(path.join(project,'assumptions.json'),{policy:contract.assumption_policy,assumptions,user_input:job.input});
      contract.model_input='source';contract.confirmation=Object.fromEntries(['mode','cycles','task','assets_physics','strategy','timing','outputs'].map(key=>[key,true]));
      contract.confirmation.evidence=evidence;await writeJSON(path.join(project,'task_contract.json'),contract);
      await stage(job,'checking','检查任务合同、碰撞与成功判据');
      await pythonStep(job,[path.join(skill,'scripts/contracts.py'),path.join(project,'task_contract.json')],'核对任务合同');
      const preflight=path.join(job.directory,'preflight-summary.json');
      await pythonStep(job,[runner,'--skill',skill,'--project',project,'--summary',preflight,'--preflight'],'运行场景预检');
      if(!(await json(preflight)).ok)throw Error('场景预检未通过。');
          break;
        }catch(err){
          if(!isConcept(job.input)||attempt>=1||job.controller.signal.aborted)throw err;
          const repair_feedback={attempt:1,error:String(err.message).slice(-3000),logs_directory:path.join(project,'logs'),instruction:'在已委托的常规假设范围内修复项目代码、场景或未指定的起点；保留诊断，不修改源模型或删除碰撞/限幅检查。'};
          await writeJSON(path.join(job.directory,'approved-task.json'),{...evidence,repair_feedback});
          await stage(job,'preparing','预检或场景准备未通过，Agent 正在自动修复一次');
        }
      }
      await stage(job,'running','MuJoCo 正在实际执行场景并记录全关节载荷');
      const summary=path.join(job.directory,'run-summary.json');
      await pythonStep(job,[runner,'--skill',skill,'--project',project,'--summary',summary],'执行任务与导出载荷谱',Number(env.ARM_SCENE_RUN_TIMEOUT_MS||300000));
      const measured=await json(summary),run=await realpath(measured.run_directory),runs=await realpath(path.join(project,'runs'));
      if(path.relative(runs,run).startsWith('..')||run===runs)throw Error('仿真返回了无效的运行目录。');
      const result=await json(path.join(run,'reports/result.json'));
      const coverage=await readIf(path.join(run,'reports/load_coverage.json')),spectrum=await readIf(path.join(run,'reports/load_spectrum.json'));
      const jointExports=await readIf(path.join(run,'reports/joint_exports.json'));
      const savedContract=await json(path.join(run,'input/task_contract.json'));
      const scene=await containedFile(path.join(run,'input'),savedContract.scene);
      let replayAvailable=false,replayError=null;
      if(result.samples>0){try{await stage(job,'exporting','生成本次实测姿态回放');await pythonStep(job,[viewer,'--scene',scene,'--output',path.join(project,'viewer.json'),'--replay',path.join(run,'data/replay.jsonl'),'--result',path.join(run,'reports/result.json')],'生成网页三维回放');replayAvailable=true;}catch(err){if(job.controller.signal.aborted)throw err;replayError=err.message;}}
      job.result={assumption_policy:job.input.assumption_policy??'strict',simulation_scope:prepared.simulation_scope??job.plan.simulation_scope,assumptions,follow_up_questions:job.plan.follow_up_questions??[],measured:result,load_coverage:coverage,load_spectrum:spectrum,joint_exports:jointExports,run_path:path.relative(project,run),replay_available:replayAvailable,replay_error:replayError,
        preparation_notes:prepared.limitations,
        limitations:['本次只覆盖所记录模型、实际假设和工况，不代表实机、寿命或全工作空间验收。','网页回放来自实测姿态，不代表官方 WebViewer 实时交互已验收。'],artifacts:await artifacts(project)};
      await stage(job,result.status==='passed'?'completed':'simulation_failed',result.status==='passed'?'实际任务通过，回放与载荷记录已生成':'任务未通过，保留本次实际记录与失败证据');
    }catch(err){await failJob(job,err);}
  })();return publicJob(job);
}
export async function cancelSceneJob(id,env=process.env){const job=await getSceneJob(id,env);job.controller?.abort(Error('SCENE_CANCELLED'));if(active.has(job.status))await stage(job,'cancelled','已取消当前场景任务');return publicJob(job);}
export async function sceneArtifact(id,relative,env=process.env){const job=await getSceneJob(id,env);return containedFile(path.join(job.directory,'project'),relative);}
export {listSceneModels};

export async function refreshSceneExports(id,env=process.env){
  const job=await getSceneJob(id,env);
  if(active.has(job.status))throw error('请等待本次实际运行结束后整理导出。','BUSY');
  if(job.exportPromise)return job.exportPromise;
  if(!job.result?.run_path)throw error('本任务尚无可整理的实际运行记录。');
  const work=(async()=>{
    const project=path.join(job.directory,'project');
    const metadata=await containedFile(project,job.result.run_path+'/data/metadata.json');
    const run=path.dirname(path.dirname(metadata));
    const runs=await realpath(path.join(project,'runs'));
    if(path.relative(runs,run).startsWith('..')||run===runs)throw error('实际运行目录无效。');
    const skill=path.resolve(job.env.ARM_SCENE_SKILL_DIR||skillDefault);
    try{
      await runProcess(scenePython(job.env),[path.join(skill,'scripts/joint_exports.py'),run],
        {cwd:project,env:{...job.env,PYTHONDONTWRITEBYTECODE:'1'},timeoutMs:180000,maxBuffer:1024*1024});
    }finally{
      job.result.joint_exports=await readIf(path.join(run,'reports/joint_exports.json'));
      job.result.artifacts=await artifacts(project);await save(job);
    }
    return publicJob(job);
  })();
  job.exportPromise=work;
  try{return await work;}finally{job.exportPromise=null;}
}

export async function sceneArchive(id,env=process.env){
  const job=await getSceneJob(id,env);if(active.has(job.status)||job.exportPromise)throw error('任务或数据导出仍在运行，请完成后下载完整记录。');
  const output=path.join(job.directory,'scene-records.zip');
  const script=`import os,sys,zipfile
from pathlib import Path
root=Path(sys.argv[1]).resolve()
with zipfile.ZipFile(sys.argv[2],'w',zipfile.ZIP_DEFLATED) as z:
 for folder,dirs,files in os.walk(root,followlinks=False):
  dirs[:]=[d for d in dirs if not (Path(folder)/d).is_symlink() and d!='__pycache__']
  for name in files:
   file=Path(folder)/name
   if not file.is_symlink() and file.is_file() and file.resolve().is_relative_to(root):z.write(file,file.relative_to(root))`;
  await runProcess(scenePython(env),['-c',script,path.join(job.directory,'project'),output],{timeoutMs:60000});return output;
}
