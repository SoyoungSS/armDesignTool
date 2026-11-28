// Test-only planner; the real scene Skill runs the generated hold controller.
import {readFile,writeFile,mkdir,cp} from 'node:fs/promises';
import path from 'node:path';
let source='';for await(const chunk of process.stdin)source+=chunk;
const task=JSON.parse(source),request=JSON.parse(await readFile(task.artifacts.input_path));let result;
if(task.operation==='scene.plan'){
 if(request.input.description==='slow')await new Promise(resolve=>setTimeout(resolve,10000));
 result={schema_name:'arm-scene-plan',schema_version:'1.0',summary:'保持模型初始姿态',simulation_scope:'task',strategy:'沿用原位置伺服并观测误差',cycles:request.input.cycles,max_time_s:request.input.max_time_s,phases:[{name:'hold',action:'保持初始姿态',completion:'指定时间且误差满足阈值',on_failure:'停止并保留载荷'}],success_criteria:['关节误差小于0.02rad'],physics:['沿用模型，无附加物体'],assumptions:[],missing_inputs:[],ready_to_run:true,limitations:['测试工况'],sources_read:['SKILL.md','references/intake.md']};
}else if(task.operation==='scene.prepare'){
 const project=task.context.paths.project,plan=request.approved_plan;
 await mkdir(path.join(project,'scene'),{recursive:true});await cp(path.join(project,'source/mujoco'),path.join(project,'scene'),{recursive:true});await cp(path.join(project,'scene/robot.xml'),path.join(project,'scene/scene.xml'));
 for(const folder of ['assets','plan','code'])await mkdir(path.join(project,folder),{recursive:true});
 await writeFile(path.join(project,'assets/manifest.json'),JSON.stringify({source:'source',additional_assets:[]}));
 await writeFile(path.join(project,'plan/strategy.json'),JSON.stringify({method:'hold initial actuator target'}));
 const contract={schema_version:'1.1',model_input:'source',task_description:request.input.description,scene:'scene/scene.xml',adapter:'code/task.py',asset_manifest:'assets/manifest.json',strategy_file:'plan/strategy.json',robot_joints:'all',end_effector_sites:[],initial_keyframe:'initial',initial_conditions:'Generated initial keyframe',mode:'headless',cycles:plan.cycles,cycle_definition:'Confirmed hold time',between_cycles:'Single episode',timing:'Confirmed hold task',success_criteria:plan.success_criteria,physics_approval:'Approved original physics',strategy:plan.strategy,state_source:'MuJoCo truth',seed:42,dt_s:.002,max_time_s:plan.max_time_s,output_root:'runs',unresolved:[],confirmation:{},require_goal_tests:true,collision_policy:{default:'forbid',phases:{initial:[],hold:[]},required_phases:['hold']},collision_path_mode:'feedback_only',collision_path_reason:'Constant initial target feedback'};
 await writeFile(path.join(project,'task_contract.json'),JSON.stringify(contract));
 await writeFile(path.join(project,'code/task.py'),`import numpy as np
class Controller:
 def __init__(self,m,d,c):
  self.target=d.ctrl.copy();self.q=d.qpos.copy();self.duration=c['max_time_s']*.5;self.finished=False
 def control(self,m,d):
  d.ctrl[:]=self.target;self.finished=float(d.time)>=self.duration
  return dict(cycle=1,phase='hold',done=self.finished)
 def evaluate(self,m,d):
  error=float(np.max(np.abs(d.qpos-self.q)))
  return dict(success=self.finished and error<.02,cycles_completed=1 if self.finished else 0,position_error_rad=error)
 def goal_success(self,m,d):return bool(np.max(np.abs(d.qpos-self.q))<.02)
 def goal_tests(self,m):
  def good(model,data):data.qpos[:]=self.q
  def bad(model,data):data.qpos[:]=self.q;data.qpos[0]+=.1
  return [dict(name='held',expected=True,setup=good),dict(name='moved',expected=False,setup=bad)]
def create(m,d,c,run_dir):return Controller(m,d,c)
`);
 if(request.input.description==='repair-once'&&!request.repair_feedback)await writeFile(path.join(project,'code/task.py'),"raise RuntimeError('synthetic preflight failure for repair test')\n");
 result={ready:true,simulation_scope:plan.simulation_scope,assumptions:[],summary:'测试控制器已准备',missing_inputs:[],limitations:['准备阶段尚未执行仿真'],sources_read:['SKILL.md']};
}else throw Error('Wrong operation');
await writeFile(task.artifacts.output_path,JSON.stringify(result));process.stdout.write(JSON.stringify({protocol:task.protocol,task_id:task.task_id,ok:true}));
