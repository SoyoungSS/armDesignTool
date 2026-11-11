import {createHash} from 'node:crypto';
import {validateContract,invalid} from '../../../shared/contracts/validate.mjs';
import {validateDesignModel,geometryModel} from '../../../shared/model/design-model.mjs';
import {forward,jacobian,rpy,mv,mm,add} from '../../../shared/kinematics/serial.mjs';
import {singularity} from '../../../shared/kinematics/singularity.mjs';
import {object,number,vector,validateLimits,generate,importTrajectory} from './trajectory.mjs';
import {prepareDynamics,inverseDynamics,summarizeDynamics} from './dynamics.mjs';
export const ENGINE_VERSION='0.2.0';
const canonical=v=>Array.isArray(v)?'['+v.map(canonical).join(',')+']':v&&typeof v==='object'?'{'+Object.keys(v).sort().map(k=>JSON.stringify(k)+':'+canonical(v[k])).join(',')+'}':JSON.stringify(v);
export const hash=v=>createHash('sha256').update(canonical(v)).digest('hex');
const multiply=(cols,q)=>cols[0].map((_,i)=>cols.reduce((sum,c,j)=>sum+c[i]*q[j],0));
export function analyze(input){
  validateContract('motion-request',input);object(input,'$',['schema_name','schema_version','design_model','trajectory','motion_limits','singularity','tcp_offset','dynamics'],['schema_name','schema_version','design_model','trajectory']);
  if(input.schema_name!=='arm-motion-request'||input.schema_version!=='1.0')invalid('$','expected arm-motion-request 1.0');
  const audit=validateDesignModel(input.design_model),model=geometryModel(input.design_model),active=input.design_model.joints.filter(j=>j.type!=='fixed');
  if(active.length>16)invalid('design_model.joints','motion analysis supports at most 16 active joints');
  if(input.tcp_offset!==undefined){object(input.tcp_offset,'tcp_offset',['xyz_m','rpy_rad'],['xyz_m','rpy_rad']);const tip={xyz_m:vector(input.tcp_offset.xyz_m,'tcp_offset.xyz_m',3),rpy_rad:vector(input.tcp_offset.rpy_rad,'tcp_offset.rpy_rad',3)};let name='analysis_flange';while(model.joints.some(j=>j.name===name))name+='_';model.joints.push({name,type:'fixed',origin:model.tip});model.tip=tip;}
  const defaults={task:'pose',characteristic_length_m:1,rank_relative_tolerance:1e-8,near_sigma_threshold:.05,condition_warning:100};
  if(input.singularity!==undefined)object(input.singularity,'singularity',Object.keys(defaults));
  const settings={...defaults,...input.singularity};
  if(!['position','pose'].includes(settings.task))invalid('singularity.task','expected position or pose');
  for(const field of ['characteristic_length_m','rank_relative_tolerance','near_sigma_threshold','condition_warning'])number(settings[field],'singularity.'+field,1e-12,1e9);
  if(settings.rank_relative_tolerance>=1)invalid('singularity.rank_relative_tolerance','must be less than 1');
  const limits=validateLimits(input.motion_limits,active);
  const trajectory=input.trajectory?.kind==='waypoints'?generate(input.trajectory,active,limits):input.trajectory?.kind==='samples'?importTrajectory(input.trajectory,active,limits):invalid('trajectory.kind','expected waypoints or samples');
  const dynamicsEnabled=input.dynamics?.enabled===true;
  if(dynamicsEnabled&&trajectory.samples.some(s=>!s.dq||!s.ddq))invalid('trajectory.samples','dynamics requires dq and ddq at every sample');
  const dynamicModel=dynamicsEnabled?prepareDynamics(input.design_model,input.dynamics):null;
  const base=input.design_model.base_in_world??{xyz_m:[0,0,0],rpy_rad:[0,0,0]},R=rpy(base.rpy_rad),events=[...trajectory.events];
  const samples=trajectory.samples.map(s=>{
    const f=forward(model,s.q),J=jacobian(f,true),metrics=singularity(J,active.map(j=>j.type),settings);
    const twist=s.dq?multiply(J,s.dq):null;let acc=null;
    if(s.dq&&s.ddq){
      const maxSpeed=Math.max(...s.dq.map(Math.abs)),h=1e-5/Math.max(1,maxSpeed);
      const A=jacobian(forward(model,s.q.map((q,i)=>q+s.dq[i]*h)),true),B=jacobian(forward(model,s.q.map((q,i)=>q-s.dq[i]*h)),true),rate=A.map((col,j)=>col.map((v,i)=>(v-B[j][i])/(2*h)));
      const a=multiply(J,s.ddq),b=multiply(rate,s.dq);acc=a.map((v,i)=>v+b[i]);
    }
    const near=active.filter((j,i)=>Math.min(s.q[i]-j.limits[0],j.limits[1]-s.q[i])/(j.limits[1]-j.limits[0])<.05).map(j=>j.id);
    const dynamicLoads=dynamicModel?inverseDynamics(dynamicModel,s.q,s.dq,s.ddq):null;
    return {...s,...(dynamicLoads?{effort:dynamicLoads.map(l=>l.effort),dynamic_loads:dynamicLoads}:{}),tcp_base:{position_m:f.position_m,rotation:f.rotation,linear_velocity_m_s:twist?.slice(0,3)??null,angular_velocity_rad_s:twist?.slice(3)??null,linear_acceleration_m_s2:acc?.slice(0,3)??null,angular_acceleration_rad_s2:acc?.slice(3)??null},tcp_world:{position_m:add(base.xyz_m,mv(R,f.position_m)),rotation:mm(R,f.rotation)},singularity:metrics,near_limit_joint_ids:near};
  });
  const intervals=[];let current=null;
  for(const s of samples){const status=s.singularity.status;if(status==='regular'){if(current){intervals.push(current);current=null;}continue;}if(!current||current.status!==status){if(current)intervals.push(current);current={status,start_s:s.time_s,end_s:s.time_s,min_sigma:s.singularity.sigma_min,critical_time_s:s.time_s,critical_q:s.q};}else{current.end_s=s.time_s;if(s.singularity.sigma_min<current.min_sigma){current.min_sigma=s.singularity.sigma_min;current.critical_time_s=s.time_s;current.critical_q=s.q;}}}if(current)intervals.push(current);
  for(const interval of intervals)events.push({type:'singularity_risk',...interval});
  const violated=events.some(e=>['motion_limit_exceeded','position_limit_exceeded'].includes(e.type)),hasDerivatives=samples.every(s=>s.dq&&s.ddq),fullCoverage=trajectory.constraint_coverage==='analytic_segment_extrema';
  const summary={sample_count:samples.length,duration_s:samples.at(-1).time_s-samples[0].time_s,trajectory_feasible:violated?false:limits&&hasDerivatives&&fullCoverage?true:null,constraint_status:violated?'violated':limits&&hasDerivatives?(fullCoverage?'verified':'samples_passed'):'incomplete',singular_sample_count:samples.filter(s=>s.singularity.status==='singular').length,near_singular_sample_count:samples.filter(s=>s.singularity.status==='near_singular').length,task_underactuated:samples[0].singularity.task_underactuated,minimum_sigma:Math.min(...samples.map(s=>s.singularity.sigma_min)),joint_peaks:active.map((j,i)=>({joint_id:j.id,position_min:Math.min(...samples.map(s=>s.q[i])),position_max:Math.max(...samples.map(s=>s.q[i])),speed_max:samples.every(s=>s.dq)?Math.max(...samples.map(s=>Math.abs(s.dq[i]))):null,acceleration_max:hasDerivatives?Math.max(...samples.map(s=>Math.abs(s.ddq[i]))):null}))};
  const warnings=[...audit.assumptions,'Collision checks, Cartesian path tracking and automatic singularity avoidance are outside this analysis.','Singularity thresholds depend on task and scaling; alerts do not by themselves invalidate a joint-space trajectory.','Singularity and near-limit intervals are sampled observations, not certified continuous boundaries.'];
  if(dynamicsEnabled){
    summary.dynamic_joints=summarizeDynamics(samples,active);
    warnings.push('Rigid-body inverse dynamics at joint output; friction, transmission losses, motor rotor inertia and structural flexibility are excluded.');
    warnings.push('Dynamic peaks and time-weighted RMS use sampled loads; refine sample_dt_s to assess convergence.');
  }
  if(summary.task_underactuated)warnings.push('Task dimension exceeds available joint count; structural underactuation is reported separately from posture rank loss.');
  if(trajectory.method==='quintic_rest_to_rest')warnings.push('Every waypoint is a stop (zero velocity and acceleration); jerk may jump at segment boundaries.');
  if(!limits)warnings.push('No velocity/acceleration limits supplied; motion feasibility is incomplete.');
  if(trajectory.derivatives.includes('estimate'))warnings.push('Derivatives estimated by local three-point polynomials on supplied times; noise can amplify.');
  const {samples:rawSamples,...trajectoryInfo}=trajectory;
  return JSON.parse(JSON.stringify({schema_name:'arm-motion-result',schema_version:'1.0',engine_version:ENGINE_VERSION,input_sha256:hash(input),design_model_sha256:hash(input.design_model),input_snapshot:input,normalized_model:model,base_in_world:base,joint_ids:active.map(j=>j.id),joint_names:active.map(j=>j.name),joint_types:active.map(j=>j.type),trajectory:trajectoryInfo,singularity_settings:{...settings,normalization:'translational rows / characteristic length; prismatic columns * characteristic length',rank_absolute_floor:1e-12},summary,samples,events,warnings,dynamics:dynamicsEnabled?{
    method:'recursive_newton_euler',effort_semantics:'inverse_dynamics_joint_output',
    interface_semantics:'parent_on_distal_subtree',interface_frame:'base',
    external_wrench_semantics:'environment_on_robot_at_physical_model_tip_world_axes',
    payload_inertia_frame:'physical_tip_axes_at_payload_COM',
    peak_verification:'samples_only',rms_method:'trapezoidal_time_integral_of_squared_effort',
  }:null,scope:{dynamics_computed:dynamicsEnabled,collision_checked:false,task_tracking_checked:false,near_limit_fraction:.05,tcp_acceleration_method:'J*qdd + directional central difference of J along qd',feasibility:trajectory.constraint_coverage}}));
}
export function report(r){return `# 轨迹状态与奇异性分析\n\n模型：${r.normalized_model.name}\n\n内核：${r.engine_version}；输入哈希：${r.input_sha256}\n\n时长：${r.summary.duration_s} s；样本：${r.summary.sample_count}；约束：${r.summary.constraint_status}\n\n奇异样本：${r.summary.singular_sample_count}；接近奇异：${r.summary.near_singular_sample_count}\n\n${r.scope.dynamics_computed?'## 动态载荷\n\n'+r.summary.dynamic_joints.map(j=>'- '+j.joint_id+' / '+j.effort_unit+' / peak '+j.effort_peak_abs+' / RMS '+j.effort_rms+' / peak time '+j.peak_time_s+' s').join('\n')+'\n\n':''}## 事件\n\n${r.events.map(e=>'- '+JSON.stringify(e)).join('\n')||'无事件'}\n\n## 假设与边界\n\n${r.warnings.map(w=>'- '+w).join('\n')}\n`;}
export function csv(r){const header=['time_s','phase_id','joint_id','position','velocity','acceleration','sigma_min','singularity_status'],rows=[header];for(const s of r.samples)r.joint_ids.forEach((id,j)=>rows.push([s.time_s,s.phase_id,id,s.q[j],s.dq?.[j]??'',s.ddq?.[j]??'',s.singularity.sigma_min,s.singularity.status]));return rows.map(row=>row.map(x=>'"'+String(x).replaceAll('"','""')+'"').join(',')).join('\n')+'\n';}
