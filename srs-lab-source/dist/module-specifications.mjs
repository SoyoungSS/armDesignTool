function peakDuration(samples,index,peak){
  const threshold=.9*Math.abs(samples[peak].effort[index]);
  const magnitude=i=>Math.abs(samples[i].effort[index]);
  const crossing=(a,b)=>samples[a].time_s+(samples[b].time_s-samples[a].time_s)
    *(threshold-magnitude(a))/(magnitude(b)-magnitude(a));
  let left=peak,right=peak;
  while(left>0&&magnitude(left-1)>=threshold)left--;
  while(right<samples.length-1&&magnitude(right+1)>=threshold)right++;
  const start=left===0?samples[left].time_s:crossing(left-1,left);
  const end=right===samples.length-1?samples[right].time_s:crossing(right,right+1);
  return {threshold,start_s:start,end_s:end,duration_s:end-start};
}

export function combineResults(model,modelHash,results){
  for(const [skill,result] of Object.entries(results)){
    if(result.design_model_sha256!==modelHash)throw Error(skill+' 返回的设计模型与公共输入不一致');
  }
  const {statics,workspace,motion}=results;
  const joints=model.joints.filter(joint=>joint.type!=='fixed');
  if(!motion.scope.dynamics_computed)throw Error('请启用动态载荷计算后重新运行轨迹 Skill');
  if(motion.summary.constraint_status==='violated')throw Error('轨迹存在关节约束违规，请修正后重新计算');
  const rows=joints.map((joint,index)=>{
    const holding=statics.summary.find(row=>row.joint===joint.name);
    const load=motion.summary.dynamic_joints.find(row=>row.joint_id===joint.id);
    const position=motion.summary.joint_peaks.find(row=>row.joint_id===joint.id);
    if(!holding||!load||!position||motion.joint_ids[index]!==joint.id)throw Error('关节结果不完整：'+joint.name);
    const speedFactor=joint.type==='revolute'?60/(2*Math.PI):1;
    const peakIndex=motion.samples.findIndex(sample=>sample.time_s===load.peak_time_s);
    const fastest=motion.samples.reduce((best,sample)=>Math.abs(sample.dq[index])>Math.abs(best.dq[index])?sample:best);
    const pair=sample=>({time_s:sample.time_s,phase_id:sample.phase_id,q:sample.q,effort:sample.effort[index],speed:sample.dq[index]*speedFactor});
    return {
      joint_id:joint.id,name:joint.name,type:joint.type,
      range:joint.type==='revolute'?joint.limits.map(x=>x*180/Math.PI):joint.limits,
      range_unit:joint.type==='revolute'?'°':'m',speed_unit:joint.type==='revolute'?'rpm':'m/s',
      effort_unit:load.effort_unit,body_mass_kg:joint.components.reduce((sum,c)=>sum+c.mass_kg,0),
      holding_effort_abs:holding.required_output,holding_pose:holding.critical_pose,
      dynamic_peak_abs:load.effort_peak_abs,dynamic_rms:load.effort_rms,
      checked_effort_abs:Math.max(holding.required_output,load.effort_peak_abs),
      speed_peak_abs:position.speed_max*speedFactor,
      effort_peak_point:pair(motion.samples[peakIndex]),speed_peak_point:pair(fastest),
      effort_peak_90pct_window:peakDuration(motion.samples,index,peakIndex),
      axial_force_peak_abs_N:load.axial_force_peak_abs_N,radial_force_peak_N:load.radial_force_peak_N,
      bending_moment_peak_Nm:load.bending_moment_peak_Nm,
    };
  });
  const worksheetLayout=rows.length===7&&rows.every((row,i)=>row.name==='J'+(i+1)&&row.type==='revolute');
  const groups=worksheetLayout?[rows.slice(0,1),rows.slice(1,4),rows.slice(4)].map(group=>({
    joints:group.map(joint=>joint.name),
    holding_effort_abs:Math.max(...group.map(joint=>joint.holding_effort_abs)),
    dynamic_peak_abs:Math.max(...group.map(joint=>joint.dynamic_peak_abs)),
    checked_effort_abs:Math.max(...group.map(joint=>joint.checked_effort_abs)),
    speed_peak_abs:Math.max(...group.map(joint=>joint.speed_peak_abs)),
  })):[];
  return {
    schema_name:'arm-module-requirements',schema_version:'1.0',model_name:model.name,design_model_sha256:modelHash,
    sources:Object.fromEntries(Object.entries(results).map(([key,value])=>[key,{input_sha256:value.input_sha256,engine_version:value.engine_version}])),
    workspace:{mode:workspace.mode,sample_count:workspace.summary.point_count,bounds_base_m:workspace.summary.bounds_base_m},
    cycle:{duration_s:motion.summary.duration_s,sample_count:motion.summary.sample_count,constraint_status:motion.summary.constraint_status},
    assumptions:[
      '静态保持列为本次检查的输出需求，包含静力学请求中的设计系数。',
      '动态需求和最高速度来自本次轨迹。更换任务节拍后需重新计算。',
      ...(motion.input_snapshot.trajectory.segments?.map(s=>s.id).join(',')==='out,hold,return'
        ?['默认工况为中位往返，附加负载质心惯量采用半径 50 mm 的均匀实心球。']:[]),
      '工作空间范围取本次分析所得可达位置。模组连续热能力、碰撞、轴承组合承载与寿命需进一步校核。',
      ...motion.warnings,
    ],
    joints:rows,groups,
  };
}
