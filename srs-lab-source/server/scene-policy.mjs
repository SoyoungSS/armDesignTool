export const isConcept=input=>input.assumption_policy==='concept';
export function conceptPolicy(input,env=process.env,previous=null){
  return {mode:input.assumption_policy,auto_run:input.auto_run,
    defaults:{cycles:input.cycles??previous?.cycles??1,max_time_s:input.max_time_s??Math.min(previous?.max_time_s??10,Number(env.ARM_SCENE_MAX_SIM_SECONDS||120)),period_s:5,settling_s:1,seed:42,randomization:'none',max_range_fraction:.1,velocity_fraction:.25},
    authorization:'概念试算请求委托 Agent 按常规补齐未指定参数并直接运行；估算值不是用户实测或逐项确认值。用户明确给出的约束优先，来源与实际假设必须记录。'};
}
export function completeConceptPlan(plan,input,model,env=process.env){
  const policy=conceptPolicy(input,env);
  plan.follow_up_questions=[...new Set([...(plan.follow_up_questions??[]),...plan.missing_inputs])];
  plan.blocking_issues=plan.blocking_issues??[];
  plan.missing_inputs=[...plan.blocking_issues];
  if(!Number.isInteger(plan.cycles)||plan.cycles<1){plan.cycles=policy.defaults.cycles;plan.assumptions.push(`轮数未明确，按概念试算采用 ${plan.cycles} 个代表循环。`);}
  if(typeof plan.max_time_s!=='number'||!Number.isFinite(plan.max_time_s)||plan.max_time_s<=0){plan.max_time_s=policy.defaults.max_time_s;plan.assumptions.push(`最大试算时长未明确，采用 ${plan.max_time_s} s；达到时限不等于任务成功。`);}
  if(!plan.strategy.trim()){plan.strategy='沿用模型驱动与限幅，选择低速、行程内、平滑反馈工况，优先生成代表性载荷。';plan.assumptions.push('未指定控制策略，委托 Agent 采用最简单可实现的概念策略。');}
  if(!plan.phases.length){
    plan.simulation_scope='load_proxy';
    plan.phases=[{name:'representative_cycle',action:'按原描述构建限位内的低速代表循环并记录载荷',completion:'实际完成定义的往返/保持工况且跟踪误差达标',on_failure:'停止并保留实际数据与诊断'}];
    plan.assumptions.push('信息不足以精细还原原任务，先运行明确标记的代表性载荷代理。');
  }
  if(!plan.success_criteria.length)plan.success_criteria=['用实际物理状态核对代表循环完成、限位/碰撞及跟踪误差；0.02 rad 或 1 mm 为初步误差阈值，具体工况另行记录。'];
  if(!plan.physics.length)plan.physics=['已有模型质量、惯量、负载和驱动优先；缺项由 Agent 按概念策略补齐并逐项记录，避免重复计重。'];
  if(!model)plan.missing_inputs.push('请选择或生成可读取的完整 MuJoCo 模型。');
  plan.ready_to_run=plan.missing_inputs.length===0;
  return plan;
}
