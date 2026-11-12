/** Front-end boundary: NO gravity torque equation or duplicated solver here. */
export async function applyInstallationToText(text, installation) {
  const {parseRequestText}=await import('./skill-input-text.mjs');
  const input=parseRequestText(text);
  if(!input || typeof input!=='object' || Array.isArray(input))throw new Error('Skill 输入必须是 JSON 对象');
  if(input.schema_name==='arm-analysis-request'){
    if(!input.design_model || typeof input.design_model!=='object')throw new Error('公共请求缺少 design_model');
    input.design_model.gravity_world_m_s2=structuredClone(installation.gravity_world_m_s2);
    input.design_model.base_in_world={xyz_m:input.design_model.base_in_world?.xyz_m??[0,0,0],rpy_rad:structuredClone(installation.base_orientation_in_world.rpy_rad)};
    return JSON.stringify(input,null,2);
  }
  delete input.gravity_m_s2;
  input.installation=structuredClone(installation);
  return JSON.stringify(input,null,2);
}
export async function runStaticsSkill(input, {endpoint='/api/statics', signal} = {}) {
  const isText = typeof input === 'string';
  const response = await fetch(endpoint, {
    method:'POST', headers:{'content-type':isText?'application/jsonc':'application/json'},
    body:isText ? input : JSON.stringify(input), signal
  });
  const payload = await response.json();
  if (!response.ok || !payload.ok) throw new Error(`静力学 Skill 失败：${payload.error || response.status} (${payload.request_id || '-'})`);
  if (payload.execution?.execution_mode !== 'skill_cli') throw new Error('未验证 Skill 执行来源，拒绝展示静力学结果');
  return payload;
}
export function makeStaticsViewModel({result, execution}) {
  if (!Array.isArray(result?.summary) || !Array.isArray(result?.joint_names)) throw new TypeError('非法 Skill 结果');
  const joints = result.summary.map((r,i) => ({
    name: r.name ?? r.joint_name ?? result.joint_names[i],
    effortUnit: r.unit ?? 'Nm',
    maxTorqueNm: r.max_abs_checked ?? null,
    requiredNm: r.required_output ?? null,
    capacityNm: r.continuous_available ?? null,
    margin: r.margin ?? null,
    status: r.status ?? 'unknown',
    criticalPose: r.critical_pose ?? null
  }));
  return {joints, poses: result.poses ?? [], warnings: result.warnings ?? [],
    engineVersion: result.engine_version ?? execution.engine_version ?? 'unknown',
    inputSha256: result.input_sha256 ?? execution.input_sha256 ?? null,
    runId:execution.request_id, executedAt:execution.executed_at};
}
export function poseToJointAngles(pose, jointNames) {
  const q = pose?.q ?? pose?.critical_pose?.q;
  if (!q) throw new Error('Skill 未提供姿态角度');
  return jointNames.map((name,i) => {
    const angle = Array.isArray(q) ? q[i] : q[name];
    if (!Number.isFinite(angle)) throw new Error(`关节 ${name} 缺少有效角度`);
    return angle; // radians. Apply directly to 3D model; never re-solve statics here.
  });
}

export async function createCriticalPoseReplay(result, jointName) {
  const {forward,rpy,mv,add}=await import('./workspace-kinematics.mjs');
  const {model,installation}=result.normalized_input;
  const summary=result.summary.find(item=>item.joint===jointName);
  const initial=poseToJointAngles(summary?.critical_pose,result.joint_names);
  const joints=model.joints.filter(joint=>joint.type!=='fixed');
  const origin=result.input_snapshot?.design_model?.base_in_world?.xyz_m??[0,0,0];
  const rotation=rpy(installation?.base_orientation_in_world?.rpy_rad??[0,0,0]);
  const world=p=>add(origin,mv(rotation,p));
  const basis=[[1,0,0],[0,1,0],[0,0,1]];
  const base_frame={origin_world_m:origin,axes_world:basis.map(axis=>mv(rotation,axis))};
  function preview(q=initial,selected=jointName) {
    const fk=forward(model,q);
    const active=fk.joints.find(joint=>joint.name===selected);
    return {
      base_frame,
      nodes_world_m:fk.nodes.map(world),
      position_world_m:world(fk.position_m),
      tip_axes_world:basis.map(axis=>mv(rotation,mv(fk.rotation,axis))),
      joints_world:fk.joints.map(joint=>({
        name:joint.name,origin:world(joint.origin),axis:mv(rotation,joint.axis),selected:joint.name===selected,
      })),
      selected_joint:active?{name:selected,origin_world_m:world(active.origin)}:null,
    };
  }
  return {initial,joints,preview};
}

export async function previewCriticalPose(result,jointName) {
  const replay=await createCriticalPoseReplay(result,jointName);
  return replay.preview();
}
