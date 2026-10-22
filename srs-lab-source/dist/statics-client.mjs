/** Front-end boundary: NO gravity torque equation or duplicated solver here. */
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
