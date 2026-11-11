import {forward,rpy,mv,add} from './workspace-kinematics.mjs';
export async function runMotion(input,{signal,expected='arm-motion-result'}={}){
  const response=await fetch('/api/motion',{method:'POST',headers:{'content-type':'application/json'},body:typeof input==='string'?input:JSON.stringify(input),signal});const data=await response.json();
  if(!response.ok||!data.ok)throw Error(data.detail??data.error??'轨迹分析失败');
  if(data.execution?.execution_mode!=='skill_cli')throw Error('未确认独立 CLI 来源');
  if(data.result?.schema_name!==expected)throw Error('请选择单条轨迹或粘贴 arm-motion-request JSON 后再分析');
  return data;
}
export function previewMotion(result,index){
  const s=result.samples[index],f=forward(result.normalized_model,s.q),base=result.base_in_world,R=rpy(base.rpy_rad),world=p=>add(base.xyz_m,mv(R,p));
  return {nodes_world_m:f.nodes.map(world),position_world_m:world(f.position_m),base_frame:{origin_world_m:base.xyz_m,axes_world:[[1,0,0],[0,1,0],[0,0,1]].map(a=>mv(R,a))}};
}
