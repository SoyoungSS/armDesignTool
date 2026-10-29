import {forward,rpy,mv,add} from './workspace-kinematics.mjs';
export {projectionSVG,pointsCSV,reportMarkdown,WORKSPACE_CATEGORIES,pointColor,pointVisible} from './workspace-exports.mjs';
import {parseRequestText} from './workspace-input-text.mjs';
export function fromWebConfig(config,model,installation,mode='joint_sampling'){
  const vectors=config.linkVectors??[0,0,config.upper,config.fore,0,0,config.tool].map(L=>[L,0,0]);
  const request={schema_version:'1.0',mode,model:{name:config.name,joints:model.axes.map((axis,j)=>({name:'J'+(j+1),type:'revolute',origin:{xyz_m:j===0?[0,0,config.baseHeight]:vectors[j-1].slice(),rpy_rad:[0,0,0]},axis:axis.slice(),limits:config.limits[j].map(x=>x*Math.PI/180)})),tip:{xyz_m:vectors[6].slice(),rpy_rad:[0,0,0]}},base_in_world:{xyz_m:[0,0,0],rpy_rad:installation.request.base_orientation_in_world.rpy_rad.slice()},sections:{center_m:[0,0,config.baseHeight],thickness_m:.03,voxel_size_m:.02}};
  if(mode==='joint_sampling')request.sampling={method:'random',samples:config.samples,seed:config.seed>>>0};
  else request.reachability={sphere:{center_m:[0,0,config.baseHeight],radius_m:config.reach*1.15,radial_steps:3,theta_steps:8,phi_steps:5},orientations_rpy_rad:[[0,0,0],[0,Math.PI/2,0]],orientation_frame:'radial',seed:config.seed>>>0,ik:{attempts:4,max_iterations:120,position_tolerance_m:.001,orientation_tolerance_rad:.02,damping:.03}};
  return request;
}
export function formatInput(input){return JSON.stringify(input,null,2);}
export async function runWorkspaceSkill(input,{signal}={}){
  const response=await fetch('/api/workspace',{method:'POST',headers:{'content-type':typeof input==='string'?'application/jsonc':'application/json'},body:typeof input==='string'?input:JSON.stringify(input),signal});
  const payload=await response.json();
  if(!response.ok||!payload.ok)throw Error(`${payload.error??response.status} (${payload.request_id??'-'})`);
  if(payload.execution?.execution_mode!=='skill_cli'||payload.execution?.skill_name!=='arm-workspace-parametric')throw Error('工作空间结果缺少独立 Skill 执行来源');
  if(!Array.isArray(payload.result?.points)||!payload.result?.summary||!payload.result?.normalized_input?.model)throw Error('工作空间结果格式不正确');
  return payload;
}
// Preview uses the SAME geometry-only FK file as the Skill; no sampling or IK in the page.
export function previewPose(result,pointIndex,solutionIndex=0){
  const point=result.points[pointIndex];if(!point)throw Error('采样点不存在');
  const q=point.solutions?.length?point.solutions[solutionIndex]?.q:point.position_solution?.q??point.q;if(!q)throw Error('此点没有已找到的关节解');
  const n=result.normalized_input,active=n.model.joints.filter(j=>j.type!=='fixed');
  const angles=active.map(j=>{const v=q[j.name];if(!Number.isFinite(v)||v<j.limits[0]-1e-9||v>j.limits[1]+1e-9)throw Error('关节解缺失或超出限位');return v;});
  const f=forward(n.model,angles),R=rpy(n.base_in_world.rpy_rad),world=p=>add(n.base_in_world.xyz_m,mv(R,p));
  return {q,nodes_world_m:f.nodes.map(world),position_base_m:f.position_m,position_world_m:world(f.position_m)};
}
export function parseInput(text){return parseRequestText(text);}
