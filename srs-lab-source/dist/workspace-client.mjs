import {forward,rpy,mv,add} from './workspace-kinematics.mjs';
export {projectionSVG,pointsCSV,reportMarkdown,WORKSPACE_CATEGORIES,pointColor,pointVisible,voxelColor,voxelSection,voxelSectionSVG} from './workspace-exports.mjs';
import {parseRequestText} from './workspace-input-text.mjs';
import {voxelPlan,conservativeReachBound} from './workspace-voxel-grid.mjs';
export function fromWebConfig(config,model,installation,mode='joint_sampling'){
  const vectors=config.linkVectors??[0,0,config.upper,config.fore,0,0,config.tool].map(L=>[L,0,0]);
  const request={schema_version:'1.0',mode,model:{name:config.name,joints:model.axes.map((axis,j)=>({name:'J'+(j+1),type:'revolute',origin:{xyz_m:j===0?[0,0,config.baseHeight]:vectors[j-1].slice(),rpy_rad:[0,0,0]},axis:axis.slice(),limits:config.limits[j].map(x=>x*Math.PI/180)})),tip:{xyz_m:vectors[6].slice(),rpy_rad:[0,0,0]}},base_in_world:{xyz_m:[0,0,0],rpy_rad:installation.request.base_orientation_in_world.rpy_rad.slice()},sections:{center_m:[0,0,config.baseHeight],thickness_m:.03,voxel_size_m:.02}};
  if(mode==='joint_sampling')request.sampling={method:'random',samples:config.samples,seed:config.seed>>>0};
  else if(mode==='dexterity_voxels')request.dexterity={voxel_size_m:.10,orientations_per_voxel:20,seed:config.seed>>>0};
  else request.reachability={sphere:{center_m:[0,0,config.baseHeight],radius_m:config.reach*1.15,radial_steps:3,theta_steps:8,phi_steps:5},orientations_rpy_rad:[[0,0,0],[0,Math.PI/2,0]],orientation_frame:'radial',seed:config.seed>>>0,ik:{attempts:4,max_iterations:120,position_tolerance_m:.001,orientation_tolerance_rad:.02,damping:.03}};
  return request;
}
export function asDexterityInput(input,settings){
  if(input?.schema_name==='arm-analysis-request')return {...input,options:asDexterityInput(input.options,settings)};
  const {sampling,reachability,dexterity,...request}=input;
  const {time_budget_s,...options}=dexterity??{};
  return {...request,mode:'dexterity_voxels',dexterity:{seed:sampling?.seed??reachability?.seed??42,...options,...settings}};
}
// Switch analysis settings while retaining the selected model, installation and provenance.
export function asWorkspaceMode(input,mode,settings){
  if(input?.schema_name==='arm-analysis-request'){
    const adapted=asWorkspaceMode({...input.options,model:input.design_model},mode,settings);
    const {model,...options}=adapted;return {...input,options};
  }
  if(mode==='dexterity_voxels')return asDexterityInput(input,settings);
  if(input.mode===mode)return input;
  const {sampling,reachability,dexterity,...base}=input;
  const seed=sampling?.seed??reachability?.seed??dexterity?.seed??42;
  if(mode==='joint_sampling')return {...base,mode,sampling:{method:'random',samples:5000,seed}};
  if(mode!=='pose_reachability')throw Error('未知工作空间分析方式');
  const bound=conservativeReachBound(input.model);
  return {...base,mode,reachability:{sphere:{center_m:bound.center_base_m,radius_m:bound.radius_m,radial_steps:2,theta_steps:8,phi_steps:5},orientations_rpy_rad:[[0,0,0],[0,Math.PI/2,0]],orientation_frame:'base',seed,ik:{attempts:2,max_iterations:60}}};
}
export function estimateDexterity(request){
  if(request?.schema_name==='arm-analysis-request')request={...request.options,model:request.design_model};
  const plan=voxelPlan(request.model,request.dexterity);
  // Full FR3 web run: 81,040 checks in 189.263 s. Other models and IK settings can differ.
  return {centers:plan.cells.length,checks:plan.checks,seconds:plan.checks*.00234,
    withinLimit:plan.within_limit};
}
export function sectionCornersWorld(result,section){
  const [a,b,normal]=section.axes,{min,max,size,position_m:position}=section;
  const frame=result.normalized_input.base_in_world,R=rpy(frame.rpy_rad);
  return [[min[a],min[b]],[max[a]+1,min[b]],[max[a]+1,max[b]+1],[min[a],max[b]+1]].map(([i,j])=>{
    const corner=[0,0,0];corner[a]=i*size;corner[b]=j*size;corner[normal]=position;
    return add(frame.xyz_m,mv(R,corner));
  });
}
export function formatInput(input){return JSON.stringify(input,null,2);}
export async function runWorkspaceSkill(input,{signal}={}){
  const response=await fetch('/api/workspace',{method:'POST',headers:{'content-type':typeof input==='string'?'application/jsonc':'application/json'},body:typeof input==='string'?input:JSON.stringify(input),signal});
  const payload=await response.json();
  if(!response.ok||!payload.ok)throw Error(`${payload.detail??payload.error??response.status} (${payload.request_id??'-'})`);
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
  const baseFrame={origin_world_m:world([0,0,0]),axes_world:[[1,0,0],[0,1,0],[0,0,1]].map(axis=>mv(R,axis))};
  return {q,nodes_world_m:f.nodes.map(world),position_base_m:f.position_m,position_world_m:world(f.position_m),base_frame:baseFrame};
}
export function parseInput(text){return parseRequestText(text);}
