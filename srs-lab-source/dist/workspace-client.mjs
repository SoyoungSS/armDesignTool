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
  return {...base,mode,reachability:{points_m:[bound.center_base_m.map((v,k)=>v+(k===2?bound.radius_m*.5:0))],orientations_rpy_rad:[[0,0,0],[0,Math.PI/2,0]],orientation_frame:'base',seed,ik:{attempts:2,max_iterations:60}}};
}
const controlEscape=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
export function workspaceModeControls(mode,busy=false){
  const disabled=busy?'disabled':'';
  return `<label for="workspaceMode">分析功能</label><select id="workspaceMode" ${disabled}><option value="joint_sampling" ${mode==='joint_sampling'?'selected':''}>快速工作范围（FK 点云）</option><option value="ik" ${mode!=='joint_sampling'?'selected':''}>空间可达与灵巧分析（IK）</option></select>${mode==='joint_sampling'?'':`<label for="workspaceIKMode">分析方式</label><select id="workspaceIKMode" ${disabled}><option value="dexterity_voxels" ${mode==='dexterity_voxels'?'selected':''}>区域扫描（默认）</option><option value="pose_reachability" ${mode==='pose_reachability'?'selected':''}>指定任务点</option></select>`}`;
}
export function workspaceIKControls(input,mode,busy=false){
  if(mode==='joint_sampling')return '';
  const options=input?.schema_name==='arm-analysis-request'?input.options:input;
  const setting=(mode==='dexterity_voxels'?options?.dexterity:options?.reachability)??{};
  const disabled=busy?'disabled':'';
  const rows=values=>(values??[]).map(row=>row.join(', ')).join('\n');
  const body=mode==='dexterity_voxels'?`<p>扫描区域以基座坐标系 B 表达；边界留空时使用模型几何外界内的区域。</p><div class="form-grid"><label>区域最小 XYZ / m<input id="workspaceBoundsMin" value="${controlEscape(setting.bounds_base_m?.min?.join(', '))}" placeholder="例如 -0.5, -0.5, 0"></label><label>区域最大 XYZ / m<input id="workspaceBoundsMax" value="${controlEscape(setting.bounds_base_m?.max?.join(', '))}" placeholder="例如 0.5, 0.5, 0.8"></label></div>`:
    `<div class="form-grid"><label>目标位置 XYZ / m（基座系 B，每行一个点）<textarea id="workspaceTaskPoints" rows="4" placeholder="0.3, 0, 0.5">${controlEscape(rows(setting.points_m))}</textarea></label><label>末端朝向 RPY / °（每行一个；留空只检查位置）<textarea id="workspaceTaskOrientations" rows="4">${controlEscape(rows(setting.orientations_rpy_rad?.map(row=>row.map(v=>v*180/Math.PI))))}</textarea></label></div>${setting.sphere?'<p>已导入球形目标定义；位置列表留空将沿用该定义。朝向参考系沿用高级输入的 orientation_frame；填写位置列表后使用基座系。</p>':''}`;
  return `<fieldset ${disabled}>${body}<details><summary>求解设置（可选）</summary><label>随机种子<input id="workspaceIKSeed" type="number" min="0" step="1" value="${setting.seed??42}"></label><label>IK 求解参数 JSON<textarea id="workspaceIKOptions" rows="4">${controlEscape(JSON.stringify(setting.ik??{},null,2))}</textarea></label><p>未指定项使用内核默认值。未找到解不等于不可达；不检查碰撞与负载。</p></details></fieldset>`;
}
export function applyWorkspaceIKControls(input,fields){
  const result=structuredClone(input),options=result.schema_name==='arm-analysis-request'?result.options:result;
  const vectors=(text,label)=>{
    let rows;
    try{rows=text.trim().startsWith('[')?JSON.parse(text):text.trim().split(/\n+/).map(line=>line.trim().split(/[,，\s]+/).map(token=>token?Number(token):NaN));}catch{throw Error(label+'格式无效');}
    if(Array.isArray(rows)&&rows.length===3&&rows.every(v=>typeof v==='number'))rows=[rows];
    if(!Array.isArray(rows)||!rows.length||rows.some(row=>!Array.isArray(row)||row.length!==3||row.some(v=>typeof v!=='number'||!Number.isFinite(v))))throw Error(label+'每行须填写三个有限数值');
    return rows;
  };
  const setting=options.mode==='dexterity_voxels'?options.dexterity:options.reachability;
  if(options.mode==='dexterity_voxels'){
    const min=fields.min?.trim(),max=fields.max?.trim();
    if(!!min!==!!max)throw Error('区域最小和最大 XYZ 须同时填写，或同时留空');
    if(min){const lo=vectors(min,'区域最小值'),hi=vectors(max,'区域最大值');if(lo.length!==1||hi.length!==1||lo[0].some((v,i)=>v>=hi[0][i]))throw Error('每个轴的区域最小值须小于最大值');setting.bounds_base_m={min:lo[0],max:hi[0]};}
    else delete setting.bounds_base_m;
  }else{
    if(fields.points?.trim()){setting.points_m=vectors(fields.points,'目标位置');delete setting.sphere;setting.orientation_frame='base';}
    else if(!setting.sphere)throw Error('请至少填写一个目标位置');
    if(fields.orientations?.trim())setting.orientations_rpy_rad=vectors(fields.orientations,'末端朝向').map(row=>row.map(v=>v*Math.PI/180));
    else delete setting.orientations_rpy_rad;
  }
  if(fields.ik!==undefined){try{setting.ik=JSON.parse(fields.ik||'{}');}catch{throw Error('IK 求解参数须为有效 JSON');}if(!setting.ik||Array.isArray(setting.ik)||typeof setting.ik!=='object')throw Error('IK 求解参数须为 JSON 对象');}
  if(fields.seed!==undefined){const seed=Number(fields.seed);if(!String(fields.seed).trim()||!Number.isInteger(seed)||seed<0||seed>4294967295)throw Error('随机种子须为 0–4294967295 的整数');setting.seed=seed;}
  return result;
}
export function estimateDexterity(request){
  if(request?.schema_name==='arm-analysis-request')request={...request.options,model:request.design_model};
  const plan=voxelPlan(request.model,request.dexterity);
  // Full FR3 web run: 81,040 checks in 189.263 s. Other models and IK settings can differ.
  return {centers:plan.cells.length,checks:plan.checks+plan.cells.length,seconds:(plan.checks+plan.cells.length)*.00234,
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
