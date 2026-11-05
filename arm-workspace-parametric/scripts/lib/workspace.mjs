import {isDesignRequest,toWorkspaceRequest} from '../../../shared/model/adapters.mjs';
import {createHash} from 'node:crypto';
import {add,sub,scale,dot,norm,cross,eye,mv,mm,rpy,forward,jacobian,orientationError,solveLinear} from './kinematics.mjs';
import {analyzeDexterity} from './dexterity.mjs';
import {conservativeReachBound,voxelPlan} from './voxel-grid.mjs';
export {conservativeReachBound};
export const ENGINE_VERSION='1.2.0';
export class InputError extends Error{constructor(path,message){super(`${path}: ${message}`);this.code='INVALID_INPUT';this.path=path;}}
const fail=(p,m)=>{throw new InputError(p,m);};
function object(v,p,keys,required=[]){if(!v||typeof v!=='object'||Array.isArray(v))fail(p,'expected object');for(const k of Object.keys(v))if(!keys.includes(k))fail(`${p}.${k}`,'unknown field');for(const k of required)if(!Object.hasOwn(v,k))fail(`${p}.${k}`,'required');return v;}
function number(v,p,lo=-1e6,hi=1e6){if(typeof v!=='number'||!Number.isFinite(v)||v<lo||v>hi)fail(p,`finite number required in [${lo},${hi}]`);return v;}
function integer(v,p,lo,hi){number(v,p,lo,hi);if(!Number.isInteger(v))fail(p,'integer required');return v;}
function array(v,p,lo,hi){if(!Array.isArray(v)||v.length<lo||v.length>hi)fail(p,`array length ${lo}..${hi} required`);return v;}
function vector(v,p){return array(v,p,3,3).map((x,i)=>number(x,`${p}[${i}]`));}
function name(v,p){if(typeof v!=='string'||!v.trim()||v.length>160)fail(p,'nonempty name up to 160 characters required');return v;}
function pose(v,p){object(v,p,['xyz_m','rpy_rad'],['xyz_m','rpy_rad']);return {xyz_m:vector(v.xyz_m,p+'.xyz_m'),rpy_rad:vector(v.rpy_rad,p+'.rpy_rad')};}
function jsonData(v,seen=new Set(),depth=0){if(depth>64)fail('$','nesting exceeds 64');if(typeof v==='number'&&!Number.isFinite(v))fail('$','non-finite number');if(['undefined','function','bigint','symbol'].includes(typeof v))fail('$','JSON data required');if(v&&typeof v==='object'){if(seen.has(v))fail('$','cyclic input');if(!Array.isArray(v)&&![Object.prototype,null].includes(Object.getPrototypeOf(v)))fail('$','plain objects required');seen.add(v);for(const x of (Array.isArray(v)?v.values():Object.values(v)))jsonData(x,seen,depth+1);seen.delete(v);}}
export function canonical(v){if(Array.isArray(v))return '['+v.map(canonical).join(',')+']';if(v&&typeof v==='object')return '{'+Object.keys(v).sort().map(k=>JSON.stringify(k)+':'+canonical(v[k])).join(',')+'}';return JSON.stringify(v);}
const hash=v=>createHash('sha256').update(canonical(v)).digest('hex');
const identity=()=>({xyz_m:[0,0,0],rpy_rad:[0,0,0]});
function ikOptions(raw,path,defaults={attempts:4,max_iterations:120}){
  const ik={...defaults,position_tolerance_m:.001,orientation_tolerance_rad:.02,damping:.03};
  if(raw!==undefined){object(raw,path,Object.keys(ik));for(const k of Object.keys(raw))ik[k]=['attempts','max_iterations'].includes(k)?integer(raw[k],path+'.'+k,1,k==='attempts'?12:300):number(raw[k],path+'.'+k,1e-8,k==='damping'?1:Math.PI);}
  return ik;
}
export function normalizeInput(input){
  if(isDesignRequest(input))input=toWorkspaceRequest(input);
  jsonData(input);object(input,'$',['schema_version','mode','model','base_in_world','sampling','reachability','dexterity','sections','metadata'],['schema_version','mode','model']);
  if(input.schema_version!=='1.0')fail('schema_version','expected 1.0');
  const parameters={joint_sampling:'sampling',pose_reachability:'reachability',dexterity_voxels:'dexterity'};
  if(typeof input.mode!=='string'||!Object.hasOwn(parameters,input.mode))fail('mode','expected joint_sampling, pose_reachability or dexterity_voxels');
  for(const field of Object.values(parameters))if(field!==parameters[input.mode]&&Object.hasOwn(input,field))fail(field,'not valid for '+input.mode);
  const warnings=[];object(input.model,'model',['name','joints','tip'],['name','joints','tip']);
  const joints=array(input.model.joints,'model.joints',1,64).map((j,i)=>{
    const p=`model.joints[${i}]`;object(j,p,['name','type','origin','axis','limits'],['name','type','origin']);
    if(!['revolute','prismatic','fixed'].includes(j.type))fail(p+'.type','unsupported joint type');
    const out={name:name(j.name,p+'.name'),type:j.type,origin:pose(j.origin,p+'.origin')};
    if(j.type==='fixed'){if('axis'in j||'limits'in j)fail(p,'fixed joint must omit axis/limits');}
    else{const a=vector(j.axis,p+'.axis'),length=norm(a);if(length<1e-12)fail(p+'.axis','nonzero axis required');if(Math.abs(length-1)>1e-12)warnings.push(p+': axis normalized');out.axis=scale(a,1/length);const limits=array(j.limits,p+'.limits',2,2).map((x,k)=>number(x,p+`.limits[${k}]`));if(limits[0]>=limits[1])fail(p+'.limits','lower must be less than upper');out.limits=limits;}
    return out;
  });
  if(new Set(joints.map(j=>j.name)).size!==joints.length)fail('model.joints','duplicate joint name');
  const active=joints.filter(j=>j.type!=='fixed');if(!active.length)fail('model.joints','at least one active joint required');
  const model={name:name(input.model.name,'model.name'),joints,tip:pose(input.model.tip,'model.tip')};
  const base_in_world=input.base_in_world===undefined?identity():pose(input.base_in_world,'base_in_world');
  const sections={center_m:[0,0,0],thickness_m:.03,voxel_size_m:.02};
  if(input.sections!==undefined){object(input.sections,'sections',Object.keys(sections));for(const k of Object.keys(input.sections))sections[k]=k==='center_m'?vector(input.sections[k],'sections.center_m'):number(input.sections[k],'sections.'+k,1e-6,1e3);}
  let sampling=null,reachability=null,dexterity=null;
  if(input.mode==='joint_sampling'){
    const s=input.sampling;object(s,'sampling',['method','samples','seed','steps_per_joint','fixed_joints'],['method']);
    if(!['random','grid'].includes(s.method))fail('sampling.method','expected random or grid');
    const fixed_joints={};if(s.fixed_joints!==undefined){object(s.fixed_joints,'sampling.fixed_joints',active.map(j=>j.name));for(const [key,v]of Object.entries(s.fixed_joints)){const j=active.find(j=>j.name===key);Object.defineProperty(fixed_joints,key,{value:number(v,'sampling.fixed_joints.'+key,...j.limits),enumerable:true});}}
    sampling={method:s.method,seed:integer(s.seed??42,'sampling.seed',0,4294967295),fixed_joints};
    if(s.method==='random'){if('steps_per_joint'in s)fail('sampling.steps_per_joint','grid only');sampling.samples=integer(s.samples,'sampling.samples',1,50000);}
    else{if('samples'in s)fail('sampling.samples','grid count is determined by steps_per_joint');sampling.steps_per_joint=integer(s.steps_per_joint,'sampling.steps_per_joint',2,100);sampling.samples=sampling.steps_per_joint**active.filter(j=>!Object.hasOwn(fixed_joints,j.name)).length;}
    if(sampling.samples>50000||sampling.samples*joints.length>2000000)fail('sampling','grid/sample budget exceeded; reduce samples or grid steps');
  }else if(input.mode==='pose_reachability'){
    if(active.length>16)fail('model.joints','IK supports at most 16 active joints');
    const r=input.reachability;object(r,'reachability',['points_m','sphere','orientations_rpy_rad','orientation_frame','ik','seed']);
    if(('points_m'in r)===('sphere'in r))fail('reachability','provide exactly one of points_m or sphere');
    let points_m=null,sphere=null;
    if(r.points_m!==undefined)points_m=array(r.points_m,'reachability.points_m',1,2000).map((p,i)=>vector(p,`reachability.points_m[${i}]`));
    else{const s=r.sphere;object(s,'reachability.sphere',['center_m','radius_m','radial_steps','theta_steps','phi_steps'],['center_m','radius_m','radial_steps','theta_steps','phi_steps']);sphere={center_m:vector(s.center_m,'reachability.sphere.center_m'),radius_m:number(s.radius_m,'reachability.sphere.radius_m',1e-6,1000),radial_steps:integer(s.radial_steps,'reachability.sphere.radial_steps',1,32),theta_steps:integer(s.theta_steps,'reachability.sphere.theta_steps',3,64),phi_steps:integer(s.phi_steps,'reachability.sphere.phi_steps',3,64)};}
    const orientations_rpy_rad=r.orientations_rpy_rad===undefined?null:array(r.orientations_rpy_rad,'reachability.orientations_rpy_rad',1,64).map((v,i)=>vector(v,`reachability.orientations_rpy_rad[${i}]`));
    const orientation_frame=r.orientation_frame??'base';if(!['base','radial'].includes(orientation_frame))fail('reachability.orientation_frame','expected base or radial');
    if(orientation_frame==='radial'&&!sphere)fail('reachability.orientation_frame','radial orientation requires sphere');
    const ik=ikOptions(r.ik,'reachability.ik');
    reachability={points_m,sphere,orientations_rpy_rad,orientation_frame,ik,seed:integer(r.seed??42,'reachability.seed',0,4294967295)};
    // Position feasibility is checked separately from orientation feasibility.
    const count=points_m?.length??spherePoints(sphere).length,queries=count*(orientations_rpy_rad?orientations_rpy_rad.length+1:1);
    if(count>2000||queries*ik.attempts*ik.max_iterations*joints.length>20000000)fail('reachability','IK budget exceeds 20 million joint-iterations; reduce resolution, orientations or attempts');
    if(orientations_rpy_rad&&distinctRotations(orientations_rpy_rad)<2)warnings.push('Dexterity not evaluated: at least two distinct sampled orientations are required.');
    warnings.push('IK failure means no solution found within budget; it does not prove unreachability.');
    if(orientation_frame==='radial')warnings.push('At sphere center the radial reference is undefined; identity orientation is used.');
  }else{
    if(active.length>16)fail('model.joints','IK supports at most 16 active joints');
    const d=input.dexterity===undefined?{}:input.dexterity;
    object(d,'dexterity',['voxel_size_m','orientations_per_voxel','seed','time_budget_s','bounds_base_m','ik']);
    let region=null;
    if(d.bounds_base_m!==undefined){
      const b=object(d.bounds_base_m,'dexterity.bounds_base_m',['min','max'],['min','max']);
      region={min:vector(b.min,'dexterity.bounds_base_m.min'),max:vector(b.max,'dexterity.bounds_base_m.max')};
      if(region.min.some((v,i)=>v>=region.max[i]))fail('dexterity.bounds_base_m','each min must be less than max');
    }
    dexterity={voxel_size_m:number(d.voxel_size_m??.10,'dexterity.voxel_size_m',.001,1000),
      orientations_per_voxel:integer(d.orientations_per_voxel??20,'dexterity.orientations_per_voxel',2,100),
      seed:integer(d.seed??42,'dexterity.seed',0,4294967295),
      bounds_base_m:region,ik:ikOptions(d.ik,'dexterity.ik',{attempts:6,max_iterations:180})};
    sections.voxel_size_m=dexterity.voxel_size_m;
    sections.thickness_m=dexterity.voxel_size_m;
    if(input.sections?.center_m===undefined)sections.center_m=joints[0].origin.xyz_m.slice();
    warnings.push('Voxel scores measure the tested orientation set at each cell center. Zero solutions do not prove unreachability.');
  }
  if(input.metadata!==undefined&&(!input.metadata||Array.isArray(input.metadata)||typeof input.metadata!=='object'))fail('metadata','object required');
  if(input.metadata?.design_model&&Array.isArray(input.metadata.normalization_assumptions))warnings.push(...input.metadata.normalization_assumptions);
  warnings.push('No collision, self-collision, load/torque, thermal or dual-arm coordination checks.','Samples do not certify a complete workspace boundary or continuous orientation coverage.');
  return {schema_version:'1.0',mode:input.mode,model,base_in_world,sampling,reachability,sections,warnings,...(dexterity?{dexterity}:{})};
}

export function spherePoints(s){
  const points=[];
  for(let layer=0;layer<s.radial_steps;layer++){
    const t=s.radial_steps===1?1:layer/(s.radial_steps-1),radius=s.radius_m*t;
    if(radius===0){points.push(s.center_m.slice());continue;}
    const nt=Math.max(3,Math.round(s.theta_steps*t)),np=Math.max(3,Math.round(s.phi_steps*t));
    // Equal-area latitude bands; poles occur once, unlike duplicated azimuth poles.
    for(let k=0;k<np;k++){const z=-1+2*k/(np-1),rho=Math.sqrt(Math.max(0,1-z*z)),count=(k===0||k===np-1)?1:nt;for(let j=0;j<count;j++){const theta=2*Math.PI*j/count;points.push(add(s.center_m,scale([rho*Math.cos(theta),rho*Math.sin(theta),z],radius)));}}
  }
  return points;
}
function rng(seed){let state=seed>>>0;return()=>{state=(1664525*state+1013904223)>>>0;return state/4294967296;};}
function radialFrame(point,center){const d=sub(center,point);if(norm(d)<1e-12)return eye();const z=scale(d,1/norm(d)),x0=cross(Math.abs(z[2])<.9?[0,0,1]:[0,1,0],z),x=scale(x0,1/norm(x0)),y=cross(z,x);return [0,1,2].map(i=>[x[i],y[i],z[i]]);}
export function solveIK(n,position,targetRotation,random,initialQ){
  const active=n.model.joints.filter(j=>j.type!=='fixed'),options=(n.reachability??n.dexterity).ik;
  let best={position_error_m:Infinity,orientation_error_rad:targetRotation?Infinity:null};
  const measure=q=>{const f=forward(n.model,q),e=sub(position,f.position_m),o=targetRotation?orientationError(targetRotation,f.rotation):[],pe=norm(e),oe=targetRotation?norm(o):null;return {f,error:[...e,...o],position_error_m:pe,orientation_error_rad:oe,score:pe*pe+(oe??0)**2};};
  for(let attempt=initialQ?-1:0;attempt<options.attempts;attempt++){
    let q=active.map(j=>attempt<0?initialQ[j.name]:attempt===0?Math.max(j.limits[0],Math.min(j.limits[1],0)):j.limits[0]+random()*(j.limits[1]-j.limits[0]));
    for(let iteration=0;iteration<options.max_iterations;iteration++){
      const m=measure(q);
      if(m.position_error_m<=options.position_tolerance_m&&(!targetRotation||m.orientation_error_rad<=options.orientation_tolerance_rad))return {success:true,q:Object.fromEntries(active.map((j,i)=>[j.name,q[i]])),position_error_m:m.position_error_m,orientation_error_rad:m.orientation_error_rad};
      if(m.position_error_m<best.position_error_m)best={position_error_m:m.position_error_m,orientation_error_rad:m.orientation_error_rad};
      const J=jacobian(m.f,!!targetRotation),D=m.error.length,A=Array.from({length:D},(_,i)=>Array.from({length:D},(_,j)=>J.reduce((s,col)=>s+col[i]*col[j],0)+(i===j?options.damping**2:0))),u=solveLinear(A,m.error);
      if(!u)break;
      const step=J.map(col=>dot(col,u));let accepted=false;
      for(const factor of [1,.5,.25,.1]){
        const candidate=q.map((v,i)=>Math.max(active[i].limits[0],Math.min(active[i].limits[1],v+factor*Math.max(-.25,Math.min(.25,step[i])))));
        if(measure(candidate).score<m.score){q=candidate;accepted=true;break;}
      }
      if(!accepted)break;
      // Check the final candidate too, including max_iterations=1.
      const end=measure(q);if(end.position_error_m<=options.position_tolerance_m&&(!targetRotation||end.orientation_error_rad<=options.orientation_tolerance_rad))return {success:true,q:Object.fromEntries(active.map((j,i)=>[j.name,q[i]])),position_error_m:end.position_error_m,orientation_error_rad:end.orientation_error_rad};
    }
  }
  return {success:false,q:null,...best};
}
function bounds(points){if(!points.length)return null;return {min:[0,1,2].map(k=>Math.min(...points.map(p=>p[k]))),max:[0,1,2].map(k=>Math.max(...points.map(p=>p[k])))};}
function distinctRotations(angles){
  if(!angles)return 0;const unique=[];
  for(const a of angles){const R=rpy(a);if(!unique.some(U=>R.every((row,i)=>row.every((v,j)=>Math.abs(v-U[i][j])<1e-8))))unique.push(R);}
  return unique.length;
}
export function analyze(input){
  const n=normalizeInput(input),active=n.model.joints.filter(j=>j.type!=='fixed'),points=[],R=rpy(n.base_in_world.rpy_rad),world=p=>add(n.base_in_world.xyz_m,mv(R,p));
  let numericalIKCalls=0,dexterityDetail=null;
  const reachBound=conservativeReachBound(n.model),orientationCount=n.dexterity?.orientations_per_voxel??distinctRotations(n.reachability?.orientations_rpy_rad);
  if(n.mode==='joint_sampling'){
    const s=n.sampling,random=rng(s.seed);
    for(let i=0;i<s.samples;i++){
      let index=i;const q=active.map(j=>{
        if(Object.hasOwn(s.fixed_joints,j.name))return s.fixed_joints[j.name];
        if(s.method==='grid'){const digit=index%s.steps_per_joint;index=Math.floor(index/s.steps_per_joint);return j.limits[0]+digit*(j.limits[1]-j.limits[0])/(s.steps_per_joint-1);}
        return i===0?Math.max(j.limits[0],Math.min(j.limits[1],0)):j.limits[0]+random()*(j.limits[1]-j.limits[0]);
      });
      const f=forward(n.model,q);points.push({id:i,position_base_m:f.position_m,position_world_m:world(f.position_m),q:Object.fromEntries(active.map((j,k)=>[j.name,q[k]])),reachability:null,spatial_status:'reachable',dexterity_status:'not_tested',category:'spatial_reachable',unreachable_reason:null});
    }
  }else if(n.mode==='pose_reachability'){
    const r=n.reachability,targets=r.points_m??spherePoints(r.sphere),random=rng(r.seed);
    targets.forEach((p,i)=>{
      const orientations=r.orientations_rpy_rad?.map(angles=>r.orientation_frame==='radial'?mm(radialFrame(p,r.sphere.center_m),rpy(angles)):rpy(angles))??[null];
      const outside=norm(sub(p,reachBound.center_base_m))>reachBound.radius_m+r.ik.position_tolerance_m+1e-9;
      let positionSolution=null;const solutions=[];
      if(!outside){
        numericalIKCalls++;const position=solveIK(n,p,null,random);if(position.success)positionSolution=position;
        if(r.orientations_rpy_rad){orientations.forEach((rotation,k)=>{numericalIKCalls++;const result=solveIK(n,p,rotation,random);if(result.success)solutions.push({orientation_index:k,...result});});}
        else if(positionSolution)solutions.push({orientation_index:0,...positionSolution});
      }
      // An orientation-constrained solution also witnesses position feasibility.
      if(!positionSolution&&solutions.length)positionSolution={...solutions[0],orientation_error_rad:null};
      const spatial=outside?'unreachable_proven':positionSolution?'reachable':'unresolved';
      const dexterity=!r.orientations_rpy_rad?'not_tested':orientationCount<2?'insufficient_orientation_samples':solutions.length===orientations.length?'all_sampled_solved':solutions.length?'partial_sampled_solved':'no_sampled_solution';
      const category=outside?'unreachable':spatial==='unresolved'?'unresolved':dexterity==='all_sampled_solved'?'sampled_dexterous':dexterity==='partial_sampled_solved'?'partially_dexterous':'spatial_reachable';
      points.push({id:i,position_base_m:p,position_world_m:world(p),reachability:solutions.length/orientations.length,orientation_successes:solutions.length,orientation_total:orientations.length,distinct_orientation_count:orientationCount,q:solutions[0]?.q??positionSolution?.q??null,solutions,position_solution:positionSolution,spatial_status:spatial,dexterity_status:dexterity,category,unreachable_reason:outside?'outside_conservative_reach_bound':null});
    });
  }else{
    let plan;
    try{plan=voxelPlan(n.model,n.dexterity);}
    catch(e){fail('dexterity.voxel_size_m',e.message);}
    if(!plan.within_limit)fail('dexterity','exceeds 50,000 centers or 1,000,000 planned pose checks; coarsen the grid or restrict bounds_base_m');
    const random=rng(n.dexterity.seed);
    const scan=analyzeDexterity({options:n.dexterity,cells:plan.cells,world,orientationRandom:rng((n.dexterity.seed^0x9e3779b9)>>>0),
      solve:(position,rotation,initialQ)=>solveIK(n,position,rotation,random,initialQ)});
    points.push(...scan.points);numericalIKCalls=scan.calls;dexterityDetail=scan.detail;
  }
  const reachable=points.filter(p=>p.spatial_status==='reachable'),positions=reachable.map(p=>p.position_base_m),worldPositions=reachable.map(p=>p.position_world_m),size=n.sections.voxel_size_m;
  const voxels=new Set(positions.map(p=>p.map(x=>Math.floor(x/size)).join(',')));
  const sections=Object.fromEntries([['xy',2],['xz',1],['yz',0]].map(([plane,k])=>[plane,points.filter(p=>Math.abs(p.position_base_m[k]-n.sections.center_m[k])<=n.sections.thickness_m/2).map(p=>p.id)]));
  const successCount=n.mode==='joint_sampling'?null:points.reduce((s,p)=>s+p.orientation_successes,0);
  const total=dexterityDetail?numericalIKCalls:n.mode==='pose_reachability'?points.reduce((s,p)=>s+p.orientation_total,0):null;
  const summary={point_count:points.length,reachable_point_count:reachable.length,ik_query_count:total,ik_success_count:successCount,sampled_orientation_success_rate:total?successCount/total:null,bounds_base_m:bounds(positions),bounds_world_m:bounds(worldPositions),occupied_voxels:voxels.size,sampled_voxel_volume_m3:voxels.size*size**3};
  summary.position_check_count=n.mode==='pose_reachability'?points.length:null;
  summary.numerical_ik_solve_count=n.mode==='joint_sampling'?null:numericalIKCalls;
  summary.workspace_counts=Object.fromEntries(['spatial_reachable','sampled_dexterous','partially_dexterous','unreachable','unresolved'].map(category=>[category,points.filter(p=>p.category===category).length]));
  if(dexterityDetail)summary.workspace_counts.untested=dexterityDetail.incomplete_voxels;
  summary.spatial_reachable_point_count=reachable.length;
  summary.sampled_dexterous_point_count=orientationCount>=2?summary.workspace_counts.sampled_dexterous:null;
  summary.proven_unreachable_point_count=summary.workspace_counts.unreachable;
  summary.unresolved_point_count=summary.workspace_counts.unresolved;
  summary.distinct_orientation_count=orientationCount;
  summary.sampled_dexterous_bounds_base_m=bounds(points.filter(p=>p.category==='sampled_dexterous').map(p=>p.position_base_m));
  const result={schema_version:'1.0',engine_version:ENGINE_VERSION,mode:n.mode,input_sha256:hash(input),model_sha256:hash({model:n.model,base_in_world:n.base_in_world}),input_snapshot:input,normalized_input:n,joint_names:active.map(j=>j.name),points,summary,sections,geometric_reach_bound:reachBound,scope:{collision_checked:false,load_checked:false,complete_boundary_proven:false,orientation_checked:!!(n.reachability?.orientations_rpy_rad||dexterityDetail),dexterous_workspace_proven:false,dexterity_evaluated:orientationCount>=2,ik_failure_proves_unreachable:false},warnings:n.warnings,...(dexterityDetail?{dexterity:dexterityDetail}:{})};
  if(isDesignRequest(input)){result.design_model_sha256=hash(input.design_model);result.design_model_info=toWorkspaceRequest(input).metadata.design_model;}
  return JSON.parse(JSON.stringify(result));
}
