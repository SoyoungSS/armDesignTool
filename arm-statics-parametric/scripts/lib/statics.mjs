/* Shared browser/Node deterministic static-load kernel. No runtime dependencies. */
export const ENGINE_VERSION = '1.0.0';
export class InputError extends Error {
  constructor(path, message) { super(`${path}: ${message}`); this.name='InputError'; this.code='INVALID_INPUT'; this.path=path; }
}
const fail=(p,m)=>{throw new InputError(p,m);};
const isObj=v=>v!==null&&typeof v==='object'&&!Array.isArray(v);
function obj(v,p,keys,required=[]) {
  if(!isObj(v)) fail(p,'must be an object');
  for(const k of Object.keys(v)) if(!keys.includes(k)) fail(`${p}.${k}`,'unknown field');
  for(const k of required) if(!(k in v)) fail(`${p}.${k}`,'required');
  return v;
}
function number(v,p,lo=-Infinity,hi=Infinity) {
  if(typeof v!=='number'||!Number.isFinite(v)||v<lo||v>hi) fail(p,`must be finite number in [${lo}, ${hi}]`);
  return Object.is(v,-0)?0:v;
}
function integer(v,p,lo,hi) {number(v,p,lo,hi);if(!Number.isSafeInteger(v))fail(p,'must be integer');return v;}
function str(v,p) {if(typeof v!=='string'||!v.trim()||v.length>160)fail(p,'must be nonempty string of at most 160 characters');return v;}
function array(v,p,min=0,max=1000) {if(!Array.isArray(v)||v.length<min||v.length>max)fail(p,`must be array of length ${min}..${max}`);return v;}
function vec(v,p) {array(v,p,3,3);return v.map((x,i)=>number(x,`${p}[${i}]`,-1e9,1e9));}
function axis(v,p,warnings) {const a=vec(v,p), n=norm(a);if(n<1e-12)fail(p,'axis cannot be zero');if(Math.abs(n-1)>1e-12)warnings.push(`${p}: axis normalized from length ${n}`);return a.map(x=>x/n);}
function limits(v,p) {array(v,p,2,2);const a=v.map((x,i)=>number(x,`${p}[${i}]`,-1e6,1e6));if(a[0]>=a[1])fail(p,'lower must be less than upper; use fixed joint for a locked axis');return a;}
const add=(a,b)=>a.map((x,i)=>x+b[i]);
const sub=(a,b)=>a.map((x,i)=>x-b[i]);
const scale=(a,s)=>a.map(x=>x*s);
const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const norm=a=>Math.hypot(...a);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const eye=()=>[[1,0,0],[0,1,0],[0,0,1]];
const mv=(R,v)=>R.map(r=>dot(r,v));
const mm=(A,B)=>A.map(r=>[0,1,2].map(j=>r.reduce((s,x,k)=>s+x*B[k][j],0)));
function rot(a,t) {const c=Math.cos(t),s=Math.sin(t),u=1-c;return [
 [c+a[0]*a[0]*u,a[0]*a[1]*u-a[2]*s,a[0]*a[2]*u+a[1]*s],
 [a[1]*a[0]*u+a[2]*s,c+a[1]*a[1]*u,a[1]*a[2]*u-a[0]*s],
 [a[2]*a[0]*u-a[1]*s,a[2]*a[1]*u+a[0]*s,c+a[2]*a[2]*u]];}
function rpy(v) {return mm(mm(rot([0,0,1],v[2]),rot([0,1,0],v[1])),rot([1,0,0],v[0]));}
function transform(v,p,optional=false) {
  if(v===undefined&&optional)return {xyz_m:[0,0,0],rpy_rad:[0,0,0]};
  obj(v,p,['xyz_m','rpy_rad'],['xyz_m','rpy_rad']);return {xyz_m:vec(v.xyz_m,p+'.xyz_m'),rpy_rad:vec(v.rpy_rad,p+'.rpy_rad')};
}
function component(v,p) {obj(v,p,['name','mass_kg','com_m'],['name','mass_kg','com_m']);return {name:str(v.name,p+'.name'),mass_kg:number(v.mass_kg,p+'.mass_kg',0,1e9),com_m:vec(v.com_m,p+'.com_m')};}
function unique(names,p) {if(new Set(names).size!==names.length)fail(p,'duplicate names');}
function serialModel(v,w) {
  obj(v,'model',['name','joints','tip'],['name','joints','tip']);
  const joints=array(v.joints,'model.joints',1,64).map((j,i)=>{
    const p=`model.joints[${i}]`;obj(j,p,['name','type','origin','axis','limits','components'],['name','type','origin','components']);
    if(!['revolute','prismatic','fixed'].includes(j.type))fail(p+'.type','supported: revolute, prismatic, fixed');
    if(j.type==='fixed'&&('axis'in j||'limits'in j))fail(p,'fixed joint must omit axis and limits');
    const out={name:str(j.name,p+'.name'),type:j.type,origin:transform(j.origin,p+'.origin'),components:array(j.components,p+'.components',0,100).map((c,k)=>component(c,`${p}.components[${k}]`))};
    unique(out.components.map(c=>c.name),p+'.components');
    if(j.type!=='fixed'){out.axis=axis(j.axis,p+'.axis',w);out.limits=limits(j.limits,p+'.limits');}
    if(!out.components.length)w.push(`${p}: explicitly massless body`);
    return out;
  });
  unique(joints.map(j=>j.name),'model.joints');
  if(joints.every(j=>j.type==='fixed'))fail('model.joints','at least one active joint required');
  return {name:str(v.name,'model.name'),joints,tip:transform(v.tip,'model.tip')};
}
function simpleModel(v,w) {
  obj(v,'simple',['name','base','segments'],['name','segments']);
  const base=transform(v.base,'simple.base',true);
  if(!v.base)w.push('simple.base: identity installation assumed');
  let prev=[0,0,0];
  const joints=array(v.segments,'simple.segments',1,32).map((s,i)=>{
    const p=`simple.segments[${i}]`;
    obj(s,p,['name','axis','length_m','direction','rpy_rad','limits_rad','link_mass_kg','link_com_m','joint_mass_kg','joint_com_m'],['name','axis','length_m','limits_rad','link_mass_kg']);
    const length=number(s.length_m,p+'.length_m',0,1e6),direction=axis(s.direction??[1,0,0],p+'.direction',w),vector=scale(direction,length);
    const cm=s.link_com_m===undefined?scale(vector,0.5):vec(s.link_com_m,p+'.link_com_m');
    if(s.direction===undefined)w.push(`${p}: link extends along local +X`);
    if(s.link_com_m===undefined)w.push(`${p}: link COM estimated at midpoint`);
    const components=[{name:'link',mass_kg:number(s.link_mass_kg,p+'.link_mass_kg',0,1e9),com_m:cm}];
    if(s.joint_mass_kg!==undefined){
      const mass=number(s.joint_mass_kg,p+'.joint_mass_kg',0,1e9);
      components.push({name:'joint_output_equivalent',mass_kg:mass,com_m:vec(s.joint_com_m??[0,0,0],p+'.joint_com_m')});
      w.push(`${p}: entire joint mass assigned to current output body; verify fixed/moving ownership`);
    }else if(s.joint_com_m!==undefined)fail(p+'.joint_com_m','requires joint_mass_kg');
    const origin=i===0?base:{xyz_m:prev,rpy_rad:s.rpy_rad===undefined?[0,0,0]:vec(s.rpy_rad,p+'.rpy_rad')};
    if(i===0&&s.rpy_rad!==undefined)fail(p+'.rpy_rad','use simple.base.rpy_rad for first installation');
    prev=vector;
    return {name:str(s.name,p+'.name'),type:'revolute',axis:axis(s.axis,p+'.axis',w),limits:limits(s.limits_rad,p+'.limits_rad'),origin,components};
  });
  unique(joints.map(j=>j.name),'simple.segments');
  return {name:str(v.name,'simple.name'),joints,tip:{xyz_m:prev,rpy_rad:[0,0,0]}};
}
function poseMap(v,names,joints,p) {
  obj(v,p,names,names);return names.map(name=>{
    const q=number(v[name],`${p}.${name}`,-1e6,1e6),j=joints.find(x=>x.name===name);
    if(q<j.limits[0]||q>j.limits[1])fail(`${p}.${name}`,`outside joint limits [${j.limits}]`);return q;
  });
}
function rawFinite(v,p='$',seen=new Set(),depth=0) {
  if(depth>64)fail(p,'input nesting exceeds 64 levels');
  if(typeof v==='number'&&!Number.isFinite(v))fail(p,'non-finite number');
  if(typeof v==='undefined'||typeof v==='function'||typeof v==='symbol'||typeof v==='bigint')fail(p,'not JSON data');
  if(v&&typeof v==='object'){
    if(seen.has(v))fail(p,'cyclic input');seen.add(v);
    if(!Array.isArray(v)&&Object.getPrototypeOf(v)!==Object.prototype&&Object.getPrototypeOf(v)!==null)fail(p,'plain JSON objects required');
    for(const [k,x] of Object.entries(v))rawFinite(x,`${p}.${k}`,seen,depth+1);seen.delete(v);
  }
}
export function normalizeInput(input) {
  rawFinite(input);const w=[];
  obj(input,'$',['schema_version','input_mode','model','simple','gravity_m_s2','payload','attachments','wrenches','poses','search','drives','design_factor','metadata'],['schema_version','input_mode','gravity_m_s2','payload']);
  if(input.schema_version!=='1.0')fail('schema_version','expected 1.0');
  if(!['standard','simple'].includes(input.input_mode))fail('input_mode','expected standard or simple');
  if(input.input_mode==='standard'&&('simple'in input)||input.input_mode==='simple'&&('model'in input))fail('input_mode','do not mix model and simple');
  const model=input.input_mode==='standard'?serialModel(input.model,w):simpleModel(input.simple,w);
  const active=model.joints.filter(j=>j.type!=='fixed'),names=active.map(j=>j.name);
  const gravity=vec(input.gravity_m_s2,'gravity_m_s2');if(norm(gravity)===0)w.push('zero gravity supplied explicitly');
  obj(input.payload,'payload',['mass_kg','com_in_tip_m'],['mass_kg','com_in_tip_m']);
  const payload={mass_kg:number(input.payload.mass_kg,'payload.mass_kg',0,1e9),com_in_tip_m:vec(input.payload.com_in_tip_m,'payload.com_in_tip_m')};
  const frames=['tip',...model.joints.map(j=>j.name)];if(names.includes('tip')||model.joints.some(j=>j.name==='tip'))fail('model.joints','tip is reserved frame name');
  const attachments=array(input.attachments??[],'attachments',0,100).map((a,i)=>{
    const p=`attachments[${i}]`;obj(a,p,['name','frame','mass_kg','com_m'],['name','frame','mass_kg','com_m']);if(!frames.includes(a.frame))fail(p+'.frame','unknown frame');
    return {...component({name:a.name,mass_kg:a.mass_kg,com_m:a.com_m},p),frame:a.frame};
  });unique(attachments.map(a=>a.name),'attachments');
  const wrenches=array(input.wrenches??[],'wrenches',0,100).map((a,i)=>{
    const p=`wrenches[${i}]`;obj(a,p,['name','frame','point_m','force_N','moment_Nm'],['name','frame','point_m','force_N','moment_Nm']);
    if(!frames.includes(a.frame))fail(p+'.frame','unknown frame');return {name:str(a.name,p+'.name'),frame:a.frame,point_m:vec(a.point_m,p+'.point_m'),force_N:vec(a.force_N,p+'.force_N'),moment_Nm:vec(a.moment_Nm,p+'.moment_Nm')};
  });unique(wrenches.map(a=>a.name),'wrenches');
  const poses=array(input.poses??[],'poses',0,10000).map((p,i)=>{obj(p,`poses[${i}]`,['name','q'],['name','q']);return {name:str(p.name,`poses[${i}].name`),q:poseMap(p.q,names,active,`poses[${i}].q`)};});unique(poses.map(p=>p.name),'poses');
  let search=null;
  if(input.search!==undefined){
    const s=input.search;obj(s,'search',['samples','seed','elite_count','refine_sweeps'],['samples','seed']);
    search={samples:integer(s.samples,'search.samples',1,200000),seed:integer(s.seed,'search.seed',0,4294967295),elite_count:integer(s.elite_count??3,'search.elite_count',1,10),refine_sweeps:integer(s.refine_sweeps??8,'search.refine_sweeps',0,40)};
    if(names.length>16)fail('search','search supports at most 16 active joints; pose calculations support 64');
  }
  if(!poses.length&&!search)fail('poses','provide at least one pose or search');
  const massCount=model.joints.reduce((total,j)=>total+j.components.length,1)+attachments.length+wrenches.length;
  const evaluationBound=poses.length+(search?search.samples+3+poses.length+names.length*search.elite_count*(1+search.refine_sweeps*names.length*4):0);
  if(evaluationBound*massCount*names.length>50000000)fail('analysis_budget','request exceeds 50 million mass-joint evaluations; reduce samples/refinement/poses');
  const design_factor=number(input.design_factor??1,'design_factor',1,100);
  if(input.design_factor===undefined)w.push('design_factor: 1.0 assumed; no added design allowance');
  const drives=Object.create(null);
  if(input.drives!==undefined){
    obj(input.drives,'drives',names);
    for(const [name,d] of Object.entries(input.drives)){
      const p=`drives.${name}`;obj(d,p,['continuous_output','continuous_conditions','ratio','efficiency'],[]);
      if(active.find(j=>j.name===name).type!=='revolute')fail(p,'drive sizing currently supports revolute joints only');
      const out={};
      if(d.continuous_output!==undefined)out.continuous_output=number(d.continuous_output,p+'.continuous_output',0,1e12);
      if(d.continuous_conditions!==undefined)out.continuous_conditions=str(d.continuous_conditions,p+'.continuous_conditions');
      if(('ratio'in d)!==('efficiency'in d))fail(p,'ratio and efficiency must be supplied together');
      if(d.ratio!==undefined){out.ratio=number(d.ratio,p+'.ratio',1e-9,1e9);out.efficiency=number(d.efficiency,p+'.efficiency',1e-9,1);}
      drives[name]=out;
    }
  }
  if(input.metadata!==undefined&&!isObj(input.metadata))fail('metadata','must be object');
  if(input.metadata?.normalization_assumptions!==undefined){
    array(input.metadata.normalization_assumptions,'metadata.normalization_assumptions',0,500).forEach((s,i)=>{if(typeof s!=='string'||s.length>2048)fail(`metadata.normalization_assumptions[${i}]`,'must be string of at most 2048 characters');w.push(s);});
  }
  w.push('collision_not_checked: evaluated joint configurations may intersect robot or environment');
  w.push('static_only: no inertia, friction, thermal, brake, bearing or structural-strength qualification');
  return {schema_version:'1.0',model,gravity_m_s2:gravity,payload,attachments,wrenches,poses,search,drives,design_factor,warnings:[...new Set(w)]};
}

export function toStandardInput(input) {
  const n=normalizeInput(input),names=n.model.joints.filter(j=>j.type!=='fixed').map(j=>j.name);
  const out={schema_version:'1.0',input_mode:'standard',model:n.model,gravity_m_s2:n.gravity_m_s2,payload:n.payload,attachments:n.attachments,wrenches:n.wrenches,poses:n.poses.map(p=>({name:p.name,q:Object.fromEntries(names.map((name,i)=>[name,p.q[i]]))})),drives:n.drives,design_factor:n.design_factor,metadata:{...input.metadata,normalization_assumptions:n.warnings}};
  if(n.search)out.search=n.search;
  return JSON.parse(canonicalJSON(out));
}

function compute(n,q,detail=true) {
  let R=eye(),p=[0,0,0],k=0;const axes=[],frames=Object.create(null),massItems=[],jointGeometry=[];
  for(const j of n.model.joints){
    p=add(p,mv(R,j.origin.xyz_m));R=mm(R,rpy(j.origin.rpy_rad));
    if(j.type!=='fixed'){
      const a=mv(R,j.axis);axes.push({name:j.name,type:j.type,a,o:p.slice()});
      if(j.type==='revolute')R=mm(R,rot(j.axis,q[k]));else p=add(p,scale(a,q[k]));k++;
      if(detail)jointGeometry.push({name:j.name,origin_m:axes.at(-1).o,axis_in_base:a});
    }
    frames[j.name]={R,p:p.slice(),count:k};
    for(const c of j.components)massItems.push({name:`${j.name}/${c.name}`,category:'robot',mass:c.mass_kg,position:add(p,mv(R,c.com_m)),count:k});
  }
  const tipR=mm(R,rpy(n.model.tip.rpy_rad)),tipP=add(p,mv(R,n.model.tip.xyz_m));frames.tip={R:tipR,p:tipP,count:k};
  massItems.push({name:'payload',category:'payload',mass:n.payload.mass_kg,position:add(tipP,mv(tipR,n.payload.com_in_tip_m)),count:k});
  for(const c of n.attachments){const f=frames[c.frame];massItems.push({name:c.name,category:'attachments',mass:c.mass_kg,position:add(f.p,mv(f.R,c.com_m)),count:f.count});}
  const parts={robot:Array(k).fill(0),payload:Array(k).fill(0),attachments:Array(k).fill(0),external:Array(k).fill(0)};
  let potential=0;const contributions=[];
  function project(force,position,moment,count){return axes.map((j,i)=>i>=count?0:j.type==='revolute'?-dot(j.a,add(cross(sub(position,j.o),force),moment)):-dot(j.a,force));}
  for(const m of massItems){
    potential-=m.mass*dot(n.gravity_m_s2,m.position);
    const v=project(scale(n.gravity_m_s2,m.mass),m.position,[0,0,0],m.count);
    parts[m.category]=add(parts[m.category],v);
    if(detail)contributions.push({name:m.name,category:m.category,mass_kg:m.mass,com_in_base_m:m.position,holding_load:v});
  }
  for(const w of n.wrenches){
    const f=frames[w.frame],v=project(mv(f.R,w.force_N),add(f.p,mv(f.R,w.point_m)),mv(f.R,w.moment_Nm),f.count);
    parts.external=add(parts.external,v);if(detail)contributions.push({name:w.name,category:'external',holding_load:v});
  }
  const holding=Array(k).fill(0).map((_,i)=>parts.robot[i]+parts.payload[i]+parts.attachments[i]+parts.external[i]);
  if(!holding.every(Number.isFinite)||!Number.isFinite(potential))throw new Error('NUMERIC_OVERFLOW');
  if(!detail)return holding;
  return {q:Object.fromEntries(axes.map((j,i)=>[j.name,q[i]])),holding_load:holding,gravity_potential_J:potential,components:parts,contributions,joint_geometry:jointGeometry,tip_pose_in_base:{position_m:tipP,rotation:tipR}};
}
function prng(seed) {let a=seed>>>0;return()=>{a=(a+0x6D2B79F5)>>>0;let t=a;t=Math.imul(t^(t>>>15),t|1);t^=t+Math.imul(t^(t>>>7),t|61);return ((t^(t>>>14))>>>0)/4294967296;};}
function searchRange(n,poseResults) {
  const active=n.model.joints.filter(j=>j.type!=='fixed'),s=n.search,N=active.length,lo=active.map(j=>j.limits[0]),hi=active.map(j=>j.limits[1]),mid=lo.map((v,i)=>(v+hi[i])/2),rng=prng(s.seed);
  const pools=Array.from({length:N},()=>[]);let evaluations=0;
  const consider=(q,tag)=>{
    const tau=compute(n,q,false);evaluations++;
    for(let j=0;j<N;j++){
      const pool=pools[j],value=Math.abs(tau[j]);
      if(pool.length<s.elite_count||value>pool.at(-1).value){
        if(pool.some(x=>x.q.every((v,i)=>v===q[i])))continue;
        pool.push({value,signed:tau[j],q:q.slice(),stage:tag});pool.sort((a,b)=>b.value-a.value);pool.length=Math.min(pool.length,s.elite_count);
      }
    }
    return tau;
  };
  consider(mid,'midpoint');consider(lo,'lower_corner');consider(hi,'upper_corner');
  for(const p of poseResults)consider(active.map(j=>p.q[j.name]),'provided_pose');
  for(let i=0;i<s.samples;i++)consider(lo.map((v,j)=>v+rng()*(hi[j]-v)),'random');
  const samplingMax=pools.map(pool=>pool[0].value);
  // Fixed budgets, fixed coordinate order and deterministic tie handling.
  const starts=pools.map(pool=>pool.map(c=>c.q.slice()));
  for(let target=0;target<N;target++)for(const start of starts[target]){
    let q=start.slice(),best=Math.abs(compute(n,q,false)[target]);evaluations++;
    for(let sweep=0;sweep<s.refine_sweeps;sweep++){
      const factor=0.25*Math.pow(0.5,sweep);
      for(let j=0;j<N;j++){
        const step=(hi[j]-lo[j])*factor;
        for(const value of [lo[j],hi[j],Math.max(lo[j],q[j]-step),Math.min(hi[j],q[j]+step)]){
          const trial=q.slice();trial[j]=value;
          const tau=consider(trial,'local_refinement'),v=Math.abs(tau[target]);
          if(v>best){q=trial;best=v;}
        }
      }
    }
  }
  return {method:'seeded_random_and_fixed_budget_coordinate_refinement',seed:s.seed,requested_random_samples:s.samples,evaluations,sampling_max_abs:samplingMax,extrema:pools.map((pool,i)=>({joint:active[i].name,max_abs:pool[0].value,signed_at_max:pool[0].signed,stage:pool[0].stage,q:Object.fromEntries(active.map((j,k)=>[j.name,pool[0].q[k]]))})),global_optimum_proven:false,collision_checked:false};
}
export function canonicalJSON(v) {
  if(Array.isArray(v))return '['+v.map(canonicalJSON).join(',')+']';
  if(v&&typeof v==='object')return '{'+Object.keys(v).sort().map(k=>JSON.stringify(k)+':'+canonicalJSON(v[k])).join(',')+'}';
  return JSON.stringify(v);
}
// SHA-256 over UTF-8; avoids Node-only imports in the shared browser module.
export function sha256(text) {
  const bytes=Array.from(new TextEncoder().encode(text)),bitLen=bytes.length*8;
  bytes.push(128);while(bytes.length%64!==56)bytes.push(0);
  const high=Math.floor(bitLen/4294967296),low=bitLen>>>0;
  for(const v of [high,low])for(let i=3;i>=0;i--)bytes.push((v>>>(8*i))&255);
  const K=[0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
  const H=[0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19],rr=(x,n)=>(x>>>n)|(x<<(32-n));
  for(let off=0;off<bytes.length;off+=64){
    const W=new Array(64);
    for(let i=0;i<16;i++)W[i]=((bytes[off+4*i]<<24)|(bytes[off+4*i+1]<<16)|(bytes[off+4*i+2]<<8)|bytes[off+4*i+3])>>>0;
    for(let i=16;i<64;i++){const x=W[i-15],y=W[i-2];W[i]=(W[i-16]+(rr(x,7)^rr(x,18)^(x>>>3))+W[i-7]+(rr(y,17)^rr(y,19)^(y>>>10)))>>>0;}
    let [a,b,c,d,e,f,g,h]=H;
    for(let i=0;i<64;i++){const t1=(h+(rr(e,6)^rr(e,11)^rr(e,25))+((e&f)^(~e&g))+K[i]+W[i])>>>0,t2=((rr(a,2)^rr(a,13)^rr(a,22))+((a&b)^(a&c)^(b&c)))>>>0;h=g;g=f;f=e;e=(d+t1)>>>0;d=c;c=b;b=a;a=(t1+t2)>>>0;}
    [a,b,c,d,e,f,g,h].forEach((v,i)=>H[i]=(H[i]+v)>>>0);
  }
  return H.map(v=>v.toString(16).padStart(8,'0')).join('');
}
export function analyze(input) {
  const n=normalizeInput(input),active=n.model.joints.filter(j=>j.type!=='fixed');
  const poses=n.poses.map(p=>({name:p.name,...compute(n,p.q)}));
  const search=n.search?searchRange(n,poses):null;
  const summary=active.map((j,i)=>{
    let max=0,critical=null,signed=0;
    for(const p of poses)if(critical===null||Math.abs(p.holding_load[i])>max){max=Math.abs(p.holding_load[i]);signed=p.holding_load[i];critical={source:'provided_pose',name:p.name,q:p.q};}
    if(search&&(!critical||search.extrema[i].max_abs>max)){const e=search.extrema[i];max=e.max_abs;signed=e.signed_at_max;critical={source:e.stage,q:e.q};}
    const required=max*n.design_factor,drive=n.drives[j.name]??{},available=drive.continuous_output;
    let status='not_sized';
    if(available!==undefined)status=available<required?'exceeds_capacity':drive.continuous_conditions?'satisfies_checked_cases':'capacity_conditions_unknown';
    return {joint:j.name,type:j.type,unit:j.type==='revolute'?'Nm':'N',max_abs_checked:max,signed_at_max:signed,critical_pose:critical,design_factor:n.design_factor,required_output:required,continuous_available:available??null,margin:available===undefined||required===0?null:available/required-1,status,motor_static_torque_Nm:j.type==='revolute'&&drive.ratio!==undefined?required/(drive.ratio*drive.efficiency):null};
  });
  const normalized_input={...n};
  const result={schema_version:'1.0',engine_version:ENGINE_VERSION,input_sha256:sha256(canonicalJSON(input)),normalized_sha256:sha256(canonicalJSON(n)),input_snapshot:JSON.parse(JSON.stringify(input)),normalized_input,joint_names:active.map(j=>j.name),poses,search,summary,scope:{global_optimum_proven:false,collision_checked:false,thermal_checked:false,scope:search?'provided_poses_and_searched_joint_limits':'provided_poses_only'},warnings:n.warnings};
  // Return JSON-native data, including identical signed-zero/prototype behavior
  // for direct calls, Worker structured-clone and serialized HTTP/CLI outputs.
  return JSON.parse(JSON.stringify(result));
}

export function summaryCSV(result) {
  const cols=['joint','type','unit','max_abs_checked','signed_at_max','design_factor','required_output','continuous_available','margin','status','motor_static_torque_Nm'];
  const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
  return cols.join(',')+'\n'+result.summary.map(row=>cols.map(k=>quote(row[k])).join(',')).join('\n')+'\n';
}
export function reportMarkdown(r) {
  const f=v=>v===null?'—':typeof v==='number'?v.toPrecision(9):String(v).replaceAll('|','\\|').replaceAll('\n',' ');
  const statuses={not_sized:'待选型',exceeds_capacity:'超限',satisfies_checked_cases:'已检查工况满足',capacity_conditions_unknown:'连续能力适用条件待确认'};
  let text=`# 机械臂静力学校核报告\n\n模型：${f(r.normalized_input.model.name)}\n\n内核：${r.engine_version}；输入 SHA-256：${r.input_sha256}\n\n`;
  text+='结果用于已检查姿态与搜索范围内的静态驱动初选。未证明全局最大值，未检查碰撞、热保持或结构强度。\n\n';
  text+='|关节|单位|最大绝对载荷|设计系数|输出端需求|连续能力|裕量|判定|\n|---|---|---:|---:|---:|---:|---:|---|\n';
  for(const s of r.summary)text+=`|${f(s.joint)}|${s.unit}|${f(s.max_abs_checked)}|${f(s.design_factor)}|${f(s.required_output)}|${f(s.continuous_available)}|${f(s.margin)}|${statuses[s.status]}|\n`;
  text+='\n裕量 = 连续能力 / 加设计系数后的需求 − 1；需求为零时不计算比值。\n\n## 典型姿态\n\n';
  for(const p of r.poses)text+=`- ${f(p.name)}：${r.joint_names.map((j,i)=>`${f(j)}=${f(p.holding_load[i])}`).join('，')}。\n`;
  text+='\n## 每关节临界姿态\n\n';for(const s of r.summary)text+=`- ${f(s.joint)}：${JSON.stringify(s.critical_pose)}\n`;
  if(r.search)text+=`\n搜索方法：${r.search.method}；种子 ${r.search.seed}；实际计算 ${r.search.evaluations} 次。\n`;
  text+='\n## 输入假设与限制\n\n'+r.warnings.map(w=>'- '+f(w)).join('\n')+'\n';
  return text;
}
