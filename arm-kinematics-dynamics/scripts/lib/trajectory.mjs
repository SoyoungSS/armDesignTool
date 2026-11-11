import {invalid} from '../../../shared/contracts/validate.mjs';
export function object(v,p,allowed,required=[]){if(!v||typeof v!=='object'||Array.isArray(v))invalid(p,'object required');for(const k of Object.keys(v))if(!allowed.includes(k))invalid(p+'.'+k,'unknown field');for(const k of required)if(!Object.hasOwn(v,k))invalid(p+'.'+k,'required');return v;}
export function number(v,p,lo=-1e9,hi=1e9){if(typeof v!=='number'||!Number.isFinite(v)||v<lo||v>hi)invalid(p,`finite number in [${lo}, ${hi}] required`);return v;}
export function vector(v,p,n){if(!Array.isArray(v)||v.length!==n)invalid(p,`expected ${n} ordered joint values`);return v.map((x,i)=>number(x,`${p}[${i}]`));}
const shape=u=>[10*u**3-15*u**4+6*u**5,30*u*u*(1-u)**2,60*u-180*u*u+120*u**3];
export function validateLimits(raw,joints){
  if(raw===undefined)return null;object(raw,'motion_limits',joints.map(j=>j.id),joints.map(j=>j.id));
  return joints.map(j=>{const p='motion_limits.'+j.id,v=object(raw[j.id],p,['velocity_max','acceleration_max','jerk_max'],['velocity_max','acceleration_max']);return Object.fromEntries(Object.entries(v).map(([k,x])=>[k,number(x,p+'.'+k,1e-12,1e9)]));});
}
export function position(q,joints,path){q.forEach((x,i)=>{if(x<joints[i].limits[0]-1e-10||x>joints[i].limits[1]+1e-10)invalid(path+`[${i}]`,'outside joint position limits');});}
export function minimumDuration(delta,limits){
  return Math.max(0,...delta.flatMap((d,j)=>[
    1.875*Math.abs(d)/limits[j].velocity_max,
    Math.sqrt((10/Math.sqrt(3))*Math.abs(d)/limits[j].acceleration_max),
    limits[j].jerk_max?Math.cbrt(60*Math.abs(d)/limits[j].jerk_max):0,
  ]));
}
export function generate(raw,joints,limits){
  object(raw,'trajectory',['kind','start_q','segments','sample_dt_s','timing_policy'],['kind','start_q','segments','sample_dt_s']);
  if(!limits)invalid('motion_limits','generation requires explicit per-joint limits');
  const n=joints.length,dt=number(raw.sample_dt_s,'trajectory.sample_dt_s',.0001,1),policy=raw.timing_policy??'strict';if(!['strict','stretch'].includes(policy))invalid('trajectory.timing_policy','expected strict or stretch');
  let q0=vector(raw.start_q,'trajectory.start_q',n);position(q0,joints,'trajectory.start_q');
  if(!Array.isArray(raw.segments)||!raw.segments.length||raw.segments.length>200)invalid('trajectory.segments','1–200 segments required');
  let time=0;const samples=[],segments=[],events=[],phaseIds=new Set();
  for(const [index,item] of raw.segments.entries()){
    const p=`trajectory.segments[${index}]`;object(item,p,['id','kind','duration_s','q_target'],['id','kind','duration_s']);
    if(typeof item.id!=='string'||!item.id.trim()||phaseIds.has(item.id))invalid(p+'.id','unique nonempty phase ID required');phaseIds.add(item.id);
    if(!['move','hold','wait'].includes(item.kind))invalid(p+'.kind','expected move, hold or wait');
    if(item.kind==='move'&&!item.q_target)invalid(p+'.q_target','required for move');
    const target=item.q_target===undefined?[...q0]:vector(item.q_target,p+'.q_target',n);position(target,joints,p+'.q_target');
    if(item.kind!=='move'&&target.some((x,j)=>x!==q0[j]))invalid(p+'.q_target','hold/wait cannot change joint position');
    const requested=number(item.duration_s,p+'.duration_s',1e-6,3600),delta=target.map((x,j)=>x-q0[j]);
    const minimum=minimumDuration(delta,limits);
    const T=policy==='stretch'?Math.max(requested,minimum):requested;
    if(T>3600)invalid(p+'.duration_s','required duration exceeds 3600 s budget');
    if(T>requested+1e-10)events.push({type:'duration_stretched',phase_id:item.id,time_s:time,requested_s:requested,actual_s:T});
    const peaks=delta.map((d,j)=>({joint_id:joints[j].id,velocity_max:1.875*Math.abs(d)/T,acceleration_max:(10/Math.sqrt(3))*Math.abs(d)/T**2,jerk_max:60*Math.abs(d)/T**3}));
    for(const [j,peak] of peaks.entries())for(const field of ['velocity_max','acceleration_max','jerk_max'])if(limits[j][field]!==undefined&&peak[field]>limits[j][field]*(1+1e-9))events.push({type:'motion_limit_exceeded',phase_id:item.id,joint_id:joints[j].id,quantity:field,actual:peak[field],limit:limits[j][field],start_s:time,end_s:time+T,verification:'analytic_segment_extrema'});
    const count=Math.ceil(T/dt);if(count+samples.length+4>20000)invalid('trajectory','exceeds 20,000 samples; increase sample_dt_s');
    const fractions=Array.from({length:count+1},(_,i)=>i/count);fractions.push(.5,(3-Math.sqrt(3))/6,(3+Math.sqrt(3))/6);
    const us=[...new Set(fractions)].sort((a,b)=>a-b);
    for(const u of us){if(index>0&&u===0)continue;const [s,ds,dds]=shape(u);samples.push({time_s:time+u*T,phase_id:item.id,q:q0.map((v,j)=>v+delta[j]*s),dq:delta.map(d=>d*ds/T),ddq:delta.map(d=>d*dds/T**2)});}
    segments.push({id:item.id,kind:item.kind,start_s:time,end_s:time+T,requested_duration_s:requested,duration_s:T,q_start:q0,q_end:target,peaks});time+=T;q0=target;
  }
  return {samples,segments,events,method:'quintic_rest_to_rest',constraint_coverage:'analytic_segment_extrema',derivatives:'analytic',timing_policy:policy};
}
function derivative(values,times,i){
  const indices=i===0?[0,1,2]:i===times.length-1?[i-2,i-1,i]:[i-1,i,i+1],t=times[i];
  return values[0].map((_,j)=>indices.reduce((sum,k,a)=>{const x=indices[(a+1)%3],y=indices[(a+2)%3];return sum+values[k][j]*(2*t-times[x]-times[y])/((times[k]-times[x])*(times[k]-times[y]));},0));
}
export function importTrajectory(raw,joints,limits){
  object(raw,'trajectory',['kind','joint_ids','samples','derivatives'],['kind','joint_ids','samples']);
  const ids=joints.map(j=>j.id);if(!Array.isArray(raw.joint_ids)||raw.joint_ids.length!==ids.length||new Set(raw.joint_ids).size!==ids.length||ids.some(id=>!raw.joint_ids.includes(id)))invalid('trajectory.joint_ids','must match model active joint IDs');
  const order=ids.map(id=>raw.joint_ids.indexOf(id)),map=(v,p)=>{const a=vector(v,p,ids.length);return order.map(i=>a[i]);};
  if(!Array.isArray(raw.samples)||raw.samples.length<2||raw.samples.length>20000)invalid('trajectory.samples','2–20,000 samples required');
  if(!['none','estimate'].includes(raw.derivatives??'none'))invalid('trajectory.derivatives','expected none or estimate');
  let last=-Infinity;
  const samples=raw.samples.map((s,i)=>{const p=`trajectory.samples[${i}]`;object(s,p,['time_s','phase_id','q','dq','ddq'],['time_s','q']);const t=number(s.time_s,p+'.time_s',0,1e7);if(t<=last)invalid(p+'.time_s','must increase');last=t;if(s.phase_id!==undefined&&(typeof s.phase_id!=='string'||!s.phase_id.trim()))invalid(p+'.phase_id','nonempty string required');return {time_s:t,phase_id:s.phase_id??'imported',q:map(s.q,p+'.q'),dq:s.dq===undefined?null:map(s.dq,p+'.dq'),ddq:s.ddq===undefined?null:map(s.ddq,p+'.ddq')};});
  const count=field=>samples.filter(s=>s[field]!==null).length;
  for(const field of ['dq','ddq'])if(count(field)!==0&&count(field)!==samples.length)invalid('trajectory.samples.'+field,'must be supplied for all samples or none');
  if(count('ddq')&&!count('dq'))invalid('trajectory.samples','ddq requires dq');
  let estimated=false;
  if(raw.derivatives==='estimate'&&!count('ddq')){
    if(samples.length<3)invalid('trajectory.samples','derivative estimation requires at least three samples');
    const times=samples.map(s=>s.time_s);if(!count('dq')){const q=samples.map(s=>s.q);samples.forEach((s,i)=>s.dq=derivative(q,times,i));}
    const dq=samples.map(s=>s.dq);samples.forEach((s,i)=>s.ddq=derivative(dq,times,i));estimated=true;
  }
  const events=[];
  for(const s of samples)for(const [i,j] of joints.entries()){
    if(s.q[i]<j.limits[0]||s.q[i]>j.limits[1])events.push({type:'position_limit_exceeded',joint_id:j.id,time_s:s.time_s,actual:s.q[i],limits:j.limits});
    for(const [field,key] of [['dq','velocity_max'],['ddq','acceleration_max']])if(limits&&s[field]!==null&&Math.abs(s[field][i])>limits[i][key]*(1+1e-9))events.push({type:'motion_limit_exceeded',joint_id:j.id,time_s:s.time_s,quantity:key,actual:Math.abs(s[field][i]),limit:limits[i][key],verification:'samples_only'});
  }
  return {samples,segments:[],events,method:'external_samples',constraint_coverage:'samples_only',derivatives:estimated?'three_point_nonuniform_estimate':count('dq')?(count('ddq')?'provided':'provided_velocity_only'):'unavailable'};
}
