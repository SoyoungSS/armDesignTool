import {schemas} from './schemas.mjs';
export class ContractError extends Error {
  constructor(path,message){super(`${path}: ${message}`);this.name='ContractError';this.code='INVALID_INPUT';this.path=path;}
}
export const invalid=(path,message)=>{throw new ContractError(path,message);};
export function assertJSON(value,path='$',seen=new Set(),depth=0){
  if(depth>64)invalid(path,'nesting exceeds 64');
  if(value===null||typeof value==='string'||typeof value==='boolean')return;
  if(typeof value==='number'){if(!Number.isFinite(value))invalid(path,'finite number required');return;}
  if(typeof value!=='object')invalid(path,'JSON value required');
  if(seen.has(value))invalid(path,'cyclic input');
  if(!Array.isArray(value)&&![Object.prototype,null].includes(Object.getPrototypeOf(value)))invalid(path,'plain JSON object required');
  seen.add(value);for(const [key,item] of Object.entries(value))assertJSON(item,`${path}.${key}`,seen,depth+1);seen.delete(value);
}
function check(v,s,p){
  if(s.anyOf){if(!s.anyOf.some(option=>{try{check(v,option,p);return true;}catch{return false;}}))invalid(p,'does not match allowed types');return;}
  if('const'in s&&v!==s.const)invalid(p,`expected ${s.const}`);
  if(s.enum&&!s.enum.includes(v))invalid(p,'unsupported value');
  if(s.type){const types=[s.type].flat();if(!types.some(t=>t==='null'?v===null:t==='array'?Array.isArray(v):t==='object'?v!==null&&typeof v==='object'&&!Array.isArray(v):t==='integer'?Number.isSafeInteger(v):typeof v===t))invalid(p,`expected ${types.join(' or ')}`);}
  if(typeof v==='number'){if(s.minimum!==undefined&&v<s.minimum||s.maximum!==undefined&&v>s.maximum||s.exclusiveMinimum!==undefined&&v<=s.exclusiveMinimum)invalid(p,'number outside allowed range');}
  if(typeof v==='string'){if(v.length<(s.minLength??0)||v.length>(s.maxLength??Infinity)||s.minLength&&!v.trim())invalid(p,'invalid string length');if(s.pattern&&!new RegExp(s.pattern).test(v))invalid(p,'invalid string format');}
  if(Array.isArray(v)){if(v.length<(s.minItems??0)||v.length>(s.maxItems??Infinity))invalid(p,'invalid array length');if(s.items)v.forEach((x,i)=>check(x,s.items,`${p}[${i}]`));}
  if(v!==null&&typeof v==='object'&&!Array.isArray(v)){
    for(const key of s.required??[])if(!Object.hasOwn(v,key))invalid(`${p}.${key}`,'required');
    for(const [key,x] of Object.entries(v)){if(s.properties?.[key])check(x,s.properties[key],`${p}.${key}`);else if(s.additionalProperties===false)invalid(`${p}.${key}`,'unknown field');}
  }
}
export function validateContract(name,value){
  assertJSON(value);if(!schemas[name])throw Error('Unknown contract: '+name);check(value,schemas[name],'$');
  if(name==='trajectory'){
    if(new Set(value.joint_ids).size!==value.joint_ids.length)invalid('$.joint_ids','duplicate ID');
    if(value.joint_types.length!==value.joint_ids.length)invalid('$.joint_types','joint count mismatch');
    let last=-Infinity;for(const [i,s] of value.samples.entries()){if(s.time_s<=last)invalid(`$.samples[${i}].time_s`,'must increase');last=s.time_s;for(const field of ['q','dq','ddq'])if(s[field].length!==value.joint_ids.length)invalid(`$.samples[${i}].${field}`,'joint count mismatch');}
  }
  if(name==='joint-trace'){
    if(new Set(value.joint_ids).size!==value.joint_ids.length)invalid('$.joint_ids','duplicate ID');
    const times=new Map();for(const [i,s] of value.samples.entries()){if(!value.joint_ids.includes(s.joint_id))invalid(`$.samples[${i}].joint_id`,'unknown joint');if(s.time_s<=(times.get(s.joint_id)??-Infinity))invalid(`$.samples[${i}].time_s`,'must increase per joint');times.set(s.joint_id,s.time_s);if((s.effort===null)!==(s.effort_semantics==='unavailable'))invalid(`$.samples[${i}].effort`,'missing effort requires unavailable semantics');}
    if(times.size!==value.joint_ids.length)invalid('$.samples','missing joint records');
  }
  if(name==='case'&&new Set(value.phases.map(p=>p.id)).size!==value.phases.length)invalid('$.phases','duplicate phase ID');
  return value;
}
