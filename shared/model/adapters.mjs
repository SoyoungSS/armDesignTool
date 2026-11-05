import {validateContract,invalid} from '../contracts/validate.mjs';
import {validateDesignModel,geometryModel} from './design-model.mjs';
export const isDesignRequest=input=>input?.schema_name==='arm-analysis-request';
const allowedStatics=['poses','search','attachments','wrenches','drives','design_factor'];
const allowedWorkspace=['mode','sampling','reachability','dexterity','sections'];
function prepare(request,required,allowed){
  validateContract('analysis-request',request);
  for(const key of Object.keys(request.options))if(!allowed.includes(key))invalid('$.options.'+key,'unknown or model-owned field');
  const audit=validateDesignModel(request.design_model,{require:required});
  const metadata={design_model:{schema_name:request.design_model.schema_name,schema_version:request.design_model.schema_version,provenance:structuredClone(request.design_model.provenance),capabilities:audit.capabilities,joint_ids:request.design_model.joints.map(j=>({id:j.id,name:j.name})),assumptions:audit.assumptions},normalization_assumptions:audit.assumptions};
  return {m:request.design_model,metadata};
}
export function toStaticsRequest(request){
  const {m,metadata}=prepare(request,'mass_properties',allowedStatics);
  if(!m.gravity_world_m_s2)invalid('$.design_model.gravity_world_m_s2','explicit gravity required for statics');
  if(!m.payload)invalid('$.design_model.payload','explicit payload required; use mass_kg: 0 for no payload');
  const model=geometryModel(m);model.joints.forEach((j,i)=>{j.components=m.joints[i].components.map(c=>({name:c.name,mass_kg:c.mass_kg,com_m:[...c.com_m]}));});
  return {schema_version:'1.0',input_mode:'standard',model,installation:{gravity_world_m_s2:[...m.gravity_world_m_s2],base_orientation_in_world:{rpy_rad:[...(m.base_in_world?.rpy_rad??[0,0,0])]}},payload:{mass_kg:m.payload.mass_kg,com_in_tip_m:[...m.payload.com_in_tip_m]},...structuredClone(request.options),metadata};
}
export function toWorkspaceRequest(request){
  const {m,metadata}=prepare(request,'geometry',allowedWorkspace);
  return {schema_version:'1.0',model:geometryModel(m),...(m.base_in_world?{base_in_world:structuredClone(m.base_in_world)}:{}),...structuredClone(request.options),metadata};
}
