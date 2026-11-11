import {invalid} from '../../shared/contracts/validate.mjs';
import {validateDesignModel} from '../../shared/model/design-model.mjs';

const sphereInertia=(mass,radius)=>{
  const moment=.4*mass*radius**2;
  return [[moment,0,0],[0,moment,0],[0,0,moment]];
};

export function prepareRequests(model){
  validateDesignModel(model,{require:'mass_properties'});
  const design_model=structuredClone(model);
  const joints=design_model.joints.filter(joint=>joint.type!=='fixed');
  if(joints.length>16)invalid('design_model.joints','规格汇总最多支持 16 个主动关节');
  design_model.gravity_world_m_s2??=[0,0,-9.8];
  design_model.payload??={mass_kg:0,com_in_tip_m:[0,0,0],provenance:{source:'规格与模型',assumptions:['未指定附加负载，按空载计算。']}};
  for(const joint of design_model.joints){
    for(const component of joint.components){
      if(component.mass_kg>0&&!component.inertia_kg_m2){
        component.inertia_kg_m2=sphereInertia(component.mass_kg,.04);
        component.provenance.assumptions.push('缺少质心惯量，按半径 40 mm 的均匀实心球估算。');
      }
    }
  }
  validateDesignModel(design_model,{require:'inertial_properties'});
  const center=joints.map(joint=>joint.limits[0]+(joint.limits[1]-joint.limits[0])/2);
  const target=joints.map((joint,i)=>center[i]+(i%2?-1:1)*.15*(joint.limits[1]-joint.limits[0]));
  const zero=joints.map(({limits:[lo,hi]})=>Math.min(hi,Math.max(lo,0)));
  const pose=(name,angles)=>({name,q:Object.fromEntries(joints.map((joint,i)=>[joint.name,angles[i]]))});
  const common={schema_name:'arm-analysis-request',schema_version:'1.0',design_model};
  const payload=design_model.payload;
  return {
    statics:{...common,options:{poses:[pose('zero_within_limits',zero),pose('cycle_start',center),pose('cycle_target',target)],search:{samples:2048,seed:42},design_factor:1}},
    workspace:{...common,options:{mode:'joint_sampling',sampling:{method:'random',samples:1000,seed:42}}},
    motion:{
      schema_name:'arm-motion-request',schema_version:'1.0',design_model,
      motion_limits:Object.fromEntries(joints.map(joint=>[joint.id,joint.type==='revolute'
        ?{velocity_max:1,acceleration_max:2}:{velocity_max:.2,acceleration_max:.5}])),
      dynamics:{enabled:true,...(payload.mass_kg>0?{payload_inertia_kg_m2:sphereInertia(payload.mass_kg,.05)}:{})},
      trajectory:{kind:'waypoints',start_q:center,sample_dt_s:.02,timing_policy:'stretch',segments:[
        {id:'out',kind:'move',duration_s:2,q_target:target},
        {id:'hold',kind:'hold',duration_s:1},
        {id:'return',kind:'move',duration_s:2,q_target:center},
      ]},
    },
  };
}


