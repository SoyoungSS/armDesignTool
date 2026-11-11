import {invalid,validateContract} from '../../../shared/contracts/validate.mjs';
import {validateDesignModel} from '../../../shared/model/design-model.mjs';
import {ENGINE_VERSION,hash} from './analyze.mjs';
import {minimumDuration,number,object,position,validateLimits,vector} from './trajectory.mjs';

const settingsBounds={
  seed:[0,2**32-1],
  trajectory_count:[1,10],
  move_count:[1,20],
  max_step_fraction:[.001,1],
  min_speed_fraction:[.05,1],
  max_speed_fraction:[.05,1],
  min_hold_s:[0,60],
  max_hold_s:[0,60],
};

export function generateRandomTrajectories(input){
  const fields=['schema_name','schema_version','template','settings'];
  object(input,'$',fields,fields);
  if(input.schema_name!=='arm-random-motion-request'||input.schema_version!=='1.0'){
    invalid('$','expected arm-random-motion-request 1.0');
  }
  const names=Object.keys(settingsBounds);
  const settings=object(input.settings,'settings',names,names);
  for(const [name,[min,max]] of Object.entries(settingsBounds))number(settings[name],'settings.'+name,min,max);
  for(const name of ['seed','trajectory_count','move_count']){
    if(!Number.isInteger(settings[name]))invalid('settings.'+name,'integer required');
  }
  if(settings.min_speed_fraction>settings.max_speed_fraction)invalid('settings.min_speed_fraction','must not exceed max_speed_fraction');
  if(settings.min_hold_s>settings.max_hold_s)invalid('settings.min_hold_s','must not exceed max_hold_s');

  const template=input.template;
  validateContract('motion-request',template);
  validateDesignModel(template.design_model);
  const joints=template.design_model.joints.filter(joint=>joint.type!=='fixed');
  if(joints.length>16)invalid('template.design_model.joints','motion analysis supports at most 16 active joints');
  const limits=validateLimits(template.motion_limits,joints);
  if(!limits)invalid('template.motion_limits','generation requires explicit per-joint limits');
  const trajectory=template.trajectory;
  const isWaypoint=trajectory.kind==='waypoints';
  const sampleDt=isWaypoint?trajectory.sample_dt_s:.02;
  let start=trajectory.start_q;
  if(!isWaypoint){
    const ids=trajectory.joint_ids;
    if(ids.length!==joints.length||joints.some(joint=>!ids.includes(joint.id))){
      invalid('template.trajectory.joint_ids','must match model active joint IDs');
    }
    const first=vector(trajectory.samples[0].q,'template.trajectory.samples[0].q',ids.length);
    start=joints.map(joint=>first[ids.indexOf(joint.id)]);
  }
  const startQ=vector(start,'template.trajectory.start_q',joints.length);
  position(startQ,joints,'template.trajectory.start_q');
  number(sampleDt,'template.trajectory.sample_dt_s',.0001,1);

  let randomState=settings.seed;
  function uniform(low,high){
    // LCG with explicit unsigned 32-bit state for repeatable output.
    randomState=(Math.imul(randomState,1664525)+1013904223)>>>0;
    return low+(high-low)*(randomState/2**32);
  }

  const trajectories=[];
  for(let index=0;index<settings.trajectory_count;index++){
    let previous=startQ;
    const segments=[];
    for(let move=1;move<=settings.move_count;move++){
      const target=joints.map(({limits:[low,high]},joint)=>{
        const step=(high-low)*settings.max_step_fraction;
        const value=uniform(Math.max(low,previous[joint]-step),Math.min(high,previous[joint]+step));
        return Math.min(high,Math.max(low,value));
      });
      const delta=target.map((value,joint)=>value-previous[joint]);
      const speed=uniform(settings.min_speed_fraction,settings.max_speed_fraction);
      const duration=Math.max(sampleDt,minimumDuration(delta,limits)/speed);
      if(duration>3600)invalid('settings','segment duration exceeds 3600 s; reduce step size or raise the speed range');
      segments.push({id:`move-${move}`,kind:'move',duration_s:duration,q_target:target});
      const hold=uniform(settings.min_hold_s,settings.max_hold_s);
      if(hold>=1e-6)segments.push({id:`hold-${move}`,kind:'hold',duration_s:hold});
      previous=target;
    }
    const sampleBudget=segments.reduce((total,segment)=>total+Math.ceil(segment.duration_s/sampleDt)+4,0);
    if(sampleBudget>20000)invalid('settings','trajectory exceeds 20,000 samples; increase sample_dt_s or reduce segment count/duration');
    const request={...template,trajectory:{kind:'waypoints',start_q:startQ,segments,sample_dt_s:sampleDt,timing_policy:'strict'}};
    const duration_s=segments.reduce((total,segment)=>total+segment.duration_s,0);
    trajectories.push({id:`random-${index+1}`,duration_s,request});
  }
  return {
    schema_name:'arm-random-motion-result',schema_version:'1.0',engine_version:ENGINE_VERSION,
    input_sha256:hash(input),settings,summary:{trajectory_count:trajectories.length},trajectories,
  };
}
