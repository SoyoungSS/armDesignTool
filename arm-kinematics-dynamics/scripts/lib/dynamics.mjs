import {invalid} from '../../../shared/contracts/validate.mjs';
import {validateDesignModel,validateInertia} from '../../../shared/model/design-model.mjs';
import {add,sub,scale,dot,cross,norm,eye,mv,mm,transpose,rpy,rot} from '../../../shared/kinematics/serial.mjs';

const zero=()=>[0,0,0];
const zeroInertia=[[0,0,0],[0,0,0],[0,0,0]];

export function prepareDynamics(model,settings){
  if(!model.gravity_world_m_s2)invalid('design_model.gravity_world_m_s2','explicit gravity required for dynamics');
  validateDesignModel(model,{require:'inertial_properties'});
  const payload=model.payload;
  if(payload?.mass_kg>0&&!settings.payload_inertia_kg_m2){
    invalid('dynamics.payload_inertia_kg_m2','explicit payload COM inertia in physical tip axes required');
  }
  if(settings.payload_inertia_kg_m2){
    if(!payload)invalid('dynamics.payload_inertia_kg_m2','payload inertia requires a declared payload');
    validateInertia(settings.payload_inertia_kg_m2,payload.mass_kg,'dynamics.payload_inertia_kg_m2');
  }
  const tip={
    type:'fixed',origin:model.tip,
    components:payload?[{
      name:'payload',mass_kg:payload.mass_kg,com_m:payload.com_in_tip_m,
      inertia_kg_m2:settings.payload_inertia_kg_m2??zeroInertia,
    }]:[],
  };
  const worldToBase=transpose(rpy(model.base_in_world?.rpy_rad??zero()));
  return {
    joints:[...model.joints,tip],
    gravity:mv(worldToBase,model.gravity_world_m_s2),
    externalForce:mv(worldToBase,settings.external_wrench_world?.force_N??zero()),
    externalMoment:mv(worldToBase,settings.external_wrench_world?.moment_Nm??zero()),
  };
}

// Newton–Euler recursion in base axes. Each body uses its post-joint frame.
export function inverseDynamics(prepared,q,dq,ddq){
  let p=zero(),R=eye(),acceleration=zero(),omega=zero(),alpha=zero(),active=0;
  const bodies=[];
  for(const joint of prepared.joints){
    const offset=mv(R,joint.origin.xyz_m);
    p=add(p,offset);
    acceleration=add(acceleration,add(cross(alpha,offset),cross(omega,cross(omega,offset))));
    R=mm(R,rpy(joint.origin.rpy_rad));
    let axis=null,index=null;
    if(joint.type!=='fixed'){
      index=active++;
      const localAxis=scale(joint.axis,1/norm(joint.axis));
      axis=mv(R,localAxis);
      if(joint.type==='revolute'){
        alpha=add(alpha,add(scale(axis,ddq[index]),cross(omega,scale(axis,dq[index]))));
        omega=add(omega,scale(axis,dq[index]));
        R=mm(R,rot(localAxis,q[index]));
      }else{
        const translation=scale(axis,q[index]);
        acceleration=add(acceleration,add(
          add(cross(alpha,translation),cross(omega,cross(omega,translation))),
          add(scale(axis,ddq[index]),scale(cross(omega,axis),2*dq[index])),
        ));
        p=add(p,translation);
      }
    }
    let force=zero(),moment=zero();
    for(const component of joint.components){
      const offset=mv(R,component.com_m);
      const comAcceleration=add(acceleration,add(cross(alpha,offset),cross(omega,cross(omega,offset))));
      const f=scale(sub(comAcceleration,prepared.gravity),component.mass_kg);
      const inertia=mm(mm(R,component.inertia_kg_m2??zeroInertia),transpose(R));
      const rotational=add(mv(inertia,alpha),cross(omega,mv(inertia,omega)));
      force=add(force,f);
      moment=add(moment,add(rotational,cross(offset,f)));
    }
    bodies.push({p,axis,index,type:joint.type,force,moment});
  }
  const last=bodies.at(-1);
  last.force=sub(last.force,prepared.externalForce);
  last.moment=sub(last.moment,prepared.externalMoment);
  const loads=Array(active);
  let child=null;
  for(let i=bodies.length-1;i>=0;i--){
    const body=bodies[i];
    if(child){
      body.force=add(body.force,child.force);
      body.moment=add(body.moment,add(child.moment,cross(sub(child.p,body.p),child.force)));
    }
    if(body.index!==null){
      const axial=dot(body.force,body.axis);
      const torsion=dot(body.moment,body.axis);
      loads[body.index]={
        effort:body.type==='revolute'?torsion:axial,
        interface_wrench:{frame:'base',point_m:body.p,force_N:body.force,moment_Nm:body.moment},
        axial_force_N:axial,
        radial_force_N:norm(sub(body.force,scale(body.axis,axial))),
        bending_moment_Nm:norm(sub(body.moment,scale(body.axis,torsion))),
      };
    }
    child=body;
  }
  return loads;
}

export function summarizeDynamics(samples,joints){
  const duration=samples.at(-1).time_s-samples[0].time_s;
  return joints.map((joint,index)=>{
    const loads=samples.map(s=>s.dynamic_loads[index]);
    const effort=loads.map(load=>load.effort);
    const peakIndex=effort.reduce((best,value,i)=>Math.abs(value)>Math.abs(effort[best])?i:best,0);
    let squareIntegral=0;
    for(let i=1;i<samples.length;i++){
      squareIntegral+=(samples[i].time_s-samples[i-1].time_s)*(effort[i-1]**2+effort[i]**2)/2;
    }
    return {
      joint_id:joint.id,effort_unit:joint.type==='revolute'?'Nm':'N',
      effort_min:Math.min(...effort),effort_max:Math.max(...effort),
      effort_peak_abs:Math.abs(effort[peakIndex]),peak_time_s:samples[peakIndex].time_s,peak_q:samples[peakIndex].q,
      effort_rms:Math.sqrt(squareIntegral/duration),
      power_peak_abs_W:Math.max(...samples.map((s,i)=>Math.abs(effort[i]*s.dq[index]))),
      axial_force_peak_abs_N:Math.max(...loads.map(l=>Math.abs(l.axial_force_N))),
      radial_force_peak_N:Math.max(...loads.map(l=>l.radial_force_N)),
      bending_moment_peak_Nm:Math.max(...loads.map(l=>l.bending_moment_Nm)),
    };
  });
}

