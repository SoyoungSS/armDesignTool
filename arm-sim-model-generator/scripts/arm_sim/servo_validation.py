"""Observable acceptance metrics; never silently tunes gains to pass a test."""
import math
import numpy as np
from .drives import mass_matrix_and_gravity, sample_poses

def check_dynamics(n,m,mujoco):
    d=mujoco.MjData(m); M=np.zeros((m.nv,m.nv)); count=0
    for q in sample_poses(n):
        mujoco.mj_resetData(m,d)
        for name,value in q.items(): d.qpos[m.joint(name).qposadr[0]]=value
        mujoco.mj_forward(m,d); mujoco.mj_fullM(m,M,d.qM)
        expected,gravity=mass_matrix_and_gravity(n,q)
        np.testing.assert_allclose(M,expected,rtol=2e-7,atol=1e-9)
        np.testing.assert_allclose(d.qfrc_bias,gravity,rtol=2e-7,atol=1e-8); count+=1
    return count

def check_servos(n,m,mujoco):
    joints=[j for j in n['model']['joints'] if j['type']!='fixed']
    driven=[j for j in joints if n['drives'][j['name']]['mode']=='position']
    if not driven: return {'status':'not_applicable','reason':'no position servos; finite short step checked separately'}
    qadr=[m.joint(j['name']).qposadr[0] for j in driven]; dadr=[m.joint(j['name']).dofadr[0] for j in driven]
    aids=[m.actuator(j['name']+'_drive').id for j in driven]
    extent=np.array([.002 if j['type']=='revolute' else .0002 for j in driven])
    speed=np.array([.01 if j['type']=='revolute' else .001 for j in driven])
    force=np.array([n['drives'][j['name']]['effort'] for j in driven])
    velocity=np.array([n['drives'][j['name']]['velocity'] for j in driven])
    configs=[('hold_initial',n['poses']['initial'],False)]
    configs += [('hold_'+p['name'],p['q'],False) for p in n['poses']['acceptance'][:2]]
    configs += [('smooth_move',n['poses']['initial'],True)]
    # A finite cap prevents pathological tiny timesteps from hanging validation.
    budget=int(sum(9 if move else 4 for _,_,move in configs)/m.opt.timestep)
    if budget>500000: return {'status':'pending','reason':'servo validation exceeds 500000-step budget; no false pass'}
    original_flags=m.opt.disableflags
    def trial(label,q,move,contacts):
        d=mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m,d,0)
        for j in joints:
            value=q[j['name']]; d.qpos[m.joint(j['name']).qposadr[0]]=value
            cfg=n['drives'][j['name']]
            if cfg['mode']=='position':
                aid=m.actuator(j['name']+'_drive').id; d.ctrl[aid]=value
                adr=m.actuator_actadr[aid]
                if adr>=0: d.act[adr]=value
        start=np.array([q[j['name']] for j in driven]); target=start.copy()
        if move:
            for i,j in enumerate(driven):
                step=min(.05 if j['type']=='revolute' else .005,.05*(j['limits'][1]-j['limits'][0]))
                direction=1 if start[i]<sum(j['limits'])/2 else -1
                target[i]=float(np.clip(start[i]+direction*step,*j['limits']))
        duration=max(1.,float(np.max(1.875*np.abs(target-start)/(.25*velocity)))) if move else 0
        total=duration+4.; tail=[]; speeds=[]; saturated=[]; maxspeed=np.zeros(len(driven)); maxcontact=0; worstpenetration=0.
        for k in range(int(math.ceil(total/m.opt.timestep))):
            if move:
                u=min(1.,k*m.opt.timestep/duration); blend=u**3*(10-15*u+6*u*u); d.ctrl[aids]=start+(target-start)*blend
            mujoco.mj_step(m,d)
            if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)) or np.any(d.warning.number): raise ValueError(label+': nonfinite state or MuJoCo warning')
            maxspeed=np.maximum(maxspeed,np.abs(d.qvel[dadr])); maxcontact=max(maxcontact,d.ncon)
            if d.ncon: worstpenetration=min(worstpenetration,float(min(c.dist for c in d.contact)))
            if d.time>total-.5:
                tail.append(d.qpos[qadr].copy()); speeds.append(np.abs(d.qvel[dadr])); saturated.append(np.abs(d.actuator_force[aids])>=.99*force)
        tail=np.array(tail); peak=np.ptp(tail,axis=0); tailspeed=np.max(speeds,axis=0); saturation=np.mean(saturated,axis=0)
        stable=(peak<=extent)&(tailspeed<=speed)
        return {'case':label,'contacts_enabled':contacts,'seconds':float(d.time),'joint_names':[j['name'] for j in driven],
                'tail_peak_to_peak':peak.tolist(),'tail_max_speed':tailspeed.tolist(),'steady_error':(target-tail.mean(0)).tolist(),
                'tail_saturation_fraction':saturation.tolist(),'max_speed':maxspeed.tolist(),'velocity_exceeded':(maxspeed>1.05*velocity).tolist(),
                'max_contacts':maxcontact,'max_penetration_m':-worstpenetration,'settled':bool(np.all(stable)),
                'capacity_not_saturated':bool(np.all(saturation<.01)),
                'scope':'finite pose tests; static gravity deflection is reported, not compensated'}
    try:
        results=[trial(label,q,move,True) for label,q,move in configs]
        isolated=[]
        if any(r['max_contacts'] for r in results):
            m.opt.disableflags=original_flags|int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
            isolated=[trial(label,q,move,False) for label,q,move in configs]
    finally: m.opt.disableflags=original_flags
    # Contact-on results decide acceptance. Diagnostic contact-off runs never hide failures.
    passed=all(r['settled'] and r['capacity_not_saturated'] and not any(r['velocity_exceeded']) for r in results)
    return {'status':'passed' if passed else 'needs_review','criteria':{'tail_window_s':.5,'peak_to_peak_rad_m':[.002,.0002],'max_tail_speed_rad_m_s':[.01,.001],'max_saturation_fraction':.01},'cases':results,'contacts_off_diagnostic':isolated}
