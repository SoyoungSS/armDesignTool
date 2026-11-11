"""Concept motor packaging, independent of joint frames and rigid-body mass data."""
import math
import numpy as np
from .math3d import forward, tf, align_z, serializable
from .model import InputError, number

POLICY='axis-module-layout-v1'
COS15=math.cos(math.radians(15))

def dimension(spec,key,default):
    mm=key.replace('_m','_mm')
    if key in spec and mm in spec: raise InputError('duplicate units '+key)
    return number(spec[key] if key in spec else spec[mm]*.001 if mm in spec else default,key,True)

def classify(n):
    js=n['model']['joints']; P=forward(n,n['poses']['geometric_zero'],world=False)
    centers=[P[j['child']][:3,3] for j in js]; result={}
    for i,j in enumerate(js):
        spec=n['dimensions'].get('joints',{}).get(j['name'],{})
        requested=spec.get('role','auto')
        if requested not in ['auto','axial_twist','transverse_bend']: raise InputError(j['name']+': invalid role')
        if j['type']!='revolute':
            if requested!='auto' or any(k.startswith('module_retreat_') for k in spec): raise InputError(j['name']+': layout role/retreat requires revolute')
            result[j['name']]={'role':j['type'],'source':'joint_type'}; continue
        axis=P[j['child']][:3,:3]@j['axis']; origin=centers[i]
        previous=next((centers[k] for k in range(i-1,-1,-1) if np.linalg.norm(origin-centers[k])>1e-8),P[n['base_link']][:3,3])
        v=origin-previous; length=float(np.linalg.norm(v))
        alignment=abs(float(v@axis))/length if length>1e-8 else None
        inferred='undetermined' if alignment is None else 'axial_twist' if alignment>=COS15 else 'transverse_bend'
        reference='nearest upstream noncoincident origin'
        if i==len(js)-1:
            tool_axis=P['tool_frame'][:3,2]
            if abs(float(tool_axis@axis))>=COS15: inferred='axial_twist'; reference='tool orientation Z axis'
        result[j['name']]={'role':inferred if requested=='auto' else requested,'source':'inferred' if requested=='auto' else 'explicit',
            'reference':reference,'incoming_angle_deg':None if alignment is None else math.degrees(math.acos(float(np.clip(alignment,0,1)))),
            'incoming_projection_m':float(v@axis),'axis_base_zero':axis.tolist(),'axis_frame':n['base_link']}
    return result

def annulus(outer,inner,length,segments=48):
    """Closed hollow sleeve; inner cavity is real geometry, not a dark disk."""
    angle=np.arange(segments)*2*math.pi/segments; xy=np.column_stack((np.cos(angle),np.sin(angle)))
    bevel=min((outer-inner)*.3,length*.15)
    rings=[(outer-bevel,-length/2),(outer-bevel*.3,-length/2+bevel*.3),(outer,-length/2+bevel),
           (outer,length/2-bevel),(outer-bevel*.3,length/2-bevel*.3),(outer-bevel,length/2),(inner,length/2),(inner,-length/2)]
    V=np.concatenate([np.column_stack((xy*r,np.full(segments,z))) for r,z in rings])
    F=[]; N=segments
    for j in range(len(rings)):
        next_ring=(j+1)%len(rings)
        for i in range(N):
            k=(i+1)%N; a=j*N+i; b=j*N+k; c=next_ring*N+k; d=next_ring*N+i
            F.extend([[a,b,c],[a,c,d]])
    return V,np.array(F,dtype=int)

def apply_layout(n,g,make_part):
    """Move/split visual components only. Never mutate model/poses/mass proxies."""
    js=n['model']['joints']; P=forward(n,n['poses']['geometric_zero'],world=False)
    roles=classify(n); clearance=dimension(n['dimensions'].get('appearance',{}),'assembly_clearance_m',.002)
    result={'policy':POLICY,'status':'resolved','clearance_m':clearance,'roles':roles,'pairs':[],'motor_envelopes':[],'unresolved':[]}
    n['joint_layout']=result; consumed=set()
    def reject(a,b,reason):
        result['unresolved'].append({'joints':[a['name'],b['name']],'reason':reason})
        n['notes'].append(a['name']+'/'+b['name']+': module layout needs review: '+reason)
    def add(owner,name,radius,length,T,color,role,hollow=False):
        p=make_part(name,'cylinder',[radius,length],T,color,True)
        p.update(assembly_role=role,owning_link=owner)
        if hollow:
            V,F=annulus(radius,radius*.54,length); p.update(vertices=V@T[:3,:3].T+T[:3,3],faces=F)
        g[owner]['parts'].append(p)
        return p
    for i in range(len(js)-1):
        a,b=js[i:i+2]
        if a['type']!='revolute' or b['type']!='revolute' or np.linalg.norm(b['origin']['xyz_m'])>1e-8: continue
        if (i and np.linalg.norm(a['origin']['xyz_m'])<=1e-8 and js[i-1]['type']=='revolute') or (i+2<len(js) and js[i+2]['type']=='revolute' and np.linalg.norm(js[i+2]['origin']['xyz_m'])<=1e-8):
            reject(a,b,'three or more coincident revolutes require an explicit mechanical design'); continue
        A=np.array(a['axis']); B=tf(b['origin'])[:3,:3]@b['axis']; dot=float(A@B)
        coaxial=abs(dot)>=1-1e-10
        kind='coaxial_stack' if coaxial else 'twist_bend'
        if roles[a['name']]['role']!='axial_twist' or (not coaxial and roles[b['name']]['role']!='transverse_bend'):
            reject(a,b,'no supported upstream twist and downstream bend/coaxial arrangement'); continue
        ga,gb=g[a['child']],g[b['child']]
        ha=next(p for p in ga['parts'] if p['name']=='housing'); hb=next(p for p in gb['parts'] if p['name']=='housing')
        ra,la=ha['size']; rb,lb=hb['size']
        projection=roles[a['name']]['incoming_projection_m']
        if abs(projection)<1e-8:
            reject(a,b,'no upstream axial installation length'); continue
        sign=-1 if projection>0 else 1; direction=sign*A
        # Bend motor sits to the side of the outgoing beam. Its rear face clears
        # the rotating beam's axis-aligned envelope, not just the joint origin.
        radial_extent=0.
        if not coaxial:
            for p in gb['parts']:
                if p['name'] not in ['housing','cap','rear_cap','cap_hub','rear_cap_hub']:
                    radial_extent=max(radial_extent,float(np.max(np.abs(p['vertices']@np.array(b['axis'])))))
        side=0. if coaxial else radial_extent+clearance+lb/2
        bcenter=B*side
        projected_b=abs(dot)*lb/2+math.sqrt(max(0.,1-dot*dot))*rb
        required=la/2+projected_b+clearance+float(direction@bcenter)
        spec=n['dimensions'].get('joints',{}).get(a['name'],{})
        retreat=dimension(spec,'module_retreat_m',max(required,la/2+clearance))
        if retreat<required-1e-10: raise InputError(a['name']+': module_retreat_m too small for motor-envelope clearance; minimum '+str(required))
        if retreat+la/2>abs(projection)*.85: raise InputError(a['name']+': insufficient axial installation length for module retreat; joint origin was not changed')
        ca=direction*retreat; Ta=tf(R=align_z(A),p=ca)
        Tb=tf(R=align_z(np.array(b['axis'])),p=np.array(b['axis'])*side)
        # Remove former origin-centered motor disks, including their decorative caps.
        for data in [ga,gb]: data['parts']=[p for p in data['parts'] if p['name'] not in ['housing','cap','rear_cap','cap_hub','rear_cap_hub']]
        # Fixed motor ends follow the parent; output shafts follow the child.
        sa=add(a['parent'],a['name']+'_stator',ra,la,tf(a['origin'])@Ta,'body','stator',True)
        sb=add(b['parent'],b['name']+'_stator',rb,lb,tf(b['origin'])@Tb,'body','stator',True)
        for j,p,r,l in [(a,sa,ra,la),(b,sb,rb,lb)]:
            result['motor_envelopes'].append({'joint':j['name'],'owner':j['parent'],'part':p['name'],'T':serializable(p['T']),'radius_m':r,'length_m':l})
        # Long hollow output sleeve, with an annular gap to the fixed stator.
        stop=lb/2+clearance if coaxial else rb*.47+abs(dot)*(side+lb/2)+clearance
        shaft_length=retreat+la/2-stop
        if shaft_length<=clearance: raise InputError(a['name']+': insufficient length between output shafts')
        add(a['child'],a['name']+'_output_shaft',ra*.47,shaft_length,tf(R=align_z(A),p=direction*(stop+shaft_length/2)),'body','rotor')
        exposed=retreat-la/2-clearance*2-stop
        if exposed>clearance:
            p=make_part(a['name']+'_output_fairing','cylinder',[ra*.84,exposed],tf(R=align_z(-direction),p=direction*(stop+exposed/2)),'body',True,
                        [(0,.92),(.08,1),(.35,1),(.75,.86),(1,.70)])
            p.update(assembly_role='rotor',owning_link=a['child']); ga['parts'].append(p)
        # A narrow ring on the rotating side makes the axial DOF visible.
        ring_center=ca-direction*(la/2+clearance*1.5)
        add(a['child'],a['name']+'_rotor_ring',ra*.96,clearance,tf(R=align_z(A),p=ring_center),'metal','rotor')
        bend_axis=tf(b['origin'])[:3,:3].T@(-direction) if coaxial else np.array(b['axis'])
        add(b['child'],b['name']+'_output_shaft',rb*.47,max(lb,side+lb/2),tf(R=align_z(bend_axis),p=bend_axis*(side+lb/2)/2),'metal','rotor')
        plate_at=side-lb/2-clearance*.5 if not coaxial else lb/2+clearance
        add(b['child'],b['name']+'_output_plate',rb*.95,clearance,tf(R=align_z(bend_axis),p=bend_axis*plate_at),'body','rotor')
        if coaxial:
            add(a['child'],a['name']+'_stack_flange',rb*.98,clearance*2,tf(R=align_z(A),p=direction*(lb/2+clearance*.5)),'body','rotor')
        if side>1e-8:
            # The stationary bend module is carried by a side support attached
            # to the upstream rotating sleeve; it is not a floating motor.
            from .surfaces import beam_frame
            T=tf(R=beam_frame(bcenter,A),p=direction*(rb*.85)+bcenter/2)
            p=make_part(b['name']+'_support','chamfer',[rb*.22,rb*.22,side],T,'body',True)
            p.update(assembly_role='stator_support',owning_link=b['parent']); g[b['parent']]['parts'].append(p)
        # Trim the stationary upstream fairing so that it does not engulf the
        # exposed rotating sleeve. Preserve the documented mass-estimation proxy.
        parent=g[a['parent']]; beam=next((p for p in parent['parts'] if p['name']=='beam'),None)
        if beam is not None:
            endpoint=tf(a['origin'])[:3,3]+tf(a['origin'])[:3,:3]@(direction*(retreat+la/2+clearance))
            L=float(np.linalg.norm(endpoint))
            if L<1e-6: raise InputError(a['name']+': upstream shell has no length after packaging')
            from .surfaces import beam_frame
            replacement=make_part('beam',beam['kind'],[beam['size'][0],beam['size'][1],L],tf(R=beam_frame(endpoint,beam['T'][:3,1]),p=endpoint/2),beam['material']['name'],True,beam.get('surface_profile'))
            ratio=beam.get('collision_size',beam['size'])[2]/beam['size'][2]
            replacement['collision_size']=[beam['size'][0],beam['size'][1],L*ratio]
            parent['parts'][parent['parts'].index(beam)]=replacement
            parent['rec']['assembly_shell_endpoint_m']=endpoint.tolist()
        ga['rec']['template']='axial_output_sleeve'; gb['rec']['template']='coaxial_output' if coaxial else 'transverse_output'
        result['pairs'].append({'joints':[a['name'],b['name']],'template':kind,'retreat_m':retreat,'required_retreat_m':required,'available_axial_length_m':abs(projection)*.85,
            'direction_local':direction.tolist(),'bend_side_offset_m':side,'shaft_stop_before_axis_m':stop,'source':'explicit' if any(k.startswith('module_retreat_') for k in spec) else 'inferred',
            'fixed_owners':[a['parent'],b['parent']],'output_owners':[a['child'],b['child']],
            'joint_frames_unchanged':True,'mass_properties_reassigned':False})
        consumed.add(a['name'])
    for j in js:
        spec=n['dimensions'].get('joints',{}).get(j['name'],{})
        if any(k.startswith('module_retreat_') for k in spec) and j['name'] not in consumed: raise InputError(j['name']+': module_retreat requires the first joint of a supported coincident pair')
    if result['unresolved']: result['status']='needs_review'
    return result

def packing_checks(n):
    """Envelope check independent of engine contact filters. Projection separation
    is sufficient, not necessary: an unseparated pair is reported as a candidate.
    """
    layout=n.get('joint_layout')
    if not layout: return {'status':'not_applicable'}
    from .validation import cases
    rows=[]; minimum=float('inf'); failures=[]
    envelopes=layout['motor_envelopes']; paired={frozenset(p['joints']) for p in layout['pairs']}
    for label,q in cases(n):
        P=forward(n,q)
        for i,a in enumerate(envelopes):
            A=P[a['owner']]@np.array(a['T']); za=A[:3,2]
            for b in envelopes[i+1:]:
                B=P[b['owner']]@np.array(b['T']); zb=B[:3,2]; delta=B[:3,3]-A[:3,3]
                candidates=[za,zb,delta,np.cross(za,zb)]
                def support(e,z,u):
                    d=float(np.clip(abs(z@u),0,1)); return e['length_m']*.5*d+e['radius_m']*math.sqrt(max(0,1-d*d))
                gaps=[]
                for u in candidates:
                    if np.linalg.norm(u)<1e-10: continue
                    u=u/np.linalg.norm(u); gaps.append(abs(float(delta@u))-support(a,za,u)-support(b,zb,u))
                gap=max(gaps); is_pair=frozenset([a['joint'],b['joint']]) in paired
                required=layout['clearance_m'] if is_pair else 0
                ok=gap>=required-1e-8
                if is_pair: minimum=min(minimum,gap)
                if not ok: failures.append({'pose':label,'joints':[a['joint'],b['joint']],'projection_gap_m':gap,'status':'potential_envelope_conflict'})
                rows.append({'pose':label,'joints':[a['joint'],b['joint']],'separated':bool(ok),'projection_gap_m':gap})
    return {'status':'needs_review' if failures or layout['unresolved'] else 'passed','independent_of_collision_filters':True,
        'min_paired_gap_m':minimum if math.isfinite(minimum) else None,'envelope_pairs_checked':len(rows),'conflicts':failures,'unresolved':layout['unresolved'],
        'scope':'generated motor cylinders only; shafts/bearings intentionally mate; finite poses, not a whole-arm CAD clearance proof'}
