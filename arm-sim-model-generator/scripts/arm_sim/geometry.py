"""Closed ring-shell visuals and analytical collision/mass proxies in link frames."""
import math, struct
from pathlib import Path
import numpy as np
from .math3d import tf, align_z, serializable
from .model import InputError, number, vector, check_inertia
from .surfaces import PALETTE, rounded, beam_frame, proportion_plan, template

COLORS={k:v[0] for k,v in PALETTE.items()}

def primitive(kind,size,segments=24):
    if kind=='cylinder':
        r,h=size; xy=[[r*math.cos(2*math.pi*i/segments),r*math.sin(2*math.pi*i/segments)] for i in range(segments)]
    else:
        w,d,h=size; x=w/2; y=d/2; c=min(w,d)*.16
        xy=[[-x+c,-y],[x-c,-y],[x,-y+c],[x,y-c],[x-c,y],[-x+c,y],[-x,y-c],[-x,-y+c]]
        if kind=='box': xy=[[-x,-y],[x,-y],[x,y],[-x,y]]
    N=len(xy); vertices=np.array([[x,y,z] for z in [-h/2,h/2] for x,y in xy]); faces=[]
    for i in range(1,N-1): faces.extend([[0,i+1,i],[N,N+i,N+i+1]])
    for i in range(N):
        j=(i+1)%N; faces.extend([[i,j,N+j],[i,N+j,N+i]])
    return vertices,np.array(faces,int)

def part(name,kind,size,T,color='body',collision=True,profile=None):
    V,F=rounded(kind,size,profile)
    return dict(name=name,kind=kind,size=size,T=T,color=COLORS[color],surface_profile=profile,material={'name':color,'roughness':PALETTE[color][1],'metallic':PALETTE[color][2]},collision=collision,vertices=V@T[:3,:3].T+T[:3,3],faces=F)

def stl(path,vertices,faces):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    V=np.asarray(vertices); F=np.asarray(faces,int)
    with path.open('wb') as f:
        f.write(b'arm-sim deterministic SI link-local STL'.ljust(80,b'\0')); f.write(struct.pack('<I',len(F)))
        for face in F:
            a,b,c=V[face]; normal=np.cross(b-a,c-a); area=np.linalg.norm(normal)
            if area<1e-15: raise InputError(str(path)+': degenerate triangle')
            f.write(struct.pack('<12fH',*(normal/area),*a,*b,*c,0))

def read_stl(path):
    data=Path(path).read_bytes()
    if len(data)<84: raise InputError('invalid STL '+str(path))
    n=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*n: raise InputError('invalid binary STL length')
    tris=np.array([struct.unpack_from('<12fH',data,84+50*i)[3:12] for i in range(n)]).reshape(-1,3,3)
    if not np.all(np.isfinite(tris)): raise InputError('nonfinite STL')
    if np.any(np.linalg.norm(np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]),axis=1)<1e-15): raise InputError('degenerate STL')
    return tris

def build_geometry(n,folder):
    js=n['model']['joints']; links=[n['base_link']]+[j['child'] for j in js]
    dims=n['dimensions']; rules=dims.get('rules',{}); result={}; records={}
    allowed={'links','joints','base','flange','rules','appearance'}
    if set(dims)-allowed: raise InputError('unknown dimensions keys '+str(set(dims)-allowed))
    if set(dims.get('links',{}))-set(links): raise InputError('dimensions.links unknown link')
    if set(dims.get('joints',{}))-{j['name'] for j in js}: raise InputError('dimensions.joints unknown joint')
    for key in rules:
        if key not in ['width_ratio','min_width_m','max_width_m','module_ratio']: raise InputError('unknown dimension rule '+key)
        number(rules[key],'rules.'+key,True)
    if rules.get('min_width_m',.025)>rules.get('max_width_m',.10): raise InputError('min_width_m exceeds max_width_m')
    appearance=dims.get('appearance',{})
    if set(appearance)-{'detail_level','assembly_clearance_m','assembly_clearance_mm'}: raise InputError('unknown dimensions.appearance field')
    detail=appearance.get('detail_level',1)
    if type(detail) is not int or detail not in [0,1]: raise InputError('detail_level must be 0 or 1')
    plan=proportion_plan(n); plan['detail_level']=detail; n['appearance_design']=plan
    def value(spec,key,default,rec):
        mm=key.replace('_m','_mm')
        if key in spec and mm in spec: raise InputError('duplicate dimension units '+key)
        v=spec.get(key,spec.get(mm,default*1000)/1000)
        number(v,'dimensions.'+key,True)
        rec[key]={'value':v,'source':'design' if key in spec or mm in spec else 'estimated','basis':plan['basis']}
        return v
    for idx,link in enumerate(links):
        rec={}; parts=[]; up=js[idx-1] if idx else None; down=js[idx] if idx<len(js) else None
        end=np.array(down['origin']['xyz_m'] if down else n['model']['tip']['xyz_m']); span=float(np.linalg.norm(end))
        spec=dims.get('links',{}).get(link,{})
        for k in spec:
            if k not in ['width_m','width_mm','thickness_m','thickness_mm','max_envelope_m','max_envelope_mm']: raise InputError('unknown link dimension '+k)
        width=value(spec,'width_m',plan['widths_m'][idx],rec)
        thick=value(spec,'thickness_m',width*.82,rec)
        module=dims.get('joints',{}).get(up['name'],{}) if up else dims.get('base',{})
        module_keys=['diameter_m','diameter_mm','axial_width_m','axial_width_mm']
        if up: module_keys+=['role','module_retreat_m','module_retreat_mm']
        for k in module:
            if k not in module_keys: raise InputError('unknown module dimension '+k)
        cap=plan['module_diameter_caps_m'][idx]
        default_d=width*rules.get('module_ratio',1.35)
        if idx and cap is not None: default_d=min(default_d,cap)
        diameter=value(module,'diameter_m',default_d,rec)
        rec['module_space_cap_m']={'value':cap,'source':'estimated','basis':'90% of nearest nonzero installation spacing across coincident frames'}
        if idx and cap is not None and diameter>cap+1e-12:
            n['notes'].append(link+': explicit module diameter exceeds inferred installation clearance; design value retained, review interference')
        axial=value(module,'axial_width_m',min(width*.82,diameter*.72),rec)
        axis=np.array(up.get('axis',[0,0,1]) if up else [0,0,1]); R=align_z(axis)
        kind=template(up,down,end,diameter); rec['template']=kind
        if idx==0:
            diameter=value(module,'diameter_m',max(diameter*1.35, .10),rec)
            parts.append(part('base','cylinder',[diameter/2,axial],tf(p=[0,0,-axial/2]),'joint'))
            parts.append(part('mounting_rim','cylinder',[diameter*.52,axial*.16],tf(p=[0,0,-axial*.92]),'metal',False))
        elif up['type']=='prismatic':
            parts.append(part('slider','chamfer',[diameter,diameter*.8,axial],tf(R=R),'joint'))
        else:
            housing_color='body' if kind in ['cross_axis_wrist','elbow_shell','fixed_adapter'] else 'joint'
            housing_profile=None
            if kind=='cross_axis_wrist':
                housing_profile=[(0,.64),(.04,.78),(.13,.90),(.3,.99),(.5,1),(.7,.99),(.87,.90),(.96,.78),(1,.64)]
            parts.append(part('housing','cylinder',[diameter/2,axial],tf(R=R),housing_color,profile=housing_profile))
            for sign,label in [(1,'cap'),(-1,'rear_cap')]:
                parts.append(part(label,'cylinder',[diameter*.40,axial*.10],tf(R=R,p=axis*sign*axial*.48),'joint',False))
                parts.append(part(label+'_hub','cylinder',[diameter*.25,axial*.035],tf(R=R,p=axis*sign*axial*.545),'metal',False))
        # A single closed multi-section outer shell spans both installation points.
        # Ends are buried into the same-link housing; no exposed narrow neck.
        clearance=min(diameter*.55,span*.30); proxy_len=max(0,span-2*clearance)
        if span>1e-6:
            T=tf(R=beam_frame(end,axis),p=end/2)
            profile=[(0,.62),(.025,.80),(.07,.94),(.15,1),(.30,.98),(.60,.90),(.82,.84),(.93,.79),(.975,.68),(1,.54)]
            if kind=='pedestal':
                profile=[(0,1),(.03,1),(.14,.98),(.45,.90),(.75,.82),(.93,.78),(1,.72)]
            elif kind in ['elbow_shell','offset_parallel']:
                profile=[(0,.65),(.04,.87),(.12,1),(.28,.98),(.58,.89),(.78,.88),(.90,.91),(.97,.79),(1,.58)]
            parts.append(part('beam','chamfer',[width,thick,span],T,profile=profile))
            parts[-1]['collision_size']=[width,thick,max(proxy_len,span*.1)]
            rec['shell_collision']='central box; rounded end transitions approximated by joint housings, not exact surface collision'
            proxy={'kind':'box','size':[width,thick,max(proxy_len,span*.1)],'T':T}
        else:
            rec['short_span']={'value':span,'source':'kinematics','basis':'full-scale wrist housing; original frames preserved'}
            proxy={'kind':'cylinder','size':[diameter/2,axial],'T':tf(R=R)}
        if span<diameter and span>1e-8:
            rec['clearance_warning']='short span compared with housing diameter; interference check required'
            if 'diameter_m' in module or 'diameter_mm' in module: n['notes'].append(link+': design housing diameter exceeds installation span; dimension/assembly conflict')
        if down and down['type']=='prismatic':
            O=tf(down['origin']); a=O[:3,:3]@np.array(down['axis']); low,high=down['limits']; length=high-low+axial*1.5
            center=O[:3,3]+a*(low+high)/2
            parts.append(part('rail','chamfer',[width*.55,thick*.4,length],tf(R=align_z(a),p=center),'joint'))
            rec['rail']='parent-owned guide spans full travel; child owns slider'
        if down is None:
            fs=dims.get('flange',{}); rec['flange']={}
            if set(fs)-{'diameter_m','diameter_mm','axial_width_m','axial_width_mm'}: raise InputError('unknown flange dimension')
            fd=value(fs,'diameter_m',width*1.05,rec['flange']); fh=value(fs,'axial_width_m',width*.18,rec['flange'])
            parts.append(part('flange','cylinder',[fd/2,fh],tf(n['model']['tip']),'metal'))
            if detail:
                F=tf(n['model']['tip'])
                for k in range(4):
                    angle=k*math.pi/2+math.pi/4
                    pos=F[:3,3]+F[:3,:3]@np.array([fd*.34*math.cos(angle),fd*.34*math.sin(angle),fh*.52])
                    parts.append(part('flange_fastener_'+str(k),'cylinder',[fd*.035,fh*.06],tf(R=F[:3,:3],p=pos),'joint',False))
        if not detail:
            parts=[p for p in parts if not p['name'].endswith('_hub') and p['name']!='mounting_rim']
        result[link]={'parts':parts,'proxy':proxy,'rec':rec,'spec':spec,'span':span}
    from .joint_layout import apply_layout
    apply_layout(n,result,part)
    for link,data in result.items():
        parts=data['parts']; proxy=data['proxy']; rec=data['rec']; spec=data['spec']; span=data['span']
        allV=np.concatenate([p['vertices'] for p in parts]); bound=np.ptp(allV,axis=0)
        env=spec.get('max_envelope_m',None)
        if 'max_envelope_mm' in spec:
            if env is not None: raise InputError('duplicate max envelope units')
            env=[v*.001 for v in vector(spec['max_envelope_mm'],'max_envelope_mm')]
        if env is not None and np.any(bound>vector(env,'max_envelope_m')): raise InputError(link+': dimensions conflict with kinematic assembly / max_envelope_m')
        faces=[]; offset=0; collision_vertices=[]; collision_faces=[]; collision_offset=0
        for p in parts:
            filename=f'{link}_{p["name"]}_visual.stl'; stl(Path(folder)/filename,p['vertices'],p['faces']); p['mesh']=filename
            faces.extend((p['faces']+offset).tolist()); offset+=len(p['vertices'])
            if p['collision']:
                ck='cylinder' if p['kind']=='cylinder' else 'box'; V,F=primitive(ck,p.get('collision_size',p['size']),12)
                V=V@p['T'][:3,:3].T+p['T'][:3,3]
                p['collision_kind']=ck; p['collision_mesh']=f'{link}_{p["name"]}_collision.stl'; stl(Path(folder)/p['collision_mesh'],V,F)
                collision_vertices.append(V); collision_faces.extend((F+collision_offset).tolist()); collision_offset+=len(V)
        stl(Path(folder)/f'{link}_visual.stl',allV,faces)
        stl(Path(folder)/f'{link}_collision.stl',np.concatenate(collision_vertices),collision_faces)
        records[link]={'method':'rounded-shell-v2: whole-arm shell and '+rec['template'],'span_m':span,'dimensions':rec,'bounds_m':[allV.min(0).tolist(),allV.max(0).tolist()],'collision':'central beam boxes and module cylinders; rounded transitions approximate; caps/details visual only','parts':serializable([{k:v for k,v in p.items() if k not in ['vertices','faces']} for p in parts]),'mass_proxy':serializable(proxy)}
    n['geometry']=records
    return result

def unit_mass_properties(proxy):
    size=proxy['size']; T=np.asarray(proxy['T'])
    if proxy['kind']=='cylinder':
        r,h=size; volume=math.pi*r*r*h; I=np.diag([(3*r*r+h*h)/12]*2+[r*r/2])
    else:
        x,y,z=size; volume=x*y*z; I=np.diag([y*y+z*z,x*x+z*z,x*x+y*y])/12
    return volume,T[:3,3],T[:3,:3]@I@T[:3,:3].T

def mass_properties(n,g):
    missing=[]; result={}; mode=n['export']['mode']; estimation=n['physics'].get('estimation',{})
    if estimation.get('method') not in [None,'uniform_density','known_mass_proxy']: raise InputError('unknown physics.estimation.method')
    for link,data in g.items():
        volume,centroid,Iunit=unit_mass_properties(data['proxy']); comps=n['components'][link]; sources=[]
        if not comps:
            if mode=='concept' and estimation.get('method')=='uniform_density':
                rho=number(estimation.get('density_kg_m3'),'density_kg_m3',True)
                comps=[{'name':'density_proxy','mass_kg':rho*volume,'com_m':centroid.tolist(),'inertia_kg_m2':(rho*volume*Iunit).tolist()}]
                sources.append('estimated uniform density on single documented solid proxy; decorative/overlapping pieces excluded')
            else: missing.append(link+': mass, COM, inertia'); continue
        cooked=[]
        for c in comps:
            mass=c['mass_kg']; com=np.array(c['com_m']); I=c.get('inertia_kg_m2')
            if I is None:
                if mode=='concept' and estimation.get('method') in ['uniform_density','known_mass_proxy']:
                    I=mass*Iunit; sources.append(c['name']+': known mass distributed on documented proxy translated to supplied COM')
                else: missing.append(link+'/'+str(c.get('name'))+': COM inertia'); continue
            else: sources.append(c['name']+': supplied mass/COM/inertia' if c['name']!='density_proxy' else 'density proxy')
            cooked.append((mass,com,check_inertia(I,link)))
        if len(cooked)!=len(comps): continue
        mass=sum(x[0] for x in cooked); com=sum(x[0]*x[1] for x in cooked)/mass
        I=np.zeros((3,3))
        for mi,ci,Ii in cooked:
            v=ci-com; I+=Ii+mi*(np.dot(v,v)*np.eye(3)-np.outer(v,v))
        check_inertia(I,link)
        result[link]={'mass_kg':mass,'com_m':com.tolist(),'inertia_kg_m2':I.tolist(),'sources':sources}
    n['mass_properties']=result; n['missing_physics']=missing
    n['model_grade']='visualization' if missing else ('concept_simulation' if any('proxy' in s or 'estimated' in s for p in result.values() for s in p['sources']) else 'supplied_physics_unverified')
    return result
