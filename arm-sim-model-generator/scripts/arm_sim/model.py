"""Input adapters are local: existing skills and their interfaces are never modified."""
import copy, json, re, math
import numpy as np
from .math3d import tf, rpy, serializable

IDENTITY={'xyz_m':[0,0,0],'rpy_rad':[0,0,0]}
class InputError(ValueError): pass

def read_input(path):
    text=path.read_text(encoding='utf-8-sig')
    # Preserve quoted URLs and escaped quotes; JSONC comments only, no trailing commas.
    pattern=r'"(?:\\.|[^"\\])*"|//[^\r\n]*|/\*[\s\S]*?\*/'
    text=re.sub(pattern,lambda m:m[0] if m[0].startswith('"') else ' ',text)
    def pairs(items):
        d={}
        for k,v in items:
            if k in d: raise InputError('Duplicate JSON key: '+k)
            d[k]=v
        return d
    return json.loads(text,object_pairs_hook=pairs,parse_constant=lambda s:(_ for _ in ()).throw(InputError(s)))

def number(x,path,positive=False):
    if isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x): raise InputError(path+': finite number required')
    if positive and x<=0: raise InputError(path+': must be positive')
    return float(x)

def vector(x,path,size=3):
    if not isinstance(x,list) or len(x)!=size: raise InputError(path+f': {size} numbers required')
    return [number(v,path) for v in x]

def transform(x,path):
    if not isinstance(x,dict): raise InputError(path+': origin/transform required')
    out={}
    for si,other,factor in [('xyz_m','xyz_mm',.001),('rpy_rad','rpy_deg',math.pi/180)]:
        if si in x and other in x: raise InputError(path+': duplicate units '+si)
        key=si if si in x else other
        if key not in x: raise InputError(path+'.'+si+': required')
        out[si]=[v*(1 if key==si else factor) for v in vector(x[key],path+'.'+key)]
    return out

def safe_name(s,path):
    if not isinstance(s,str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',s):
        raise InputError(path+': ASCII letter then letters/digits/underscore required (explicit rename before import)')
    return s

def adapt(raw):
    d=copy.deepcopy(raw); notes=[]
    if d.get('schema_version')!='1.0': raise InputError('schema_version must be 1.0')
    source=d.get('source','native')
    if d.get('input_mode')=='simple':
        source='arm-statics-parametric/simple'; s=d['simple']; js=[]; prev=None
        for i,v in enumerate(s['segments']):
            for k in ['name','axis','length_m','limits_rad','link_mass_kg']:
                if k not in v: raise InputError(f'simple.segments[{i}].{k}: required')
            L=number(v['length_m'],'length_m',True)
            direction=np.array(vector(v.get('direction',[1,0,0]),'direction'))
            if np.linalg.norm(direction)<1e-12: raise InputError('direction: zero vector')
            delta=direction/np.linalg.norm(direction)*L
            if i==0 and 'rpy_rad' in v: raise InputError('simple first rpy belongs in base')
            origin=s.get('base',copy.deepcopy(IDENTITY)) if i==0 else {'xyz_m':prev.tolist(),'rpy_rad':v.get('rpy_rad',[0,0,0])}
            comps=[{'name':'link','mass_kg':v['link_mass_kg'],'com_m':v.get('link_com_m',(delta/2).tolist())}]
            if 'joint_mass_kg' in v:
                comps.append({'name':'module','mass_kg':v['joint_mass_kg'],'com_m':v.get('joint_com_m',[0,0,0])})
                notes.append(v['name']+': simple whole module assigned to output body; fixed/moving split unavailable')
            elif 'joint_com_m' in v: raise InputError('joint_com_m requires joint_mass_kg')
            for key in ['direction','link_com_m']:
                if key not in v: notes.append(v['name']+': simple default '+key)
            js.append(dict(name=v['name'],type='revolute',origin=origin,axis=v['axis'],limits=v['limits_rad'],components=comps)); prev=delta
        if 'base' not in s: notes.append('simple.base default identity')
        d['model']={'name':s['name'],'joints':js,'tip':{'xyz_m':prev.tolist(),'rpy_rad':[0,0,0]}}
    elif d.get('input_mode')=='standard': source='arm-statics-parametric/standard'
    elif 'mode' in d and 'model' in d: source='arm-workspace-parametric/standard'
    if 'base_in_world' in d:
        d.setdefault('installation',{})['base_in_world']=d['base_in_world']
    ins=d.setdefault('installation',{})
    if 'base_orientation_in_world' in ins:
        ins['base_in_world']={'xyz_m':[0,0,0],**ins.pop('base_orientation_in_world')}
    if 'gravity_m_s2' in d:
        if 'gravity_world_m_s2' in ins: raise InputError('both base and world gravity provided')
        R=rpy(transform(ins.get('base_in_world',IDENTITY),'installation')['rpy_rad'])
        ins['gravity_world_m_s2']=(R@np.array(vector(d['gravity_m_s2'],'gravity_m_s2'))).tolist()
    if isinstance(d.get('poses'),list): d['poses']={'acceptance':d['poses']}
    if source.startswith('arm-statics'):
        if d.get('drives'): notes.append('statics drives retained as provenance, NOT reused as actuator/URDF ratings')
        d['source_statics_drives']=d.pop('drives',{})
        if d.get('wrenches'): notes.append('external statics wrenches not exported as simulator forces')
    notes.extend(d.get('metadata',{}).get('normalization_assumptions',[]))
    return d,source,notes

def normalize(raw):
    d,source,notes=adapt(raw)
    if source=='native':
        allowed={'schema_version','source','model','dimensions','physics','installation','poses','drives','export','metadata'}
        if set(d)-allowed: raise InputError('unknown native root fields: '+', '.join(sorted(set(d)-allowed)))
    if 'model' not in d: raise InputError('model: required')
    m=d['model']; missing=[]
    for k in ['name','joints','tip']:
        if k not in m: missing.append('model.'+k)
    for i,j in enumerate(m.get('joints',[])):
        for k in ['name','type','origin']+([] if j.get('type')=='fixed' else ['axis','limits']):
            if k=='limits' and ('limits_deg' in j or 'limits_mm' in j): continue
            if k not in j: missing.append(f'model.joints[{i}].{k}')
    if missing: raise InputError('Missing required fields: '+', '.join(missing))
    name=safe_name(m['name'],'model.name').lower(); js=m['joints']
    if not isinstance(js,list) or not 1<=len(js)<=64: raise InputError('model.joints: 1..64 serial joints required')
    names=set(); links=set(); base=safe_name(m.get('base_link','base_link'),'base_link'); links.add(base)
    if base in ['world','tool_frame']: raise InputError('reserved base link name '+base)
    explicit=any('parent' in j or 'child' in j for j in js)
    if explicit:
        if not all('parent' in j and 'child' in j for j in js): raise InputError('all parent/child required when explicit connectivity used')
        remaining=js[:]; js=[]; cur=base
        while remaining:
            found=[j for j in remaining if j['parent']==cur]
            if len(found)!=1: raise InputError('connectivity: disconnected, branched or cyclic chain at '+cur)
            j=found[0]; js.append(j); remaining.remove(j); cur=j['child']
    parent=base
    for i,j in enumerate(js):
        nm=safe_name(j['name'],f'joints[{i}].name')
        if nm in names or nm in ['tip','tool_fixed','world_fixed']: raise InputError('duplicate/reserved joint name: '+nm)
        names.add(nm)
        if any(k in j for k in ['mimic','pitch','floating']): raise InputError(nm+': coupled/helical/floating joints unsupported')
        if j['type'] not in ['revolute','prismatic','fixed']: raise InputError(nm+': unsupported joint type')
        j['origin']=transform(j['origin'],nm+'.origin')
        j['parent']=parent; j['child']=safe_name(j.get('child',nm+'_link'),nm+'.child')
        if j['child'] in links or j['child'] in ['world','tool_frame']: raise InputError('duplicate/reserved link '+j['child'])
        links.add(j['child']); parent=j['child']
        if j['type']=='fixed':
            if any(k in j for k in ['axis','limits','limits_deg','limits_mm']): raise InputError(nm+': fixed forbids axis/limits')
        else:
            a=np.array(vector(j['axis'],nm+'.axis')); norm=np.linalg.norm(a)
            if norm<1e-12: raise InputError(nm+'.axis: zero vector')
            if abs(norm-1)>1e-9: notes.append(nm+': axis normalized')
            j['axis']=(a/norm).tolist()
            keys=[k for k in ['limits','limits_deg','limits_mm'] if k in j]
            if len(keys)!=1: raise InputError(nm+': one limits unit required')
            key=keys[0]
            if key=='limits_deg' and j['type']!='revolute' or key=='limits_mm' and j['type']!='prismatic': raise InputError(nm+': incompatible limits units')
            scale={'limits':1,'limits_deg':math.pi/180,'limits_mm':.001}[key]
            j['limits']=[v*scale for v in vector(j[key],nm+'.limits',2)]
            if j['limits'][0]>=j['limits'][1]: raise InputError(nm+': lower limit must be below upper')
    if names & links: raise InputError('joint and link names must not collide in USD namespace: '+str(names & links))
    m={'name':name,'joints':js,'tip':transform(m['tip'],'model.tip')}
    ins=d['installation']; ins={'base_in_world':transform(ins.get('base_in_world',IDENTITY),'installation.base_in_world'), 'gravity_world_m_s2':vector(ins.get('gravity_world_m_s2',[0,0,-9.81]),'gravity_world_m_s2')}
    if 'gravity_world_m_s2' not in d['installation']: notes.append('world gravity default [0,0,-9.81] m/s2')
    ex=d.get('export',{}); mode=ex.get('mode','strict')
    if mode not in ['strict','concept']: raise InputError('export.mode must be strict or concept')
    targets=ex.get('targets',['urdf','ros1','mujoco','isaac'])
    if not targets or set(targets)-{'urdf','ros1','mujoco','isaac'}: raise InputError('export.targets invalid')
    active=[j for j in js if j['type']!='fixed']; active_names={j['name'] for j in active}
    poses=d.get('poses',{}); initial=poses.get('initial',{j['name']:max(j['limits'][0],min(0,j['limits'][1])) for j in active})
    if 'geometric_zero' in poses and (set(poses['geometric_zero'])!=active_names or any(v!=0 for v in poses['geometric_zero'].values())): raise InputError('poses.geometric_zero must be all active names at zero; encode geometry in origin')
    def pose(q,label):
        if not isinstance(q,dict) or set(q)!=active_names: raise InputError(label+': supply exactly all active joint names')
        out={}
        for j in active:
            x=q[j['name']]
            if isinstance(x,dict):
                unit='deg' if j['type']=='revolute' else 'mm'
                if set(x)!={unit}: raise InputError(label+': expected '+unit)
                x=number(x[unit],label)*(math.pi/180 if unit=='deg' else .001)
            x=number(x,label)
            if not j['limits'][0]<=x<=j['limits'][1]: raise InputError(label+': out of limits '+j['name'])
            out[j['name']]=x
        return out
    poses={'geometric_zero':{j['name']:0 for j in active},'initial':pose(initial,'poses.initial'), 'acceptance':[{'name':p['name'],'q':pose(p['q'],'poses.acceptance')} for p in poses.get('acceptance',[])]}
    drives=d.get('drives',{})
    for name,dynamics in d.get('physics',{}).get('joints',{}).items():
        if name not in active_names or set(dynamics)-{'damping','friction'}: raise InputError('physics.joints: active name and damping/friction only')
        cfg=drives.setdefault(name,{})
        for k,v in dynamics.items():
            if k in cfg and cfg[k]!=v: raise InputError(name+': contradictory '+k+' in physics/drives')
            cfg[k]=v
    if set(drives)-active_names: raise InputError('drives: unknown/fixed joint')
    missing_limits=[]
    for j in active:
        cfg=drives.setdefault(j['name'],{}); cfg.setdefault('mode','auto')
        if cfg['mode'] not in ['auto','passive','position','velocity','effort']: raise InputError('unsupported drive mode')
        for key in ['effort','velocity']:
            if key not in cfg:
                val=ex.get('limit_placeholders',{}).get(key) if mode=='concept' else None
                if val is None: missing_limits.append(j['name']+'.'+key)
                else: cfg[key]=val; notes.append(j['name']+'.'+key+': EXPLICIT PLACEHOLDER '+str(val))
            if key in cfg: cfg[key]=number(cfg[key],j['name']+'.'+key,True)
        for key in ['kp','kv','damping','friction','timeconst']:
            if key in cfg:
                cfg[key]=number(cfg[key],j['name']+'.'+key)
                if cfg[key]<0: raise InputError(key+': must be nonnegative')
        # Resolve missing servo gains only after physical properties are available.
    components={base:[]}; aliases={base:base,'tip':js[-1]['child'],'tool_frame':js[-1]['child']}
    aliases.update({j['name']:j['child'] for j in js}); aliases.update({j['child']:j['child'] for j in js})
    for j in js: components[j['child']]=copy.deepcopy(j.get('components',[]))
    physics=d.get('physics',{})
    for link,comps in physics.get('components',{}).items():
        if link not in aliases: raise InputError('physics.components unknown frame '+link)
        components[aliases[link]].extend([{**copy.deepcopy(c),'frame':c.get('frame',link)} for c in comps])
    for att in d.get('attachments',[])+physics.get('attachments',[]):
        frame=att.get('frame')
        if frame not in aliases: raise InputError('attachment unknown frame '+str(frame))
        components[aliases[frame]].append({**att,'frame':frame})
    payload=physics.get('payload',d.get('payload'))
    if payload and number(payload['mass_kg'],'payload.mass_kg')!=0:
        if 'included_in' in payload:
            inc=payload['included_in']
            if not isinstance(inc,dict) or inc.get('link') not in aliases: raise InputError('payload.included_in requires {link,component}')
            if not any(c.get('name')==inc.get('component') for c in components[aliases[inc['link']]]): raise InputError('payload.included_in component not found')
            notes.append('payload already included in '+str(inc)+'; not added twice')
        else:
            if not payload.get('separate_mass_confirmed',False): raise InputError('payload: specify included_in {link,component} or separate_mass_confirmed:true to avoid double counting')
            if 'com_in_tip_m' in payload and 'com_in_tip_mm' in payload: raise InputError('payload duplicate COM units')
            com=payload.get('com_in_tip_m',[v*.001 for v in vector(payload.get('com_in_tip_mm'),'payload.com_in_tip_mm')] if 'com_in_tip_mm' in payload else None)
            components[js[-1]['child']].append({'name':'payload','frame':'tip','mass_kg':payload['mass_kg'],'com_m':com,**{k:v for k,v in payload.items() if k.startswith('inertia')}})
    for link,cs in components.items():
        seen=set()
        for c in cs:
            if not isinstance(c.get('name'),str) or not c['name']: raise InputError(link+': component name required')
            if c.get('name') in seen: raise InputError(link+': duplicate mass component '+str(c.get('name')))
            seen.add(c.get('name'))
            if 'com_mm' in c:
                if 'com_m' in c: raise InputError(link+': duplicate COM units')
                c['com_m']=[v*.001 for v in vector(c.pop('com_mm'),'com_mm')]
            if 'inertia_rpy_deg' in c:
                if 'inertia_rpy_rad' in c: raise InputError(link+': duplicate inertia rotation units')
                c['inertia_rpy_rad']=[v*math.pi/180 for v in vector(c.pop('inertia_rpy_deg'),'inertia_rpy_deg')]
            c['mass_kg']=number(c.get('mass_kg'),link+'.mass_kg',True); c['com_m']=vector(c.get('com_m'),link+'.com_m')
            frame=c.get('frame',link)
            if frame not in aliases or aliases[frame]!=link: raise InputError(link+': component frame must belong to its body or tip')
            C=tf(m['tip']) if frame in ['tip','tool_frame'] else np.eye(4)
            c['com_m']=(C[:3,:3]@c['com_m']+C[:3,3]).tolist()
            if 'inertia_kg_m2' in c:
                I=np.asarray(c['inertia_kg_m2'],float)
                if I.shape!=(3,3) or not np.all(np.isfinite(I)): raise InputError(link+': inertia must be finite 3x3')
                if 'inertia_frame' not in c: raise InputError(link+': inertia_frame required (component or link)')
                R=C[:3,:3]@rpy(vector(c.get('inertia_rpy_rad',[0,0,0]),'inertia_rpy_rad')) if c['inertia_frame']=='component' else np.eye(3)
                if c['inertia_frame'] not in ['component','link']: raise InputError('inertia_frame must be component or link')
                c['inertia_kg_m2']=check_inertia(R@I@R.T,link).tolist()
            c['frame']=link
    return serializable(dict(schema_version='1.0',generator_version='1.3.0',source=source,model=m,base_link=base,installation=ins,poses=poses,dimensions=d.get('dimensions',{}),components=components,physics=physics,drives=drives,export={**ex,'mode':mode,'targets':targets},notes=notes,missing_drive_limits=missing_limits,source_statics_drives=d.get('source_statics_drives',{})))

def check_inertia(I,label):
    I=np.asarray(I,float)
    if not np.allclose(I,I.T,rtol=1e-8,atol=1e-12): raise InputError(label+': inertia not symmetric')
    eig=np.linalg.eigvalsh(I)
    if eig[0]<=0 or eig[2]>eig[0]+eig[1]+1e-10*max(1,float(eig[2])): raise InputError(label+': nonphysical principal inertia/triangle inequality')
    return I
