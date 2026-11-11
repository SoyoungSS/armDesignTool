"""Deterministic, engine-independent SI position-servo design (policy serial-pd-v1).

No fake armature, density, friction or gravity cancellation. Sampled inertia and
gravity are design evidence, not a global bound or a real motor identification.
"""
import copy, math
import numpy as np
from .math3d import tf, forward
from .model import InputError, number

POLICY='serial-pd-v1'
DEFAULTS={'frequency_hz':1.5,'damping_ratio':1.2,'position_error_rad':0.035,
          'position_error_m':0.002,'design_step_rad':0.1,'design_step_m':0.01,
          'proportional_effort_fraction':0.5,'max_timestep_s':0.001,
          'modal_step_limit':0.1,'command_filter_s':0.03}

def sample_poses(n):
    active=[j for j in n['model']['joints'] if j['type']!='fixed']
    poses=[dict(n['poses']['initial'])]+[dict(p['q']) for p in n['poses']['acceptance']]
    poses.append({j['name']:sum(j['limits'])/2 for j in active})
    poses.append({j['name']:float(np.clip(0,*j['limits'])) for j in active})
    primes=[]; k=2
    while len(primes)<len(active):
        if all(k%p for p in primes if p*p<=k): primes.append(k)
        k+=1
    def halton(i,b):
        v=0.; f=1.
        while i: f/=b; v+=f*(i%b); i//=b
        return v
    for sample in range(1,17):
        poses.append({j['name']:j['limits'][0]+halton(sample,primes[k])*(j['limits'][1]-j['limits'][0]) for k,j in enumerate(active)})
    return poses

def mass_matrix_and_gravity(n,q):
    """Rigid-body Jacobian sum. Joint/world frames follow the same FK convention."""
    active=[j for j in n['model']['joints'] if j['type']!='fixed']; N=len(active)
    poses=forward(n,q); axes=[]; origins=[]; kinds=[]; M=np.zeros((N,N)); gravity=np.zeros(N)
    g=np.array(n['installation']['gravity_world_m_s2'])
    for j in n['model']['joints']:
        if j['type']!='fixed':
            T=poses[j['parent']]@tf(j['origin']); axes.append(T[:3,:3]@j['axis']); origins.append(T[:3,3]); kinds.append(j['type'])
        p=n['mass_properties'][j['child']]; T=poses[j['child']]; c=T[:3,:3]@p['com_m']+T[:3,3]
        Jv=np.zeros((3,N)); Jw=np.zeros((3,N))
        for k,(a,o,kind) in enumerate(zip(axes,origins,kinds)):
            if kind=='revolute': Jv[:,k]=np.cross(a,c-o); Jw[:,k]=a
            else: Jv[:,k]=a
        I=T[:3,:3]@p['inertia_kg_m2']@T[:3,:3].T
        M+=p['mass_kg']*(Jv.T@Jv)+Jw.T@I@Jw
        gravity-=Jv.T@(p['mass_kg']*g)
    return M,gravity

def prepare_drives(n):
    settings={**DEFAULTS,**n['export'].get('drive_policy',{})}
    if set(settings)-set(DEFAULTS): raise InputError('export.drive_policy: unknown setting')
    for k,v in settings.items(): number(v,'export.drive_policy.'+k,True)
    if settings['damping_ratio']<1: raise InputError('automatic damping_ratio must be >= 1')
    if settings['proportional_effort_fraction']>1: raise InputError('proportional_effort_fraction must be <= 1')
    if settings['max_timestep_s']>0.005 or settings['modal_step_limit']>0.2: raise InputError('requested timestep policy exceeds supported bounds')
    if n['missing_physics']:
        n['drive_design']={'policy':POLICY,'status':'pending','reason':'complete physical properties required','settings':settings}
        return
    active=[j for j in n['model']['joints'] if j['type']!='fixed']
    n['drive_design']={'policy':POLICY,'status':'resolved','settings':settings,'joints':{},'timestep_s':settings['max_timestep_s'],'integrator':'implicitfast','gravity_compensation':False}
    if not active: return
    for j in active:
        cfg=n['drives'][j['name']]
        if cfg['mode']!='passive' and any(k not in cfg for k in ['effort','velocity']):
            raise InputError(j['name']+': driven simulation requires explicit effort/velocity or concept limit_placeholders')
    poses=sample_poses(n); matrices=[]; gravities=[]
    for q in poses:
        M,g=mass_matrix_and_gravity(n,q)
        if np.linalg.eigvalsh(M)[0]<=0: raise InputError('non-positive joint-space inertia; check physical parameters')
        matrices.append(M); gravities.append(g)
    diagonal=np.max([np.diag(M) for M in matrices],axis=0); gmax=np.max(np.abs(gravities),axis=0)
    scale=1/np.sqrt(diagonal)
    coupling=max(float(np.linalg.eigvalsh(M*scale[:,None]*scale[None,:])[-1]) for M in matrices)
    n['drive_design'].update(sample_poses=poses,coupling_factor=coupling)
    K=[]
    for k,j in enumerate(active):
        cfg=n['drives'][j['name']]; requested=copy.deepcopy(cfg); kind=j['type']; warnings=[]
        if cfg['mode']=='auto': cfg['mode']='position'
        inertia=float(diagonal[k]); bound=inertia*coupling
        record={'requested':requested,'inertia_diagonal_max':inertia,'inertia_damping_bound':bound,
                'sampled_gravity_max':float(gmax[k]),'warnings':warnings}
        if cfg['mode']=='position':
            tol=settings['position_error_rad' if kind=='revolute' else 'position_error_m']
            step=min(settings['design_step_rad' if kind=='revolute' else 'design_step_m'],0.1*(j['limits'][1]-j['limits'][0]))
            baseline=inertia*(2*math.pi*settings['frequency_hz'])**2
            gravity_gain=float(gmax[k])/tol
            effort_cap=settings['proportional_effort_fraction']*cfg['effort']/step
            if 'kp' not in cfg: cfg['kp']=min(max(baseline,gravity_gain),effort_cap); record['kp_source']='automatic'
            else: record['kp_source']='explicit'
            if cfg['kp']<=0: raise InputError(j['name']+': position kp must be positive')
            if 'kv' not in cfg:
                # Do not rely on passive damping: generic USD cannot author the
                # same viscous joint loss. Both engines get this conservative
                # active damping; supplied passive damping remains separate.
                cfg['kv']=2*settings['damping_ratio']*math.sqrt(cfg['kp']*bound); record['kv_source']='automatic'
            else: record['kv_source']='explicit'
            record.update(baseline_kp=baseline,gravity_kp=gravity_gain,effort_cap_kp=effort_cap,
                          estimated_gravity_deflection=float(gmax[k])/cfg['kp'])
            if gravity_gain>cfg['kp']: warnings.append('sampled static deflection exceeds design target; effort budget/given gains retained')
            if cfg['kv']+cfg.get('damping',0)<2*math.sqrt(cfg['kp']*bound): warnings.append('explicit damping below sampled critical envelope; oscillation checks required')
            cfg.setdefault('timeconst',settings['command_filter_s'])
        elif cfg['mode']=='velocity' and 'kv' not in cfg:
            cfg['kv']=2*settings['damping_ratio']*bound*2*math.pi*settings['frequency_hz']; record['kv_source']='automatic'
        if cfg['mode']!='passive' and gmax[k]>=cfg['effort']:
            warnings.append('sampled gravity reaches/exceeds effort limit; not sized for holding those poses')
        K.append(cfg.get('kp',0) if cfg['mode']=='position' else 0)
        record['resolved_SI']=copy.deepcopy(cfg); n['drive_design']['joints'][j['name']]=record
        for w in warnings: n['notes'].append(j['name']+': '+w)
    omega=0.
    for M in matrices:
        L=np.linalg.cholesky(M); invL=np.linalg.inv(L)
        omega=max(omega,math.sqrt(max(0,float(np.linalg.eigvalsh(invL@np.diag(K)@invL.T)[-1]))))
    dt=min(settings['max_timestep_s'],settings['modal_step_limit']/omega) if omega else settings['max_timestep_s']
    # Round downward at a fixed precision; never increase the stability cap.
    dt=math.floor(dt*1e9)/1e9
    if dt<1e-6: raise InputError('drive gains/inertia require timestep below 1 us; review physical parameters or explicit gains')
    n['drive_design'].update(timestep_s=dt,sampled_max_modal_frequency_rad_s=omega)
