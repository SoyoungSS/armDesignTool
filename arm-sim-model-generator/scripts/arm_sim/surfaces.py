"""Native deterministic ring surfaces: no CAD, boolean or remeshing dependency."""
import math
import numpy as np

POLICY='rounded-shell-v2'
PALETTE={
    'body':([.82,.84,.85,1],.36,.10),
    'joint':([.16,.18,.20,1],.40,.15),
    'accent':([.06,.36,.43,1],.32,.20),
    'metal':([.55,.58,.61,1],.24,.72),
}

def loft(rings, exponent=3.2, segments=48):
    """Closed positive-volume solid. Rings=(z,width,depth,center_x,center_y).

    Equal samples on a superellipse; all radii positive, no duplicate pole rings.
    Endpoint center fans keep the caps closed without sliver triangulation.
    """
    a=np.arange(segments)*2*math.pi/segments
    c=np.cos(a); s=np.sin(a)
    xy=np.column_stack((np.sign(c)*np.abs(c)**(2/exponent),np.sign(s)*np.abs(s)**(2/exponent)))
    V=[]; F=[]
    for z,w,d,x,y in rings: V.extend(np.column_stack((xy[:,0]*w/2+x,xy[:,1]*d/2+y,np.full(segments,z))))
    for k in range(len(rings)-1):
        for i in range(segments):
            a=k*segments+i; b=k*segments+(i+1)%segments
            F.extend([[a,b,b+segments],[a,b+segments,a+segments]])
    for k,reverse in [(0,True),(len(rings)-1,False)]:
        z,_,_,x,y=rings[k]; center=len(V); V.append([x,y,z])
        for i in range(segments):
            a=k*segments+i; b=k*segments+(i+1)%segments
            F.append([center,b,a] if reverse else [center,a,b])
    return np.array(V),np.array(F,dtype=int)

def rounded(kind,size,profile=None):
    if kind=='cylinder': w=d=2*size[0]; h=size[1]; exponent=2.
    else: w,d,h=size; exponent=3.4
    # Round the silhouette at axial shoulders, independently of tessellation.
    r=min(w,d,h)*.16
    zs=[-h/2,-h/2+r*.134,-h/2+r*.5,-h/2+r,h/2-r,h/2-r*.5,h/2-r*.134,h/2]
    inset=[r,r*.5,r*.134,0,0,r*.134,r*.5,r]
    rings=[(z,w-2*b,d-2*b,0,0) for z,b in zip(zs,inset)]
    if profile is not None:
        rings=[((t-.5)*h,w*s,d*s,0,0) for t,s in profile]
    return loft(rings,exponent)

def beam_frame(endpoint,up_axis):
    z=np.array(endpoint,dtype=float,copy=True); z/=np.linalg.norm(z)
    y=np.array(up_axis,dtype=float,copy=True); y-=z*np.dot(z,y)
    if np.linalg.norm(y)<1e-8:
        y=np.eye(3)[np.argmin(np.abs(z))]; y-=z*np.dot(z,y)
    y/=np.linalg.norm(y); x=np.cross(y,z)
    return np.column_stack((x,y,z))

def normals(vertices,faces):
    """Area weighted smooth normals, only for visual shading; no geometry change."""
    V=np.asarray(vertices); F=np.asarray(faces); tri=V[F]
    face=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); out=np.zeros_like(V)
    for i in range(3): np.add.at(out,F[:,i],face)
    return out/np.maximum(np.linalg.norm(out,axis=1)[:,None],1e-30)

def proportion_plan(n):
    js=n['model']['joints']; rules=n['dimensions'].get('rules',{})
    spans=[float(np.linalg.norm(j['origin']['xyz_m'])) for j in js]+[float(np.linalg.norm(n['model']['tip']['xyz_m']))]
    reach=max(sum(spans),.12); minw=rules.get('min_width_m',.025); maxw=rules.get('max_width_m',.10)
    local_ratio=rules.get('width_ratio',.16)
    # Global support envelope, gradually reducing toward wrist. A zero origin
    # does not imply a zero-sized motor. Explicit dimensions still win later.
    widths=[float(np.clip(max(reach*.085*(1-.44*i/max(1,len(js))),s*local_ratio),minw,maxw)) for i,s in enumerate(spans)]
    caps=[]
    for i in range(len(spans)):
        before=next((v for v in reversed(spans[:i]) if v>1e-6),float('inf'))
        after=next((v for v in spans[i:] if v>1e-6),float('inf'))
        caps.append(.90*min(before,after))
    return {'policy':POLICY,'reach_scale_m':reach,'widths_m':widths,'module_diameter_caps_m':[v if math.isfinite(v) else None for v in caps],
            'basis':'whole-chain reach, root-to-tip taper and local span; explicit dimensions take precedence'}

def template(up,down,end,diameter):
    if up is None: return 'pedestal'
    if up['type']=='prismatic': return 'linear_carriage'
    if up['type']=='fixed': return 'fixed_adapter'
    if down is None: return 'tool_wrist'
    if down['type']=='prismatic': return 'linear_support'
    from .math3d import tf
    a=np.array(up.get('axis',[0,0,1])); b=tf(down['origin'])[:3,:3]@np.array(down.get('axis',[0,0,1]))
    span=np.linalg.norm(end); parallel=abs(np.dot(a,b))>.94
    if span<diameter*.35: return 'coaxial_wrist' if parallel else 'cross_axis_wrist'
    if parallel: return 'offset_parallel' if np.linalg.norm(np.cross(a,end))>span*.2 else 'axial_spindle'
    return 'elbow_shell'
