"""SI/right-handed transforms; column vectors, Rz Ry Rx, scalar-first quaternions."""
import numpy as np
import math

def rot(a, q):
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    x,y,z = a; K = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3) + math.sin(q)*K + (1-math.cos(q))*(K@K)

def rpy(v):
    return rot([0,0,1],v[2]) @ rot([0,1,0],v[1]) @ rot([1,0,0],v[0])

def to_rpy(R):
    p = math.asin(float(np.clip(-R[2,0],-1,1)))
    if abs(math.cos(p)) > 1e-9:
        return [math.atan2(R[2,1],R[2,2]),p,math.atan2(R[1,0],R[0,0])]
    return [0,p,math.atan2(-R[0,1],R[1,1])]

def tf(origin=None, R=None, p=None):
    T=np.eye(4)
    if origin is not None: R=rpy(origin['rpy_rad']); p=origin['xyz_m']
    if R is not None: T[:3,:3]=R
    if p is not None: T[:3,3]=p
    return T

def align_z(a):
    z=np.array(a,dtype=float,copy=True); z/=np.linalg.norm(z)
    ref=np.eye(3)[np.argmin(np.abs(z))]; x=np.cross(ref,z); x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))

def quat(R):
    # Stable matrix->quaternion, including pi rotations.
    vals,vecs=np.linalg.eigh(np.array([
      [R[0,0]-R[1,1]-R[2,2],R[0,1]+R[1,0],R[0,2]+R[2,0],R[2,1]-R[1,2]],
      [R[0,1]+R[1,0],R[1,1]-R[0,0]-R[2,2],R[1,2]+R[2,1],R[0,2]-R[2,0]],
      [R[0,2]+R[2,0],R[1,2]+R[2,1],R[2,2]-R[0,0]-R[1,1],R[1,0]-R[0,1]],
      [R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1],np.trace(R)]])/3)
    q=vecs[:,np.argmax(vals)][[3,0,1,2]]
    return q if q[0]>=0 else -q

def motion(j,q):
    if j['type']=='revolute': return tf(R=rot(j['axis'],q))
    if j['type']=='prismatic': return tf(p=np.asarray(j['axis'])*q)
    return np.eye(4)

def forward(n,q,world=True):
    T=tf(n['installation']['base_in_world']) if world else np.eye(4)
    out={n['base_link']:T.copy()}
    for j in n['model']['joints']:
        T=T@tf(j['origin'])@motion(j,q.get(j['name'],0)); out[j['child']]=T.copy()
    out['tool_frame']=T@tf(n['model']['tip'])
    return out

def serializable(x):
    if isinstance(x,np.ndarray): return x.tolist()
    if isinstance(x,np.generic): return x.item()
    if isinstance(x,dict): return {k:serializable(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)): return [serializable(v) for v in x]
    return x
