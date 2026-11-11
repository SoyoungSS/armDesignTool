"""Reloaded resources, FK, mass, primitive geometry and conservative interference checks."""
import json, math, tempfile, shutil, sys, platform
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .math3d import tf, motion, forward, rot, rpy
from .geometry import read_stl

def nums(s): return np.array([float(v) for v in s.split()])
def xml_origin(node):
    o=node.find('origin'); return tf() if o is None else tf({'xyz_m':nums(o.get('xyz','0 0 0')),'rpy_rad':nums(o.get('rpy','0 0 0'))})

def cases(n):
    active=[j for j in n['model']['joints'] if j['type']!='fixed']
    result=[('geometric_zero',n['poses']['geometric_zero']),('initial',n['poses']['initial'])]
    result.extend((p['name'],p['q']) for p in n['poses']['acceptance'])
    for k in range(4): result.append(('sample_'+str(k),{j['name']:j['limits'][0]+(j['limits'][1]-j['limits'][0])*((.173+i*.271+k*.199)%1) for i,j in enumerate(active)}))
    return result

def validate_urdf(n,path,resource_root):
    root=ET.parse(path).getroot(); joints=root.findall('joint'); checks=0; meshes=0
    for node in root.findall('.//mesh'):
        fn=node.attrib['filename']; local=resource_root/('meshes/'+fn.split('/meshes/')[1] if fn.startswith('package://') else fn)
        tris=read_stl(local); assert len(tris)>0; meshes+=1
    for label,q in cases(n):
        poses={'world':np.eye(4)}
        for j in joints:
            name=j.get('name'); kind=j.get('type'); T=xml_origin(j)
            if kind!='fixed':
                source=next(x for x in n['model']['joints'] if x['name']==name)
                axis=nums(j.find('axis').get('xyz')); limits=j.find('limit')
                np.testing.assert_allclose(axis,source['axis'],atol=1e-10)
                np.testing.assert_allclose([float(limits.get('lower')),float(limits.get('upper'))],source['limits'],atol=1e-10)
                T=T@motion({'type':kind,'axis':axis},q[name])
            poses[j.find('child').get('link')]=poses[j.find('parent').get('link')]@T
        for link,expected in forward(n,q).items(): np.testing.assert_allclose(poses[link],expected,atol=2e-9); checks+=1
    for le in root.findall('link'):
        link=le.get('name')
        if link in n['geometry']:
            for p in n['geometry'][link]['parts']:
                vis=next(x for x in le.findall('visual') if x.get('name')==p['name'])
                np.testing.assert_allclose(xml_origin(vis),np.eye(4),atol=1e-12)
                assert vis.find('geometry/mesh').get('filename').endswith('/'+p['mesh'])
                if p['collision']:
                    co=next(x for x in le.findall('collision') if x.get('name')==p['name'])
                    np.testing.assert_allclose(xml_origin(co),p['T'],atol=1e-9)
                    geom=co.find('geometry')[0]
                    size=nums(geom.get('size')) if geom.tag=='box' else [float(geom.get('radius')),float(geom.get('length'))]
                    np.testing.assert_allclose(size,p.get('collision_size',p['size']),atol=1e-10)
        if link not in n['mass_properties']: continue
        p=n['mass_properties'][link]; ie=le.find('inertial'); assert ie is not None
        assert abs(float(ie.find('mass').get('value'))-p['mass_kg'])<1e-8
        np.testing.assert_allclose(xml_origin(ie)[:3,3],p['com_m'],atol=1e-9)
        a=ie.find('inertia').attrib; I=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
        np.testing.assert_allclose(I,p['inertia_kg_m2'],atol=1e-10)
    return {'mesh_loads':meshes,'link_pose_checks':checks,'mass_checks':len(n['mass_properties']),'poses':[x[0] for x in cases(n)]}

def mesh_checks(folder):
    count=0; triangles=0
    for file in folder.glob('*.stl'):
        t=read_stl(file); volume=np.einsum('ij,ij->i',t[:,0],np.cross(t[:,1],t[:,2])).sum()/6
        if volume<=0: raise ValueError('nonpositive oriented volume '+file.name)
        # Every triangle edge must pair with its reverse (closed, consistent winding).
        from collections import Counter
        edges=Counter()
        for tri in t:
            v=[tuple(x) for x in tri]
            for i in range(3): edges[(v[i],v[(i+1)%3])]+=1
        if any(edges[(b,a)]!=c for (a,b),c in edges.items()): raise ValueError('open/inconsistent mesh '+file.name)
        count+=1; triangles+=len(t)
    return {'files':count,'triangles':triangles,'closed_and_oriented':True}

def interference(n):
    # Broad phase only: transformed link-local AABBs, report candidates, never clearance proof.
    results=[]; witnesses=[]; links=list(n['geometry'])
    def interior(p,points,margin=1e-4):
        s=p.get('collision_size',p['size']); v=np.asarray(points)
        if p['collision_kind']=='box': return np.all(np.abs(v)<np.array(s)/2-margin,axis=1)
        return (np.sum(v[:,:2]**2,axis=1)<(s[0]-margin)**2)&(np.abs(v[:,2])<s[1]/2-margin)
    def samples(p):
        s=p.get('collision_size',p['size']); extent=np.array(s)/2 if p['collision_kind']=='box' else np.array([s[0],s[0],s[1]/2])
        pts=np.array([[x,y,z] for x in [-.7,0,.7] for y in [-.7,0,.7] for z in [-.7,0,.7]])*extent
        return pts[interior(p,pts)]
    for label,q in cases(n):
        poses=forward(n,q); boxes={}
        for link in links:
            low,high=np.array(n['geometry'][link]['bounds_m']); corners=np.array([[x,y,z] for x in [low[0],high[0]] for y in [low[1],high[1]] for z in [low[2],high[2]]]); T=poses[link]; V=corners@T[:3,:3].T+T[:3,3]; boxes[link]=(V.min(0),V.max(0))
        for i,a in enumerate(links):
            for k,b in enumerate(links[i+1:],i+1):
                overlap=np.minimum(boxes[a][1],boxes[b][1])-np.maximum(boxes[a][0],boxes[b][0])
                if np.all(overlap>1e-4):
                    results.append({'pose':label,'links':[a,b],'adjacent':k==i+1,'aabb_overlap_m':overlap.tolist(),'status':'possible_interference','note':'conservative AABB candidate; adjacent joint/guide mating volumes may intentionally overlap'})
                    for pa in n['geometry'][a]['parts']:
                        if not pa['collision']: continue
                        for pb in n['geometry'][b]['parts']:
                            if not pb['collision']: continue
                            A=poses[a]@np.array(pa['T']); B=poses[b]@np.array(pb['T']); found=False
                            for p1,p2,T in [(pa,pb,np.linalg.inv(B)@A),(pb,pa,np.linalg.inv(A)@B)]:
                                pts=samples(p1); transformed=pts@T[:3,:3].T+T[:3,3]
                                if np.any(interior(p2,transformed)): found=True; break
                            if found: witnesses.append({'pose':label,'links':[a,b],'parts':[pa['name'],pb['name']],'adjacent':k==i+1,'status':'collision_solid_overlap_witness','note':'sample lies strictly inside both collision primitives; joint/guide mating overlap may be intentional'})
    return {'method':'visual AABB broad phase plus sampled collision-primitive interior witnesses, including adjacent links','clearance_proven':False,'candidates':results,'solid_overlap_witnesses':witnesses}

def mujoco_check(n,path):
    import mujoco
    from .servo_validation import check_dynamics, check_servos
    m=mujoco.MjModel.from_xml_path(str(path)); d=mujoco.MjData(m); count=0
    driven=[j for j in n['model']['joints'] if j['type']!='fixed' and n['drives'][j['name']]['mode']!='passive']
    assert m.nu==len(driven), 'missing or extra actuators'
    for j in driven:
        cfg=n['drives'][j['name']]; a=m.actuator(j['name']+'_drive'); aid=a.id; mode=cfg['mode']
        assert a.trnid[0]==m.joint(j['name']).id
        assert m.actuator_forcelimited[aid] and m.actuator_ctrllimited[aid]
        np.testing.assert_allclose(a.forcerange,[-cfg['effort'],cfg['effort']],rtol=1e-9)
        limits=j['limits'] if mode=='position' else [-cfg['velocity'],cfg['velocity']] if mode=='velocity' else [-cfg['effort'],cfg['effort']]
        np.testing.assert_allclose(a.ctrlrange,limits,rtol=1e-9)
        gain=cfg['kp'] if mode=='position' else cfg['kv'] if mode=='velocity' else 1
        bias=[0,-cfg['kp'] if mode=='position' else 0,-cfg['kv'] if mode in ['position','velocity'] else 0]
        np.testing.assert_allclose(a.gainprm[0],gain,rtol=1e-9)
        np.testing.assert_allclose(a.biasprm[:3],bias,rtol=1e-9)
        if mode=='position' and cfg.get('timeconst',0)>0:
            assert m.actuator_dyntype[aid]==int(mujoco.mjtDyn.mjDYN_FILTEREXACT)
            np.testing.assert_allclose(a.dynprm[0],cfg['timeconst'],rtol=1e-9)
    for label,q in cases(n):
        mujoco.mj_resetData(m,d)
        for name,value in q.items(): d.qpos[m.joint(name).qposadr[0]]=value
        mujoco.mj_forward(m,d)
        for link,T in forward(n,q).items():
            if link=='tool_frame': pos=d.site('tool_frame').xpos; R=d.site('tool_frame').xmat.reshape(3,3)
            else: pos=d.body(link).xpos; R=d.body(link).xmat.reshape(3,3)
            np.testing.assert_allclose(pos,T[:3,3],atol=2e-8); np.testing.assert_allclose(R,T[:3,:3],atol=2e-8); count+=1
    for j in n['model']['joints']:
        if j['type']!='fixed': np.testing.assert_allclose(m.joint(j['name']).range,j['limits'],atol=1e-9)
    for link,p in n['mass_properties'].items():
        b=m.body(link); np.testing.assert_allclose(b.mass,[p['mass_kg']],atol=1e-9); np.testing.assert_allclose(b.ipos,p['com_m'],atol=1e-9)
        R=np.zeros(9); mujoco.mju_quat2Mat(R,b.iquat); R=R.reshape(3,3); np.testing.assert_allclose(R@np.diag(b.inertia)@R.T,p['inertia_kg_m2'],atol=1e-8)
    mujoco.mj_resetDataKeyframe(m,d,0)
    for j in driven:
        cfg=n['drives'][j['name']]; aid=m.actuator(j['name']+'_drive').id
        if cfg['mode']=='position':
            np.testing.assert_allclose(d.ctrl[aid],n['poses']['initial'][j['name']],atol=1e-10)
            adr=m.actuator_actadr[aid]
            if adr>=0: np.testing.assert_allclose(d.act[adr],d.ctrl[aid],atol=1e-10)
    for _ in range(50): mujoco.mj_step(m,d)
    if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)): raise ValueError('nonfinite short step')
    if np.any(d.warning.number): raise ValueError('MuJoCo short-step warnings: '+str(d.warning.number.tolist()))
    dynamics=check_dynamics(n,m,mujoco); servos=check_servos(n,m,mujoco)
    return {'version':mujoco.__version__,'link_pose_checks':count,'actuator_checks':len(driven),'steps':50,'timestep_s':m.opt.timestep,'finite':True,'mass_matrix_gravity_checks':dynamics,'servos':servos,'scope':'FK, dynamics consistency, limited-pose holds and smooth small motion; no global stability guarantee'}

def usd_check(n,path):
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf
    stage=Usd.Stage.Open(str(path)); assert stage and stage.GetDefaultPrim(); assert UsdGeom.GetStageMetersPerUnit(stage)==1; assert UsdGeom.GetStageUpAxis(stage)=='Z'
    expected=forward(n,n['poses']['geometric_zero'])
    material_checks=0
    for link,g in n['geometry'].items():
        for p in g['parts']:
            if 'material' not in p: continue
            prim=stage.GetPrimAtPath('/Robot/'+link+'/'+p['name']+'_visual')
            mesh=UsdGeom.Mesh(prim); nn=np.array(mesh.GetNormalsAttr().Get())
            assert mesh.GetNormalsInterpolation()=='vertex' and len(nn)==len(mesh.GetPointsAttr().Get())
            np.testing.assert_allclose(np.linalg.norm(nn,axis=1),1,atol=2e-6)
            bound,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            assert bound and str(bound.GetPath())=='/Robot/__materials/'+p['material']['name']
            shader=UsdShade.Shader(stage.GetPrimAtPath(str(bound.GetPath())+'/Surface'))
            np.testing.assert_allclose(shader.GetInput('diffuseColor').Get(),p['color'][:3],atol=1e-6)
            for key in ['roughness','metallic']: np.testing.assert_allclose(shader.GetInput(key).Get(),p['material'][key],atol=1e-6)
            material_checks+=1
    for filename,pose in [('robot_zero.usd','geometric_zero'),('robot.usd','initial')]:
        pose_stage=Usd.Stage.Open(str(path.with_name(filename))); cache=UsdGeom.XformCache()
        for link,T in forward(n,n['poses'][pose]).items():
            pp='/Robot/'+link if link!='tool_frame' else '/Robot/'+n['model']['joints'][-1]['child']+'/tool_frame'
            np.testing.assert_allclose(np.array(cache.GetLocalToWorldTransform(pose_stage.GetPrimAtPath(pp))).T,T,atol=2e-7)
    def R(q): return np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(q)))).T
    checks=0
    if not n['missing_physics']:
        for j in n['model']['joints']:
            prim=stage.GetPrimAtPath('/Robot/'+j['name'])
            if j['type']=='fixed':
                assert not UsdPhysics.DriveAPI.GetAll(prim); continue
            cfg=n['drives'][j['name']]; instance='angular' if j['type']=='revolute' else 'linear'; factor=180/math.pi if j['type']=='revolute' else 1
            drives=UsdPhysics.DriveAPI.GetAll(prim)
            if cfg['mode'] not in ['position','velocity']: assert not drives; continue
            assert len(drives)==1
            api=UsdPhysics.DriveAPI.Get(prim,instance)
            assert api.GetTypeAttr().Get()=='force'
            np.testing.assert_allclose(api.GetMaxForceAttr().Get(),cfg['effort'],rtol=2e-6)
            np.testing.assert_allclose(api.GetStiffnessAttr().Get()*factor,cfg['kp'] if cfg['mode']=='position' else 0,rtol=2e-6)
            np.testing.assert_allclose(api.GetDampingAttr().Get()*factor,cfg['kv'],rtol=2e-6)
            np.testing.assert_allclose(api.GetTargetPositionAttr().Get()/factor,n['poses']['initial'][j['name']] if cfg['mode']=='position' else 0,atol=2e-7)
        for pair in n.get('collision_exclusions',[]):
            assert '/Robot/'+pair['body2'] in [str(p) for p in UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath('/Robot/'+pair['body1'])).GetFilteredPairsRel().GetTargets()]
        for label,q in cases(n):
            poses={n['base_link']:tf(n['installation']['base_in_world'])}
            for j in n['model']['joints']:
                u=UsdPhysics.Joint(stage.GetPrimAtPath('/Robot/'+j['name'])); T0=tf(R=R(u.GetLocalRot0Attr().Get()),p=u.GetLocalPos0Attr().Get()); T1=tf(R=R(u.GetLocalRot1Attr().Get()),p=u.GetLocalPos1Attr().Get())
                M=motion({'type':j['type'],'axis':[0,0,1]},q.get(j['name'],0)); poses[j['child']]=poses[j['parent']]@T0@M@np.linalg.inv(T1)
                if j['type']!='fixed':
                    uj=UsdPhysics.RevoluteJoint(u) if j['type']=='revolute' else UsdPhysics.PrismaticJoint(u); factor=180/math.pi if j['type']=='revolute' else 1
                    np.testing.assert_allclose([uj.GetLowerLimitAttr().Get()/factor,uj.GetUpperLimitAttr().Get()/factor],j['limits'],atol=2e-7)
            poses['tool_frame']=poses[n['model']['joints'][-1]['child']]@tf(n['model']['tip'])
            for link,T in forward(n,q).items(): np.testing.assert_allclose(poses[link],T,atol=4e-7); checks+=1
        for link,p in n['mass_properties'].items():
            api=UsdPhysics.MassAPI(stage.GetPrimAtPath('/Robot/'+link)); np.testing.assert_allclose(api.GetMassAttr().Get(),p['mass_kg'],rtol=1e-6); np.testing.assert_allclose(api.GetCenterOfMassAttr().Get(),p['com_m'],atol=2e-7)
            RR=R(api.GetPrincipalAxesAttr().Get()); np.testing.assert_allclose(RR@np.diag(api.GetDiagonalInertiaAttr().Get())@RR.T,p['inertia_kg_m2'],atol=2e-7)
    return {'openusd_version':'.'.join(map(str,Usd.GetVersion())),'zero_pose_checks':len(expected),'initial_pose_checks':len(expected),'joint_frame_pose_checks':checks,'material_normal_checks':material_checks,'drive_attribute_checks':'passed' if not n['missing_physics'] else 'not_applicable','isaac_runtime':'pending','physics_authored':not n['missing_physics']}

def run_checks(n,run,targets,runtime=False):
    from .joint_layout import packing_checks
    checks={'module_packing':packing_checks(n),'mesh':mesh_checks(run/'meshes'),'interference':interference(n),'environment':{'python':sys.version.split()[0],'platform':platform.platform()}}
    for target,status in targets.items():
        if not status.get('generated'): continue
        status['status']='generated'; status.pop('error',None)
        if runtime and target=='mujoco': status['runtime_validation']='pending'
        try:
            if target in ['urdf','ros1']:
                base=run/'urdf_standalone' if target=='urdf' else run/'ros1'/(n['model']['name']+'_description')
                path=base/(n['model']['name']+'.urdf') if target=='urdf' else base/'urdf'/(n['model']['name']+'.urdf')
                with tempfile.TemporaryDirectory(prefix='arm-relocated-') as temp:
                    dest=Path(temp)/'relocated'; shutil.copytree(base,dest)
                    checks[target]=validate_urdf(n,dest/path.relative_to(base),dest); checks[target]['relocation']='passed'
                if target=='ros1':
                    ET.parse(base/'package.xml'); ET.parse(base/'launch'/'display.launch')
                    checks[target]['ros_runtime']='pending: run catkin/RViz checks in ROS1 Noetic environment'
            elif target=='mujoco':
                tree=ET.parse(run/'mujoco'/'robot.xml')
                for mesh in tree.findall('.//mesh'): read_stl(run/'mujoco'/'meshes'/mesh.get('file'))
                checks[target]={'xml_and_resources':'passed'}
                if runtime:
                    try:
                        with tempfile.TemporaryDirectory(prefix='mj-relocated-') as temp:
                            relocated=Path(temp)/'mujoco'; shutil.copytree(run/'mujoco',relocated)
                            checks[target]['runtime']=mujoco_check(n,relocated/'robot.xml')
                        checks[target]['relocation']='passed'; status['runtime_validation']='passed'
                        servo_status=checks[target]['runtime']['servos']['status']
                        if servo_status=='needs_review': status.update(status='failed',runtime_validation='failed',error='servo acceptance needs review; inspect validation.json runtime.servos')
                        elif servo_status=='pending': status['runtime_validation']='pending'
                    except ImportError: checks[target]['runtime']='pending: install mujoco'
                    except Exception as e:
                        status.update(runtime_validation='failed',status='failed',error=str(e)); checks[target]['runtime']={'error':str(e)}
            elif target=='isaac':
                with tempfile.TemporaryDirectory(prefix='usd-relocated-') as temp:
                    relocated=Path(temp)/'isaac'; shutil.copytree(run/'isaac',relocated)
                    checks[target]=usd_check(n,relocated/'robot.usd'); checks[target]['relocation']='passed'
            status['static_checks']='passed'
        except Exception as e:
            status['static_checks']='failed'; status['status']='failed'; status['error']=str(e); checks[target]={'error':str(e)}
            if target=='mujoco' and runtime: status['runtime_validation']='failed'
        if status.get('status')!='failed': status['status']='runtime_validated' if status['runtime_validation']=='passed' else ('failed' if status['runtime_validation']=='failed' else 'static_checked')
    packing=checks.get('module_packing',{})
    for target,status in targets.items():
        if not status.get('generated'): continue
        status['assembly_validation']=packing.get('status','not_applicable')
        if packing.get('conflicts'):
            status.update(status='failed',error='motor envelope conflict; inspect validation.json/module_packing')
        elif packing.get('unresolved') and status.get('status')!='failed':
            status.update(status='pending',reason='coincident joint mechanical arrangement needs review')
    return checks
