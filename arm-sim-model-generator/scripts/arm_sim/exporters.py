"""All exporters consume the same normalized model and generated part geometry."""
import json, math, shutil
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from .math3d import tf, quat, to_rpy, align_z, forward
from .surfaces import PALETTE, normals

def fmt(v):
    if isinstance(v,(list,tuple,np.ndarray)): return ' '.join(format(float(x),'.12g') for x in v)
    return str(v)

def el(parent,tag,**attrs): return ET.SubElement(parent,tag,{k:fmt(v) for k,v in attrs.items()})
def xml_write(root,path):
    ET.indent(root,space='  '); path.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def origin(parent,T): return el(parent,'origin',xyz=T[:3,3],rpy=to_rpy(T[:3,:3]))

def copy_meshes(out,source): shutil.copytree(source,out/'meshes',dirs_exist_ok=True)

def urdf(n,g,out,source,package=None):
    out.mkdir(parents=True,exist_ok=True); copy_meshes(out,source)
    root=ET.Element('robot',{'name':n['model']['name']}); el(root,'link',name='world')
    for link,data in g.items():
        le=el(root,'link',name=link)
        if link in n['mass_properties']:
            p=n['mass_properties'][link]; ie=el(le,'inertial'); origin(ie,tf(p=p['com_m'])); el(ie,'mass',value=p['mass_kg'])
            I=np.array(p['inertia_kg_m2']); el(ie,'inertia',ixx=I[0,0],iyy=I[1,1],izz=I[2,2],ixy=I[0,1],ixz=I[0,2],iyz=I[1,2])
        for part in data['parts']:
            vis=el(le,'visual',name=part['name']); geo=el(vis,'geometry')
            prefix='package://'+package+'/meshes/' if package else 'meshes/'
            el(geo,'mesh',filename=prefix+part['mesh']); mat=el(vis,'material',name=link+'_'+part['name']); el(mat,'color',rgba=part['color'])
            if part['collision']:
                cs=part.get('collision_size',part['size'])
                co=el(le,'collision',name=part['name']); origin(co,part['T']); geo=el(co,'geometry')
                if part['collision_kind']=='cylinder': el(geo,'cylinder',radius=cs[0],length=cs[1])
                else: el(geo,'box',size=cs)
    el(root,'link',name='tool_frame')
    joints=[{'name':'world_fixed','parent':'world','child':n['base_link'],'type':'fixed','origin':n['installation']['base_in_world']}]+n['model']['joints']+[{'name':'tool_fixed','parent':n['model']['joints'][-1]['child'],'child':'tool_frame','type':'fixed','origin':n['model']['tip']}]
    for j in joints:
        je=el(root,'joint',name=j['name'],type=j['type']); el(je,'parent',link=j['parent']); el(je,'child',link=j['child']); origin(je,tf(j['origin']))
        if j['type']!='fixed':
            cfg=n['drives'][j['name']]; el(je,'axis',xyz=j['axis']); el(je,'limit',lower=j['limits'][0],upper=j['limits'][1],effort=cfg['effort'],velocity=cfg['velocity'])
            el(je,'dynamics',damping=cfg.get('damping',0),friction=cfg.get('friction',0))
    path=out/('urdf' if package else '')/(n['model']['name']+'.urdf'); xml_write(root,path)
    return path

def ros1(n,g,out,source):
    package=n['model']['name']+'_description'; out=out/package; urdf(n,g,out,source,package)
    (out/'package.xml').write_text(f'''<?xml version="1.0"?>
<package format="2"><name>{package}</name><version>0.1.0</version>
<description>Generated fixed-base serial arm description</description>
<maintainer email="owner@example.invalid">Model owner</maintainer><license>BSD-3-Clause</license>
<buildtool_depend>catkin</buildtool_depend><exec_depend>robot_state_publisher</exec_depend>
<exec_depend>joint_state_publisher</exec_depend><exec_depend>joint_state_publisher_gui</exec_depend><exec_depend>rviz</exec_depend></package>
''',encoding='utf-8')
    (out/'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.0.2)
project({package})
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf meshes launch rviz DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}})
''',encoding='utf-8')
    launch=ET.Element('launch'); el(launch,'arg',name='gui',default='true'); el(launch,'arg',name='rviz',default='true')
    el(launch,'param',name='robot_description',textfile=f'$(find {package})/urdf/{n["model"]["name"]}.urdf')
    for gui in [True,False]:
        attrs={'if' if gui else 'unless':'$(arg gui)'}
        node=el(launch,'node',pkg='joint_state_publisher_gui' if gui else 'joint_state_publisher',type='joint_state_publisher_gui' if gui else 'joint_state_publisher',name='joint_state_publisher',**attrs)
        el(node,'rosparam',param='zeros').text=json.dumps(n['poses']['initial'])
    el(launch,'node',pkg='robot_state_publisher',type='robot_state_publisher',name='robot_state_publisher')
    el(launch,'node',pkg='rviz',type='rviz',name='rviz',args=f'-d $(find {package})/rviz/display.rviz',**{'if':'$(arg rviz)'})
    xml_write(launch,out/'launch'/'display.launch'); (out/'rviz').mkdir(exist_ok=True)
    (out/'rviz'/'display.rviz').write_text(f'''Panels:
  - Class: rviz/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: {n['base_link']}
    Background Color: 40; 43; 48
  Displays:
    - Class: rviz/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.1
    - Class: rviz/RobotModel
      Name: RobotModel
      Enabled: true
      Robot Description: robot_description
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz/TF
      Name: TF
      Enabled: false
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 1.5
      Pitch: 0.5
      Yaw: 0.7
Window Geometry:
  Width: 1200
  Height: 800
''',encoding='utf-8')
    return out

def mjcf(n,g,out,source):
    out.mkdir(parents=True,exist_ok=True); copy_meshes(out,source)
    root=ET.Element('mujoco',{'model':n['model']['name']}); el(root,'compiler',angle='radian',meshdir='meshes',inertiafromgeom='false',fusestatic='false')
    el(root,'option',gravity=n['installation']['gravity_world_m_s2'],timestep=n['drive_design']['timestep_s'],integrator='implicitfast')
    asset=el(root,'asset'); world=el(root,'worldbody'); bodies={}
    for name,(rgba,roughness,metallic) in PALETTE.items():
        el(asset,'material',name=name,rgba=rgba,specular=.2+.4*metallic,shininess=1-roughness)
    for link,data in g.items():
        if link==n['base_link']: parent=world; T=tf(n['installation']['base_in_world']); j=None
        else:
            j=next(j for j in n['model']['joints'] if j['child']==link); parent=bodies[j['parent']]; T=tf(j['origin'])
        b=el(parent,'body',name=link,pos=T[:3,3],quat=quat(T[:3,:3])); bodies[link]=b
        if link in n['mass_properties']:
            p=n['mass_properties'][link]; I=np.array(p['inertia_kg_m2'])
            eig,R=np.linalg.eigh(I)
            if np.linalg.det(R)<0: R[:,0]*=-1
            el(b,'inertial',pos=p['com_m'],mass=p['mass_kg'],diaginertia=eig,quat=quat(R))
        if j and j['type']!='fixed':
            cfg=n['drives'][j['name']]
            el(b,'joint',name=j['name'],type='hinge' if j['type']=='revolute' else 'slide',axis=j['axis'],limited='true',range=j['limits'],damping=cfg.get('damping',0),frictionloss=cfg.get('friction',0))
        for p in data['parts']:
            key=link+'_'+p['name']; el(asset,'mesh',name=key,file=p['mesh'])
            el(b,'geom',name=key+'_visual',type='mesh',mesh=key,material=p['material']['name'],contype=0,conaffinity=0,group=2,mass=0)
            if p['collision']:
                kind=p['collision_kind']; cs=p.get('collision_size',p['size']); size=np.array(cs)/2 if kind=='box' else [cs[0],cs[1]/2]
                el(b,'geom',name=key+'_collision',type=kind,size=size,pos=p['T'][:3,3],quat=quat(p['T'][:3,:3]),rgba=[.3,.3,.3,.2],group=3,mass=0)
    tip=tf(n['model']['tip']); el(bodies[n['model']['joints'][-1]['child']],'site',name='tool_frame',pos=tip[:3,3],quat=quat(tip[:3,:3]),size=.006,rgba=[.1,.7,.8,1])
    contact=el(root,'contact')
    for pair in n.get('collision_exclusions',[]): el(contact,'exclude',body1=pair['body1'],body2=pair['body2'])
    act=el(root,'actuator'); active=[j for j in n['model']['joints'] if j['type']!='fixed']
    for j in active:
        cfg=n['drives'][j['name']]; mode=cfg['mode']
        if mode=='passive': continue
        opts={'name':j['name']+'_drive','joint':j['name'],'forcelimited':'true','forcerange':[-cfg['effort'],cfg['effort']],'ctrllimited':'true'}
        if mode=='position': el(act,'position',**opts,kp=cfg['kp'],kv=cfg['kv'],ctrlrange=j['limits'],timeconst=cfg.get('timeconst',0))
        elif mode=='velocity': el(act,'velocity',**opts,kv=cfg['kv'],ctrlrange=[-cfg['velocity'],cfg['velocity']])
        else: el(act,'motor',**opts,ctrlrange=[-cfg['effort'],cfg['effort']],gear=1)
    if not len(act): root.remove(act)
    keys=el(root,'keyframe'); ctrl=[n['poses']['initial'][j['name']] if n['drives'][j['name']]['mode']=='position' else 0 for j in active if n['drives'][j['name']]['mode']!='passive']
    activation=[n['poses']['initial'][j['name']] for j in active if n['drives'][j['name']]['mode']=='position' and n['drives'][j['name']].get('timeconst',0)>0]
    el(keys,'key',name='initial',qpos=[n['poses']['initial'][j['name']] for j in active],**({'ctrl':ctrl} if ctrl else {}),**({'act':activation} if activation else {}))
    xml_write(root,out/'robot.xml')
    shutil.copyfile(Path(__file__).resolve().parents[1]/'mujoco_run.py',out/'run.py')
    return out/'robot.xml'

def usd(n,g,out,source):
    # OpenUSD offline path. No Isaac/PhysX runtime claims. Vertices embedded in USD,
    # since USD does not natively consume STL as a Mesh prim.
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
    out.mkdir(parents=True,exist_ok=True)
    stage=Usd.Stage.CreateNew(str(out/'robot_zero.usd')); UsdGeom.SetStageMetersPerUnit(stage,1); UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    root=UsdGeom.Xform.Define(stage,'/Robot'); stage.SetDefaultPrim(root.GetPrim())
    materials={}
    for name,(rgba,roughness,metallic) in PALETTE.items():
        mat=UsdShade.Material.Define(stage,'/Robot/__materials/'+name)
        shader=UsdShade.Shader.Define(stage,'/Robot/__materials/'+name+'/Surface'); shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgba[:3]))
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface'); materials[name]=mat
    dynamic=not n['missing_physics']
    def qgf(R):
        q=quat(R); return Gf.Quatf(float(q[0]),Gf.Vec3f(*map(float,q[1:])))
    def setpose(x,T):
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float,T[:3,3])))
        q=quat(T[:3,:3]); x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))
    scene=UsdPhysics.Scene.Define(stage,'/PhysicsScene'); gravity=np.array(n['installation']['gravity_world_m_s2']); norm=np.linalg.norm(gravity)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(*map(float,gravity/norm if norm else [0,0,-1]))); scene.CreateGravityMagnitudeAttr(float(norm))
    poses=forward(n,{j['name']:0 for j in n['model']['joints'] if j['type']!='fixed'})
    for link,data in g.items():
        x=UsdGeom.Xform.Define(stage,'/Robot/'+link); setpose(x,poses[link])
        if dynamic:
            UsdPhysics.RigidBodyAPI.Apply(x.GetPrim()); mass=UsdPhysics.MassAPI.Apply(x.GetPrim()); p=n['mass_properties'][link]
            mass.CreateMassAttr(float(p['mass_kg'])); mass.CreateCenterOfMassAttr(Gf.Vec3f(*p['com_m']))
            eig,R=np.linalg.eigh(p['inertia_kg_m2'])
            if np.linalg.det(R)<0: R[:,0]*=-1
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,eig))); mass.CreatePrincipalAxesAttr(qgf(R))
        for p in data['parts']:
            mesh=UsdGeom.Mesh.Define(stage,'/Robot/'+link+'/'+p['name']+'_visual')
            mesh.CreateNormalsAttr([Gf.Vec3f(*map(float,v)) for v in normals(p['vertices'],p['faces'])]); mesh.SetNormalsInterpolation('vertex')
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[p['material']['name']])
            mesh.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in p['vertices']]); mesh.CreateFaceVertexCountsAttr([3]*len(p['faces'])); mesh.CreateFaceVertexIndicesAttr(p['faces'].flatten().tolist()); mesh.CreateSubdivisionSchemeAttr('none'); mesh.CreateDisplayColorAttr([Gf.Vec3f(*p['color'][:3])])
            if p['collision']:
                path='/Robot/'+link+'/'+p['name']+'_collision'; cs=p.get('collision_size',p['size'])
                if p['collision_kind']=='box':
                    co=UsdGeom.Cube.Define(stage,path); co.CreateSizeAttr(1); setpose(co,p['T']); co.AddScaleOp().Set(Gf.Vec3f(*map(float,cs)))
                else:
                    co=UsdGeom.Cylinder.Define(stage,path); co.CreateRadiusAttr(cs[0]); co.CreateHeightAttr(cs[1]); co.CreateAxisAttr('Z'); setpose(co,p['T'])
                co.CreatePurposeAttr('guide')
                if dynamic: UsdPhysics.CollisionAPI.Apply(co.GetPrim())
    tip=UsdGeom.Xform.Define(stage,'/Robot/'+n['model']['joints'][-1]['child']+'/tool_frame'); setpose(tip,tf(n['model']['tip']))
    mapping={}
    if dynamic:
        fj=UsdPhysics.FixedJoint.Define(stage,'/Robot/world_fixed'); fj.CreateBody1Rel().SetTargets([Sdf.Path('/Robot/'+n['base_link'])])
        T=tf(n['installation']['base_in_world']); fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float,T[:3,3]))); fj.CreateLocalRot0Attr(qgf(T[:3,:3])); UsdPhysics.ArticulationRootAPI.Apply(fj.GetPrim())
        for j in n['model']['joints']:
            cls={'revolute':UsdPhysics.RevoluteJoint,'prismatic':UsdPhysics.PrismaticJoint,'fixed':UsdPhysics.FixedJoint}[j['type']]
            joint=cls.Define(stage,'/Robot/'+j['name']); joint.CreateBody0Rel().SetTargets([Sdf.Path('/Robot/'+j['parent'])]); joint.CreateBody1Rel().SetTargets([Sdf.Path('/Robot/'+j['child'])])
            T=tf(j['origin']); A=align_z(j['axis']) if j['type']!='fixed' else np.eye(3)
            joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,T[:3,3]))); joint.CreateLocalPos1Attr(Gf.Vec3f(0)); joint.CreateLocalRot0Attr(qgf(T[:3,:3]@A)); joint.CreateLocalRot1Attr(qgf(A))
            joint.CreateCollisionEnabledAttr(False)
            if j['type']!='fixed':
                factor=180/math.pi if j['type']=='revolute' else 1
                joint.CreateAxisAttr('Z'); joint.CreateLowerLimitAttr(j['limits'][0]*factor); joint.CreateUpperLimitAttr(j['limits'][1]*factor)
                cfg=n['drives'][j['name']]
                if cfg['mode'] in ['position','velocity']:
                    dr=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular' if j['type']=='revolute' else 'linear'); dr.CreateTypeAttr('force'); dr.CreateMaxForceAttr(cfg['effort'])
                    dr.CreateStiffnessAttr(cfg.get('kp',0)/factor if cfg['mode']=='position' else 0); dr.CreateDampingAttr(cfg['kv']/factor)
                    dr.CreateTargetPositionAttr(n['poses']['initial'][j['name']]*factor if cfg['mode']=='position' else 0); dr.CreateTargetVelocityAttr(0)
                elif cfg['mode']=='effort': n['notes'].append(j['name']+': USD effort mode requires runtime effort command; no position drive authored')
                if cfg.get('damping',0) or cfg.get('friction',0): n['notes'].append(j['name']+': USD joint damping/friction require Isaac runtime settings; retained in runtime_config.json')
            mapping[j['name']]=str(joint.GetPath())
    if dynamic:
        for pair in n.get('collision_exclusions',[]):
            api=UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/Robot/'+pair['body1']))
            api.CreateFilteredPairsRel().AddTarget(Sdf.Path('/Robot/'+pair['body2']))
    stage.GetRootLayer().Save()
    # Keep zero geometry in its own layer. The entry layer starts at the initial
    # configuration without changing either endpoint's zero joint frame.
    entry=Usd.Stage.CreateNew(str(out/'robot.usd')); entry.GetRootLayer().subLayerPaths=['robot_zero.usd']
    entry.SetDefaultPrim(entry.GetPrimAtPath('/Robot')); UsdGeom.SetStageMetersPerUnit(entry,1); UsdGeom.SetStageUpAxis(entry,UsdGeom.Tokens.z)
    for link,T in forward(n,n['poses']['initial']).items():
        if link=='tool_frame': continue
        x=UsdGeom.Xformable(entry.GetPrimAtPath('/Robot/'+link))
        for op in x.GetOrderedXformOps():
            if op.GetOpType()==UsdGeom.XformOp.TypeTranslate: op.Set(Gf.Vec3d(*map(float,T[:3,3])))
            elif op.GetOpType()==UsdGeom.XformOp.TypeOrient:
                q=quat(T[:3,:3]); op.Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))
    entry.GetRootLayer().Save()
    (out/'runtime_config.json').write_text(json.dumps({'initial_joint_positions_SI':n['poses']['initial'],'joint_paths':mapping,'drives':n['drives'],'drive_design':n.get('drive_design'),'recommended_physics_dt_s':n.get('drive_design',{}).get('timestep_s',.001),'command_filter':'apply the same per-drive timeconst in your runtime target generator; USD DriveAPI has no input-filter attribute','fixed_base':True,'merge_fixed_joints':False,'collision_approximation':'analytic primitives','requested_isaac_version':n['export'].get('runtime_versions',{}).get('isaac'),'creation':'OpenUSD offline; initialize q through Isaac articulation API before first physics step, do not offset authored zero frames'},indent=2),encoding='utf-8')
    return out/'robot.usd'
