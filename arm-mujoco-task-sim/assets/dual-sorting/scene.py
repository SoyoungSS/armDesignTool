"""Assemble the untouched seven-axis mechanism and the sorting assets."""
from pathlib import Path
import shutil
import numpy as np
from runtime import mj
import hashlib
import json
import xml.etree.ElementTree as ET

OPEN = .036
TRAY_Y = .120
SLOTS = (-.044, .044)


def box(x, y, z, a, b, c):
    return ("box", (x, y, z), (a, b, c))


def cylinder(x, y, z, r, h):
    return ("cylinder", (x, y, z), (r, h))


# Each dimension is in metres. Heights are measured from the supporting face.
OBJECTS = [
    ("round_can", "red", .044, .026, [cylinder(0, 0, .028, .022, .028)]),
    ("stepped_block", "red", .036, .044, [box(0, 0, .011, .027, .023, .011),
        box(0, 0, .041, .018, .017, .019)]),
    ("block", "blue", .038, .026, [box(0, 0, .028, .019, .024, .028)]),
    ("twin_posts", "blue", .038, .028, [cylinder(0, -.013, .029, .019, .029),
        cylinder(0, .019, .020, .014, .020)]),

]


def vals(seq):
    return " ".join(f"{v:.9g}" for v in seq)


def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {
        key: value if isinstance(value, str) else vals(value) if isinstance(value, (list, tuple))
        else str(value) for key, value in attrs.items()
    })


def inertia(parent, mass, center, halfsize):
    a, b, c = halfsize
    add(parent, "inertial", pos=center, mass=mass,
        diaginertia=tuple(mass/3*s for s in (b*b+c*c, a*a+c*c, a*a+b*b)))


def build(source, project, arm):
    source, project = Path(source).resolve(), Path(project).resolve()
    root = ET.parse(source).getroot()
    model = mj.MjModel.from_xml_path(str(source))
    key = root.find("keyframe/key[@name='initial']")
    home = np.fromstring(key.get('qpos'), sep=' ')
    tool = root.find(".//site[@name='tool_frame']")
    link = next(body for body in root.iter('body') if tool in list(body))
    # Pick a task posture within source joint limits. A rigid tool adapter
    # points its pad approach axis down in this mounting configuration.
    if arm == 'worksheet':
        home = np.array([-.65,-.42,.25,1.10,.12,.65,.12])
    d = mj.MjData(model)
    d.qpos[:] = home
    mj.mj_forward(model,d)
    tcp = d.site('tool_frame').xpos.copy()
    wrist = d.body(link.get('name')).xmat.reshape(3,3)
    adapter_rot = wrist.T @ np.diag([1.,-1.,-1.])
    adapter_quat = np.empty(4)
    mj.mju_mat2Quat(adapter_quat, np.ascontiguousarray(adapter_rot).ravel())
    cx, cy = map(float,tcp[:2])
    table = float(tcp[2]-.111-.13)
    if arm == 'worksheet':
        cx -= .030
        cy += .05
        table += .038
    if arm == 'fr3':
        # Register the table to the forward workspace, outside the base shell.
        cx = .46
        cy = .04
        table = .32
    scene = project/'scene/scene.xml'
    scene.parent.mkdir(parents=True,exist_ok=True)
    meshes = scene.parent/'meshes'
    meshes.mkdir(exist_ok=True)
    compiler = root.find('compiler')
    meshdir = source.parent/compiler.get('meshdir','.')
    for node in root.findall('asset/mesh[@file]'):
        mesh = meshdir/node.get('file')
        shutil.copy2(mesh,meshes/mesh.name)
        node.set('file',mesh.name)
    compiler.set('meshdir','meshes')
    root.set("model", arm+" tabletop sorting")
    world, asset = root.find("worldbody"), root.find("asset")
    visual = add(root, "visual")
    add(visual, "global", azimuth=135, elevation=-28)
    add(visual, "map", znear=.015)
    add(root, "statistic", center=(cx, cy, table+.10), extent=.85)
    add(world, "light", pos=(.9, -.6, 1.65), dir=(-.45, .30, -1), diffuse=(.8, .8, .8))
    add(world, "camera", name="sorting", pos=(1.48, -.58, 1.09),
        xyaxes=(.4472, .8944, 0, -.4025, .2012, .893), fovy=47)

    def geom(parent, name, pos, size, color, **extra):
        add(parent, "geom", name=name, type="box", pos=pos, size=size, rgba=color, **extra)

    add(world, "geom", name="floor", pos=(0,0,min(table-.23,-.12)), type="plane", size=(0, 0, .06), rgba=(.28, .32, .36, 1))
    geom(world, "table_top", (cx,cy,table-.012), (.135,.225,.012), (.71,.63,.51,1))
    for side, y, color in (
        ("left",cy-TRAY_Y,(.52,.14,.13,1)),("right",cy+TRAY_Y,(.10,.32,.57,1))
    ):
        geom(world,f"tray_{side}_base",(cx,y,table+.0045),(.105,.048,.0045),color, solref=(.008,1), solimp=(.98,.995,.001),priority=1)
        for sign in (-1,1):
            geom(world,f"tray_{side}_side_{sign}",(cx,y+sign*.046,table+.015),(.105,.002,.006),color)
            geom(world,f"tray_{side}_end_{sign}",(cx+sign*.103,y,table+.015),(.002,.044,.006),color)
    palm = add(link, "body", name="gripper", pos=tool.get('pos','0 0 0'), quat=vals(adapter_quat))
    inertia(palm, .22, (0, 0, .026), (.056, .023, .026))
    geom(palm, "gripper_mount", (0, 0, .012), (.028, .022, .012), (.23, .28, .32, 1))
    geom(palm, "gripper_palm", (0, 0, .039), (.055, .022, .014), (.20, .24, .28, 1))
    add(palm, "site", name="grip_tcp", pos=(0, 0, .111), size=.002, rgba=(1, .8, .3, 0))
    actuators = root.find("actuator")
    for side, sign in (("a", -1), ("b", 1)):
        finger = add(palm, "body", name=f"finger_{side}", pos=(sign*.010, 0, .053))
        inertia(finger, .035, (sign*.005, 0, .028), (.005, .012, .032))
        add(finger, "joint", name=f"jaw_{side}", type="slide", axis=(sign, 0, 0),
            range=(0, OPEN), limited="true", damping=2, armature=.004)
        geom(finger, f"finger_stem_{side}", (sign*.014, 0, .026), (.004, .010, .026),
             (.72, .74, .76, 1))
        geom(finger, f"pad_{side}", (sign*.005, 0, .058), (.005, .020, .012),
             (.12, .15, .17, 1), friction=(2.2, .020, .002), condim=6, priority=2, solref=(.008,1), solimp=(.97,.995,.001))
        add(actuators, "position", name=f"jaw_{side}_drive", joint=f"jaw_{side}", kp=1400, kv=8,
            ctrllimited="true", ctrlrange=(0, OPEN), forcelimited="true", forcerange=(-22, 22))

    records = []
    parked_qpos = []
    for i, (name, color, width, grasp_z, parts) in enumerate(OBJECTS):
        position = (cx + (-.06,.06)[i%2], cy + (-.042,.042)[i//2], table+.0003)
        body = add(world, "body", name=name, pos=position)
        add(body, "freejoint", name=f"free_{name}")
        parked_qpos.extend((*position, 1, 0, 0, 0))
        mass = .045 + (i % 4)*.009
        rgba = (.82, .12, .095, 1) if color=="red" else (.07, .34, .88, 1)
        for j, (kind, center, size) in enumerate(parts):
            add(body,"geom",name=f"part_{name}_{j}",type=kind,pos=center,
                rgba=rgba,mass=mass/len(parts),friction=(.75,.010,.001),condim=4,size=size)
        records.append(dict(id=name, color=color, mass_kg=mass, grip_width_m=width,
                            grip_height_m=grasp_z, parts=parts, origin="generated in scene.py"))
    root.find("compiler").set("inertiafromgeom", "auto")
    key = root.find("keyframe/key")
    key.set("qpos", vals((*home, OPEN, OPEN, *parked_qpos)))
    key.set("ctrl", vals((*home, OPEN, OPEN)))
    if "act" in key.attrib:
        key.set("act", vals(home))
    ET.indent(root)
    ET.ElementTree(root).write(scene, encoding="utf-8", xml_declaration=True)
    manifest = dict(units="SI", scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        source_model=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        center_xy_m=[cx,cy],tabletop_z_m=table, tray_floor_z_m=table+.009,
        home_rad=home.tolist(),arm=arm,objects=records,
        tool=dict(mount_quat_wxyz=adapter_quat.tolist(),tcp_from_mount_m=.111,
                  palm_mass_kg=.22,finger_mass_kg=.035,finger_force_limit_N=22,pad_friction=2.2))
    (project/'assets').mkdir(exist_ok=True)
    (project/'assets/manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return manifest
