"""Shared imports and a name-based instantaneous MuJoCo recorder."""
import csv
import ctypes
import json
import os
from pathlib import Path
if os.name == 'nt' and os.environ.get('ARM_SIM_SYSTEM_MSVC') == '1':
    ctypes.WinDLL(str(Path(os.environ['SystemRoot'])/'System32/msvcp140.dll'))
import mujoco as mj
import numpy as np


def dump(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


class Recorder:
    def __init__(self, m, folder, joints, sites):
        self.m, self.folder = m, Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.joints = []
        for name in joints:
            j = m.joint(name); kind = int(m.jnt_type[j.id])
            if kind not in [int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)]:
                raise ValueError('Robot recorder supports hinge/slide only: '+name)
            self.joints.append((name, j.id, int(m.jnt_qposadr[j.id]), int(m.jnt_dofadr[j.id]), kind))
        self.sites = [(name, m.site(name).id) for name in sites]
        self.files = []
        def table(name, cols):
            f = (self.folder/(name+'.csv')).open('w', newline='', encoding='utf-8-sig'); self.files.append(f)
            w = csv.writer(f); w.writerow(cols); return w
        lead = ['time_s', 'cycle', 'phase']
        self.jw = table('joints', lead+['joint', 'type', 'position', 'velocity', 'acceleration', 'effort', 'bias', 'passive', 'constraint', 'applied', 'power_W', 'limit_margin'])
        self.tw = table('end_effectors', lead+['site']+[f'p_{a}_m' for a in 'xyz']+[f'quat_{a}' for a in 'wxyz']+[f'v_{a}_m_s' for a in 'xyz']+[f'a_{a}_m_s2' for a in 'xyz']+[f'omega_{a}_rad_s' for a in 'xyz']+[f'alpha_{a}_rad_s2' for a in 'xyz'])
        self.aw = table('actuators', lead+['actuator', 'ctrl', 'force', 'length', 'velocity', 'ctrl_at_limit'])
        self.cw = table('contacts', lead+['geom1', 'geom2', 'distance_m']+[f'p_{a}_m' for a in 'xyz']+['normal_force_N', 'tangent1_N', 'tangent2_N', 'moment_normal_Nm', 'moment_tangent1_Nm', 'moment_tangent2_Nm']+[f'frame_{i}' for i in range(9)])
        self.sw = table('sensors', lead+['sensor', 'component', 'value'])
        self.dw = table('diagnostics', lead+['balance_residual', 'contacts', 'warning_count'])
        self.jp, self.jr, self.dp, self.dr = [np.zeros((3, m.nv)) for _ in range(4)]
        self.mass = np.zeros((m.nv, m.nv)); self.quats = {}; self.samples = 0
        dump(self.folder/'metadata.json', dict(joints=[dict(name=n, id=i, qposadr=q, dofadr=v, type='hinge' if k==3 else 'slide', position_unit='rad' if k==3 else 'm', effort_unit='N m' if k==3 else 'N') for n,i,q,v,k in self.joints],
            sites=sites, frame='world', quaternion='wxyz', sample='instantaneous state at time_s after control + mj_forward, before mj_step',
            acceleration='Jacobian*qacc + Jacobian_time_derivative*qvel; world-coordinate kinematic acceleration, gravity not added',
            contact_frame='3x3 row-major axes in world; wrench from mj_contactForce in this contact frame',
            sensors=[dict(name=m.sensor(i).name, type=int(m.sensor_type[i]), dim=int(m.sensor_dim[i])) for i in range(m.nsensor)]))

    def sample(self, d, cycle, phase):
        m = self.m; lead = [float(d.time), cycle, phase]
        for name, jid, q, v, kind in self.joints:
            margin = min(d.qpos[q]-m.jnt_range[jid, 0], m.jnt_range[jid, 1]-d.qpos[q]) if m.jnt_limited[jid] else ''
            self.jw.writerow(lead+[name, 'hinge' if kind==3 else 'slide', d.qpos[q], d.qvel[v], d.qacc[v], d.qfrc_actuator[v], d.qfrc_bias[v], d.qfrc_passive[v], d.qfrc_constraint[v], d.qfrc_applied[v], d.qfrc_actuator[v]*d.qvel[v], margin])
        for name, sid in self.sites:
            mj.mj_jacSite(m, d, self.jp, self.jr, sid)
            mj.mj_jacDot(m, d, self.dp, self.dr, d.site_xpos[sid], int(m.site_bodyid[sid]))
            quat = np.empty(4); mj.mju_mat2Quat(quat, d.site_xmat[sid])
            if name in self.quats and quat@self.quats[name] < 0: quat *= -1
            self.quats[name] = quat.copy()
            self.tw.writerow(lead+[name, *d.site_xpos[sid], *quat, *(self.jp@d.qvel), *(self.jp@d.qacc+self.dp@d.qvel), *(self.jr@d.qvel), *(self.jr@d.qacc+self.dr@d.qvel)])
        for i in range(m.nu):
            at_limit = bool(m.actuator_ctrllimited[i] and (d.ctrl[i] <= m.actuator_ctrlrange[i,0] or d.ctrl[i] >= m.actuator_ctrlrange[i,1]))
            self.aw.writerow(lead+[m.actuator(i).name or str(i), d.ctrl[i], d.actuator_force[i], d.actuator_length[i], d.actuator_velocity[i], int(at_limit)])
        wrench = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]; mj.mj_contactForce(m,d,i,wrench)
            self.cw.writerow(lead+[m.geom(c.geom1).name, m.geom(c.geom2).name, c.dist, *c.pos, *wrench, *c.frame])
        for i in range(m.nsensor):
            for k in range(int(m.sensor_dim[i])):
                self.sw.writerow(lead+[m.sensor(i).name or str(i), k, d.sensordata[m.sensor_adr[i]+k]])
        mj.mj_fullM(m,d,self.mass)
        residual = self.mass@d.qacc+d.qfrc_bias-d.qfrc_passive-d.qfrc_actuator-d.qfrc_constraint-d.qfrc_applied
        # xfrc_applied needs a generalized-force conversion before this check.
        balance = float(np.max(abs(residual))) if not np.any(d.xfrc_applied) else ''
        self.dw.writerow(lead+[balance,d.ncon,sum(int(w.number) for w in d.warning)])
        self.samples += 1
        if self.samples % 1000 == 0:
            for f in self.files: f.flush()

    def close(self):
        for f in self.files: f.close()
