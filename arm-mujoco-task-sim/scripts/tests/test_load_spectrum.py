"""Physical and numerical regressions independent of the sorting demonstration."""
import csv
import json
import shutil
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
from load_spectrum import export
from runtime import mj, np, Recorder, dump
from recording_contract import resolve
from goal_checks import check as check_goals


class SpectrumTests(unittest.TestCase):
    def make_run(self, root, rows):
        run = Path(root); (run/'data').mkdir()
        dump(run/'data/metadata.json', dict(joints=[dict(name='hinge',type='hinge',effort_unit='N m'),
                                                        dict(name='slide',type='slide',effort_unit='N')]))
        with (run/'data/joints.csv').open('w', newline='') as f:
            w = csv.writer(f); w.writerow(['time_s','cycle','phase','joint','effort','velocity','power_W'])
            w.writerows(rows)
        return run

    def rows(self):
        return [[t,cycle,phase,name,effort,2.,effort*2.] for t,cycle,phase,effort in
                [(0.,1,'hold',-2.),(.5,1,'move',4.),(2.,1,'move',8.),(3.,2,'hold',100.),(4.,2,'hold',1.)]
                for name in ['hinge','slide']]

    def test_signed_duration_phase_reset_and_power(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.make_run(tmp, self.rows())
            (run/'data/events.jsonl').write_text(json.dumps(dict(time_s=3.,event={'kind':'episode_reset'}))+'\n')
            result = export(run, bins=4)
            self.assertEqual(result['section_status'], 'not_available')
            with (run/'reports/load_statistics.csv').open(encoding='utf-8-sig') as f:
                rows = list(csv.DictReader(f))
            all_effort = next(r for r in rows if r['joint']=='hinge' and r['signal']=='effort' and r['scope']=='all')
            self.assertAlmostEqual(float(all_effort['duration_s']), 3.)
            self.assertAlmostEqual(float(all_effort['time_rms']), ((4*.5+16*1.5+10000)/3)**.5)
            self.assertEqual(float(all_effort['min']), -2.)
            self.assertEqual(float(all_effort['max']), 100.)
            phase = next(r for r in rows if r['joint']=='hinge' and r['signal']=='effort' and r['scope']=='phase' and r['scope_id']=='hold')
            self.assertAlmostEqual(float(phase['duration_s']), 1.5)
            with (run/'reports/load_spectrum.csv').open(encoding='utf-8-sig') as f:
                spectrum = [r for r in csv.DictReader(f) if r['joint']=='hinge' and r['signal']=='effort' and r['scope']=='all']
            self.assertAlmostEqual(sum(float(r['duration_s']) for r in spectrum), 3.)
            self.assertAlmostEqual(sum(float(r['time_fraction']) for r in spectrum), 1.)
            with (run/'reports/effort_speed_spectrum.csv').open(encoding='utf-8-sig') as f:
                joint = [r for r in csv.DictReader(f) if r['joint']=='hinge' and r['scope']=='all']
            self.assertAlmostEqual(sum(float(r['duration_s']) for r in joint),3.)

    def test_missing_joint_and_stale_success_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.make_run(tmp, self.rows()); export(run)
            rows = self.rows(); rows.pop()
            with (run/'data/joints.csv').open('w',newline='') as f:
                w=csv.writer(f); w.writerow(['time_s','cycle','phase','joint','effort','velocity','power_W']); w.writerows(rows)
            with self.assertRaisesRegex(ValueError, 'incomplete final'):
                export(run)
            self.assertEqual(json.loads((run/'reports/load_spectrum.json').read_text())['status'],'failed')

    def test_continuous_cycle_transition_preserves_duty_duration(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=self.make_run(tmp,self.rows()); export(run)
            with (run/'reports/load_statistics.csv').open(encoding='utf-8-sig') as f:
                rows=list(csv.DictReader(f))
            all_effort=next(r for r in rows if r['joint']=='hinge' and r['signal']=='effort' and r['scope']=='all')
            self.assertAlmostEqual(float(all_effort['duration_s']),4.)
            cycles=[r for r in rows if r['joint']=='hinge' and r['signal']=='effort' and r['scope']=='cycle']
            self.assertAlmostEqual(sum(float(r['duration_s']) for r in cycles),4.)

    def test_nonfinite_and_time_reversal_rejected(self):
        for value in [float('nan'), float('inf')]:
            with tempfile.TemporaryDirectory() as tmp:
                rows=self.rows(); rows[0][-1]=value
                with self.assertRaisesRegex(ValueError,'nonfinite'): export(self.make_run(tmp,rows))
        with tempfile.TemporaryDirectory() as tmp:
            rows=self.rows(); rows[2][0]=rows[3][0]=-.5
            with self.assertRaisesRegex(ValueError,'non-increasing'): export(self.make_run(tmp,rows))

    def test_zero_duration_has_no_invented_rms(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=self.make_run(tmp,self.rows()[:2]); export(run)
            with (run/'reports/load_statistics.csv').open(encoding='utf-8-sig') as f:
                for row in csv.DictReader(f):
                    self.assertEqual(row['time_rms'],''); self.assertEqual(float(row['duration_s']),0.)


class PhysicalTests(unittest.TestCase):
    XML='''<mujoco><option gravity="0 0 -10"/><worldbody>
    <body name="unrelated" pos="0 3 0"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
    <body name="base"><body name="link"><joint name="lift" type="slide" axis="0 0 1"/>
    <inertial pos="0 0 0" mass="2" diaginertia=".1 .1 .1"/>
    <site name="load_site_lift"/><body name="fixed_tip"><inertial pos=".2 0 0" mass="1" diaginertia=".1 .1 .1"/>
    <site name="load_site_fixed"/></body></body></body></worldbody>
    <actuator><motor name="motor" joint="lift"/></actuator><sensor>
    <force name="load_force_lift" site="load_site_lift"/><torque name="load_torque_lift" site="load_site_lift"/>
    <force name="load_force_fixed" site="load_site_fixed"/><torque name="load_torque_fixed" site="load_site_fixed"/>
    </sensor></mujoco>'''

    def handoff(self):
        js=[]
        for name,typ,parent,child in [('lift','prismatic','base','link'),('fixed','fixed','link','fixed_tip')]:
            js.append(dict(name=name,type=typ,parent=parent,child=child,mjcf_joint=name if name=='lift' else None,
                           section=dict(site='load_site_'+name,force_sensor='load_force_'+name,
                                        torque_sensor='load_torque_'+name,position_in_child_m=[0,0,0],quaternion_wxyz=[1,0,0,0])))
        return dict(kind='arm-load-handoff',active_joint_names=['lift'],fixed_joint_names=['fixed'],
                    section_sensors_generated=True,joints=js)

    def test_all_joint_mapping_and_six_axis_static_wrench(self):
        m=mj.MjModel.from_xml_string(self.XML); d=mj.MjData(m)
        joints,sections,coverage=resolve(m,{'robot_joints':'all'},{'load_handoff':self.handoff()})
        self.assertEqual(joints,['lift']); self.assertEqual(coverage['section_count'],2)
        d.ctrl[0]=30.; mj.mj_forward(m,d)
        self.assertAlmostEqual(d.qacc[m.jnt_dofadr[m.joint('lift').id]],0.,places=10)
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp); (run/'data').mkdir()
            r=Recorder(m,run/'data',joints,[],sections,coverage)
            r.sample(d,1,'static_hold'); d.time=.25; r.sample(d,1,'static_hold'); r.close()
            with (run/'data/joints.csv').open(encoding='utf-8-sig') as f:
                row=next(csv.DictReader(f)); self.assertAlmostEqual(float(row['effort']),30.)
            with (run/'data/joint_loads.csv').open(encoding='utf-8-sig') as f:
                rows={r['joint']:r for r in csv.DictReader(f)}
            # Independent equilibrium: 3 kg supported, 1 kg at 0.2 m -> 2 N m.
            self.assertAlmostEqual(float(rows['lift']['force_z_N']),30.)
            self.assertAlmostEqual(float(rows['fixed']['force_z_N']),10.)
            self.assertAlmostEqual(float(rows['fixed']['torque_y_Nm']),-2.)
            result=export(run); self.assertEqual(result['section_status'],'available')
            self.assertEqual(result['section_names'],['lift','fixed'])

    def test_missing_joint_or_changed_sensor_not_accepted(self):
        m=mj.MjModel.from_xml_string(self.XML)
        with self.assertRaisesRegex(ValueError,'Missing arm joints'):
            resolve(m,{'robot_joints':['unrelated']},{'load_handoff':self.handoff()})
        m.site_pos[m.site('load_site_fixed').id,0]=.1
        with self.assertRaisesRegex(ValueError,'reference frame'):
            resolve(m,{'robot_joints':'all'},{'load_handoff':self.handoff()})

    def test_live_plot_sampling_does_not_require_tcp(self):
        from viewer_ui import viewer_plugins
        m=mj.MjModel.from_xml_string(self.XML); d=mj.MjData(m); mj.mj_forward(m,d)
        plots,idle=viewer_plugins(dict(robot_joints=['lift'],end_effector_sites=[]),0)
        plots.reset_series(SimpleNamespace(m=m,run=Path('fixture')))
        plots.sample(d,1); d.time=.1; plots.sample(d,1)
        self.assertEqual(len(plots.series),2)
        self.assertIsNone(plots.site_id)

    def test_goal_predicate_false_positive_detected_without_moving_live_state(self):
        m=mj.MjModel.from_xml_string(self.XML); d=mj.MjData(m); mj.mj_forward(m,d)
        adr=m.jnt_qposadr[m.joint('lift').id]
        class Task:
            def goal_tests(self,m):
                return [dict(name='reset',expected=False,setup=lambda m,d:None),
                        dict(name='raised',expected=True,setup=lambda m,d:d.qpos.__setitem__(adr,.1))]
            def goal_success(self,m,d): return bool(d.qpos[adr]>.05)
        self.assertEqual(check_goals(Task(),m,d)['status'],'passed')
        self.assertEqual(d.qpos[adr],0.)
        class Broken(Task):
            def goal_success(self,m,d): return True
        self.assertEqual(check_goals(Broken(),m,d)['status'],'failed')

    def test_non_sorting_cycle_adapter_exports_all_sections_through_session(self):
        from run_task import Session
        with tempfile.TemporaryDirectory() as tmp:
            project=Path(tmp)
            for folder in ['scene','code','assets','plan']: (project/folder).mkdir()
            xml=self.XML.replace('<motor name="motor" joint="lift"/>',
                                 '<position name="motor" joint="lift" kp="1000" kv="100"/>')
            (project/'scene/scene.xml').write_text(xml)
            shutil.copyfile(SCRIPTS.parent/'assets/workloads/joint_cycle.py',project/'code/task.py')
            dump(project/'assets/manifest.json',{'purpose':'isolated non-sorting regression'})
            dump(project/'plan/strategy.json',{'purpose':'cyclic servo excitation'})
            dump(project/'model_inspection.json',{'load_handoff':self.handoff()})
            c=dict(schema_version='1.0',model_input='isolated regression model',
                   scene='scene/scene.xml',adapter='code/task.py',asset_manifest='assets/manifest.json',strategy_file='plan/strategy.json',
                   task_description='Cyclic motion and final hold, no sorting assets',initial_conditions='zero joints',
                   cycle_definition='0.1 s continuous motion',between_cycles='continuous',timing='0.2 s motion + 0.2 s settling',
                   physics_approval='synthetic tool fixture only',strategy='direct position servo',state_source='measured joints',
                   mode='headless',cycles=2,seed=4,dt_s=.001,max_time_s=.5,robot_joints='all',end_effector_sites=[],
                   success_criteria=['measured return within explicit fixture tolerance'],
                   workload=dict(period_s=.1,settling_s=.2,excursion_by_joint={'lift':.02},completion_tolerance_SI={'lift':.04}),
                   confirmation={k:True for k in ['mode','cycles','task','assets_physics','strategy','timing','outputs']})
            c['confirmation']['evidence']='Synthetic regression authorization only; not a user production duty.'
            dump(project/'task_contract.json',c)
            s=Session(project)
            try:
                while not s.done: s.tick()
            finally: s.finish()
            result=json.loads((s.run/'reports/result.json').read_text())
            self.assertEqual(result['status'],'passed')
            spectrum=json.loads((s.run/'reports/load_spectrum.json').read_text())
            self.assertEqual(spectrum['joint_names'],['lift'])
            self.assertEqual(spectrum['section_names'],['lift','fixed'])
            self.assertEqual(spectrum['coverage']['status'],'complete')
            self.assertEqual(spectrum['task_result']['status'],'passed')

    def test_contact_push_records_actual_constraint_and_drive_effort(self):
        xml='''<mujoco><option timestep=".001" gravity="0 0 0"/><worldbody>
        <geom name="wall" type="box" pos=".15 0 0" size=".02 .2 .2"/>
        <body name="slider"><joint name="push" type="slide" axis="1 0 0"/>
        <geom name="tool" type="sphere" size=".03" mass="1"/></body>
        </worldbody><actuator><motor name="drive" joint="push"/></actuator></mujoco>'''
        m=mj.MjModel.from_xml_string(xml); d=mj.MjData(m); d.ctrl[0]=10.
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp); r=Recorder(m,run/'data',['push'],[])
            for _ in range(501):
                mj.mj_forward(m,d); r.sample(d,1,'push'); mj.mj_step(m,d)
            r.close()
            self.assertGreater(d.ncon,0)
            self.assertAlmostEqual(d.qfrc_actuator[0],10.)
            self.assertLess(d.qfrc_constraint[0],-9.)
            result=export(run)
            self.assertEqual(result['joint_names'],['push'])


if __name__ == '__main__': unittest.main()
