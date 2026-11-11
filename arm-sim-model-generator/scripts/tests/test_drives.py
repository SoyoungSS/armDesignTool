import copy, json, sys, tempfile, unittest, subprocess, hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arm_sim.model import normalize, read_input
from arm_sim.geometry import build_geometry, mass_properties
from arm_sim.drives import prepare_drives, mass_matrix_and_gravity
from arm_sim.pipeline import generate
SKILL=Path(__file__).resolve().parents[2]

def example(kind='revolute'):
    return {'schema_version':'1.0','model':{'name':'test_arm','joints':[{'name':'J1','type':kind,'origin':{'xyz_m':[0,0,0],'rpy_rad':[0,0,0]},'axis':[0,1,0] if kind=='revolute' else [0,0,1],'limits':[-1,1] if kind=='revolute' else [-.1,.2]}], 'tip':{'xyz_m':[.4,0,0],'rpy_rad':[0,0,0]}},
            'physics':{'components':{'base_link':[{'name':'base','mass_kg':1,'com_m':[0,0,0],'inertia_frame':'link','inertia_kg_m2':np.diag([.02,.02,.02]).tolist()}],'J1':[{'name':'arm','mass_kg':2,'com_m':[.2,0,0],'inertia_frame':'link','inertia_kg_m2':np.diag([.01,.03,.03]).tolist()}]}},
            'drives':{'J1':{'effort':100,'velocity':2}},'export':{'mode':'strict','targets':['mujoco','isaac']}}

def prepared(raw):
    n=normalize(raw)
    with tempfile.TemporaryDirectory() as temp: g=build_geometry(n,Path(temp)); mass_properties(n,g)
    prepare_drives(n); return n

class DrivePolicyTests(unittest.TestCase):
    def test_closed_form_mass_and_gravity(self):
        n=prepared(example()); M,g=mass_matrix_and_gravity(n,{'J1':0})
        np.testing.assert_allclose(M,[[.11]],atol=1e-12); np.testing.assert_allclose(g,[-3.924],atol=1e-12)
        n=prepared(example('prismatic')); M,g=mass_matrix_and_gravity(n,{'J1':.1})
        np.testing.assert_allclose(M,[[2]],atol=1e-12); np.testing.assert_allclose(g,[19.62],atol=1e-12)
    def test_deterministic_and_physical_scaling(self):
        a=example(); n=prepared(a); self.assertEqual(n,prepared(a)); self.assertEqual(n['drives']['J1']['mode'],'position')
        c=n['drives']['J1']; self.assertGreater(c['kp'],0); self.assertGreater(c['kv'],0)
        for comps in a['physics']['components'].values():
            for p in comps: p['mass_kg']*=2; p['inertia_kg_m2']=(np.array(p['inertia_kg_m2'])*2).tolist()
        a['drives']['J1']['effort']*=2; c2=prepared(a)['drives']['J1']
        np.testing.assert_allclose([c2['kp'],c2['kv']],[2*c['kp'],2*c['kv']],rtol=1e-12)
    def test_explicit_passive_and_partial_gains(self):
        a=example(); a['drives']['J1'].update(mode='position',kp=12)
        n=prepared(a); self.assertEqual(n['drives']['J1']['kp'],12); self.assertGreater(n['drives']['J1']['kv'],0)
        a['drives']['J1']['kv']=.4; self.assertEqual(prepared(a)['drives']['J1']['kv'],.4)
        a['drives']['J1']={'mode':'passive'}; self.assertEqual(prepared(a)['drives']['J1'],{'mode':'passive'})
    def test_low_capacity_is_reported_not_invented(self):
        a=example('prismatic'); a['drives']['J1']['effort']=1
        n=prepared(a); self.assertEqual(n['drives']['J1']['effort'],1)
        self.assertTrue(any('gravity reaches' in s for s in n['drive_design']['joints']['J1']['warnings']))
    def test_timestep_adapts_without_altering_inertia(self):
        a=example(); a['drives']['J1'].update(mode='position',kp=100000,kv=100)
        n=prepared(a); self.assertLess(n['drive_design']['timestep_s'],.001)
        self.assertEqual(n['mass_properties']['J1_link']['mass_kg'],2)
    def test_missing_capacity_rejected(self):
        a=example(); a['drives']={}
        with self.assertRaisesRegex(ValueError,'effort/velocity'): prepared(a)
    def test_auto_damping_does_not_rely_on_engine_specific_joint_loss(self):
        a=example(); base=prepared(a)['drives']['J1']['kv']
        a['drives']['J1']['damping']=1000
        n=prepared(a)
        self.assertEqual(n['drives']['J1']['kv'],base)
        self.assertEqual(n['drives']['J1']['damping'],1000)

class DriveExportTests(unittest.TestCase):
    def setUp(self):
        try: import mujoco; from pxr import Usd
        except (ImportError,OSError): self.skipTest('MuJoCo/OpenUSD optional dependencies unavailable')
    def run_example(self,raw,temp,runtime=True):
        p=Path(temp)/'in.json'; p.write_text(json.dumps(raw),encoding='utf-8'); r=generate(p,Path(temp)/'out',runtime=runtime)
        self.assertEqual(r['status'],'completed',r); return Path(r['run_directory'])
    def test_revolute_and_slider_servo_runtime(self):
        for kind in ['revolute','prismatic']:
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as temp:
                run=self.run_example(example(kind),temp)
                v=read_input(run/'report'/'validation.json')
                self.assertEqual(v['mujoco']['runtime']['servos']['status'],'passed')
                self.assertEqual(v['isaac']['drive_attribute_checks'],'passed')
    def test_exports_repeat_and_portable_launcher(self):
        with tempfile.TemporaryDirectory() as temp:
            run=self.run_example(example(),temp,runtime=False); second=self.run_example(example(),temp,runtime=False)
            for filename in ['mujoco/robot.xml','isaac/robot.usd','isaac/robot_zero.usd']:
                self.assertEqual((run/filename).read_bytes(),(second/filename).read_bytes())
            # The runtime in this test may use embedded Python; this portable file
            # imports only installed packages, never the authoring skill.
            subprocess.run([sys.executable,str(run/'mujoco'/'run.py'),'--headless','--seconds','.2'],check=True,capture_output=True)
    def test_effort_and_passive_are_not_position_drives(self):
        import mujoco
        from pxr import Usd,UsdPhysics
        for mode in ['passive','effort']:
            a=example(); a['drives']['J1']['mode']=mode
            with tempfile.TemporaryDirectory() as temp:
                run=self.run_example(a,temp,runtime=False); m=mujoco.MjModel.from_xml_path(str(run/'mujoco'/'robot.xml'))
                self.assertEqual(m.nu,0 if mode=='passive' else 1)
                stage=Usd.Stage.Open(str(run/'isaac'/'robot.usd')); self.assertFalse(UsdPhysics.DriveAPI.GetAll(stage.GetPrimAtPath('/Robot/J1')))
    def test_seven_axis_coupled_servos(self):
        with tempfile.TemporaryDirectory() as temp:
            run=self.run_example(read_input(SKILL/'assets'/'seven-axis-driven.json'),temp)
            v=read_input(run/'report'/'validation.json')
            self.assertEqual(v['mujoco']['runtime']['actuator_checks'],7)
            self.assertEqual(v['mujoco']['runtime']['servos']['status'],'passed')
            self.assertEqual(v['isaac']['drive_attribute_checks'],'passed')
            self.assertGreater(v['isaac']['initial_pose_checks'],7)
    def test_velocity_gain_and_units(self):
        a=example(); a['drives']['J1']['mode']='velocity'
        with tempfile.TemporaryDirectory() as temp:
            run=self.run_example(a,temp)
            n=read_input(run/'report'/'normalized_model.json')
            self.assertGreater(n['drives']['J1']['kv'],0)
            v=read_input(run/'report'/'validation.json')
            self.assertEqual(v['mujoco']['runtime']['actuator_checks'],1)
            self.assertEqual(v['isaac']['drive_attribute_checks'],'passed')

if __name__=='__main__': unittest.main()
