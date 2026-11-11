import copy, json, subprocess, sys, tempfile, unittest, shutil
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arm_sim.model import normalize, InputError, read_input
from arm_sim.math3d import forward, tf
from arm_sim.geometry import build_geometry, mass_properties, read_stl
from arm_sim.pipeline import generate, validate

SKILL=Path(__file__).resolve().parents[2]

class ModelTests(unittest.TestCase):
    def setUp(self): self.raw=read_input(SKILL/'assets'/'skew-prismatic.json')
    def test_nonzero_fk_against_existing_workspace_skill(self):
        reference=SKILL.parent/'arm-workspace-parametric'/'scripts'/'lib'/'kinematics.mjs'
        if not reference.exists() or not shutil.which('node'): self.skipTest('optional independent FK reference skill/Node unavailable')
        n=normalize(self.raw)
        for q in [n['poses']['geometric_zero'],n['poses']['initial'],n['poses']['acceptance'][0]['q']]:
            program="import {forward} from "+json.dumps(reference.as_uri())+"; const v=JSON.parse(process.argv[1]); console.log(JSON.stringify(forward(v.model,v.q)));"
            out=subprocess.check_output(['node','--input-type=module','-e',program,json.dumps({'model':n['model'],'q':list(q.values())})],text=True)
            f=json.loads(out); got=forward(n,q,world=False)['tool_frame']
            np.testing.assert_allclose(got[:3,3],f['position_m'],atol=1e-12); np.testing.assert_allclose(got[:3,:3],f['rotation'],atol=1e-12)
    def test_missing_fields_are_listed(self):
        del self.raw['model']['joints'][0]['axis']; del self.raw['model']['joints'][1]['origin']
        with self.assertRaisesRegex(InputError,'axis.*origin'): normalize(self.raw)
    def test_explicit_connectivity_sorted(self):
        j=self.raw['model']['joints']; parent='base_link'
        for k,x in enumerate(j): x['parent']=parent; x['child']='L'+str(k); parent=x['child']
        self.raw['model']['joints']=j[::-1]; self.raw['physics'].pop('payload')
        self.assertEqual(normalize(self.raw)['model']['joints'][0]['name'],'shoulder')
    def test_units_and_installation_once(self):
        expected=normalize(self.raw)
        j=self.raw['model']['joints'][0]; j['origin']['xyz_mm']=[1000*x for x in j['origin'].pop('xyz_m')]; j['origin']['rpy_deg']=[x*180/np.pi for x in j['origin'].pop('rpy_rad')]
        j['limits_deg']=[x*180/np.pi for x in j.pop('limits')]
        n=normalize(self.raw); q=n['poses']['initial']
        np.testing.assert_allclose(forward(n,q)['tool_frame'],forward(expected,q)['tool_frame'],atol=1e-12)
        np.testing.assert_allclose(forward(n,q)['tool_frame'],tf(n['installation']['base_in_world'])@forward(n,q,False)['tool_frame'],atol=1e-12)
    def test_bad_axes_limits_inertia_and_payload(self):
        variants=[]
        a=copy.deepcopy(self.raw); a['model']['joints'][0]['axis']=[0,0,0]; variants.append(a)
        a=copy.deepcopy(self.raw); a['model']['joints'][1]['limits']=[1,0]; variants.append(a)
        a=copy.deepcopy(self.raw); a['model']['joints'][0]['components'][0]['inertia_kg_m2']=[[1,0,0],[0,1,0],[0,0,3]]; variants.append(a)
        a=copy.deepcopy(self.raw); del a['physics']['payload']['included_in']; variants.append(a)
        for a in variants:
            with self.assertRaises(InputError): normalize(a)
    def test_simple_adapter_matches_reference(self):
        source=SKILL.parent/'arm-statics-parametric'/'assets'/'examples'/'simple-two-link.json'
        if not source.exists(): self.skipTest('optional reference skill unavailable')
        a=read_input(source); a['payload']['separate_mass_confirmed']=True
        n=normalize(a); np.testing.assert_allclose(n['model']['joints'][1]['origin']['xyz_m'],[1,0,0]); np.testing.assert_allclose(n['model']['tip']['xyz_m'],[1,0,0])
        self.assertEqual(sum(c['mass_kg'] for cs in n['components'].values() for c in cs),4)
    def test_workspace_input_adapter(self):
        source=SKILL.parent/'arm-workspace-parametric'/'assets'/'examples'/'worksheet-workspace.jsonc'
        if not source.exists(): self.skipTest('optional reference input unavailable')
        raw=read_input(source); n=normalize(raw)
        self.assertEqual(n['source'],'arm-workspace-parametric/standard')
        self.assertEqual(len(n['model']['joints']),len(raw['model']['joints']))
        np.testing.assert_allclose(tf(n['installation']['base_in_world']),tf(raw['base_in_world']))
    def test_mass_parallel_axis_and_rotation(self):
        n=normalize(self.raw)
        with tempfile.TemporaryDirectory() as temp:
            g=build_geometry(n,Path(temp)); mass_properties(n,g)
        p=n['mass_properties']['shoulder_link']; cs=n['components']['shoulder_link']; m=sum(c['mass_kg'] for c in cs); com=sum(c['mass_kg']*np.array(c['com_m']) for c in cs)/m
        I=sum(np.array(c['inertia_kg_m2'])+c['mass_kg']*(np.dot(np.array(c['com_m'])-com,np.array(c['com_m'])-com)*np.eye(3)-np.outer(np.array(c['com_m'])-com,np.array(c['com_m'])-com)) for c in cs)
        np.testing.assert_allclose(p['com_m'],com); np.testing.assert_allclose(p['inertia_kg_m2'],I)
    def test_short_coincident_fixed_meshes(self):
        self.raw['model']['joints'][1]['origin']['xyz_m']=[0,0,0]
        self.raw['model']['tip']['xyz_m']=[0,0,0]
        n=normalize(self.raw)
        with tempfile.TemporaryDirectory() as temp:
            build_geometry(n,Path(temp))
            for p in Path(temp).glob('*.stl'): self.assertGreater(len(read_stl(p)),0)
    def test_beam_centers_preserve_real_span_and_proxy_com(self):
        n=normalize(self.raw)
        with tempfile.TemporaryDirectory() as temp:
            g=build_geometry(n,Path(temp))
            links=list(g)
            for i,link in enumerate(links):
                endpoint=np.array(n['model']['joints'][i]['origin']['xyz_m'] if i<len(n['model']['joints']) else n['model']['tip']['xyz_m'])
                beam=next(p for p in g[link]['parts'] if p['name']=='beam')
                np.testing.assert_allclose(beam['T'][:3,3],endpoint/2,atol=1e-12)
                np.testing.assert_allclose(g[link]['proxy']['T'][:3,3],endpoint/2,atol=1e-12)
                axial=(beam['vertices']-endpoint/2)@(endpoint/np.linalg.norm(endpoint))
                self.assertLessEqual(max(abs(axial)),np.linalg.norm(endpoint)/2+1e-12)

class PipelineTests(unittest.TestCase):
    def test_kinematic_only_and_relocation_and_determinism(self):
        with tempfile.TemporaryDirectory() as temp:
            a=generate(SKILL/'assets'/'two-revolute.json',Path(temp),['urdf','ros1'])
            self.assertEqual(a['status'],'completed',a)
            b=generate(SKILL/'assets'/'two-revolute.json',Path(temp),['urdf','ros1'])
            r1=Path(a['run_directory']); r2=Path(b['run_directory'])
            for p in (r1/'meshes').glob('*.stl'): self.assertEqual(p.read_bytes(),(r2/'meshes'/p.name).read_bytes())
            self.assertEqual(validate(r2)['status'],'completed')
            mesh=next((r2/'meshes').glob('*.stl')); mesh.write_bytes(b'corrupt')
            self.assertEqual(validate(r2)['status'],'failed')
    def test_stale_success_never_reused(self):
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'out'; a=generate(SKILL/'assets'/'two-revolute.json',out,['urdf']); self.assertEqual(a['status'],'completed')
            bad=Path(temp)/'bad.json'; bad.write_text('{"schema_version":"bogus"}')
            b=generate(bad,out); self.assertEqual(b['status'],'failed'); self.assertNotEqual(a['run_directory'],b['run_directory']); self.assertEqual(validate(out)['status'],'failed')
    def test_all_targets_runtime(self):
        try: import mujoco; from pxr import Usd
        except (ImportError,OSError): self.skipTest('optional MuJoCo/OpenUSD runtime unavailable')
        with tempfile.TemporaryDirectory() as temp:
            result=generate(SKILL/'assets'/'skew-prismatic.json',Path(temp),runtime=True)
            self.assertEqual(result['status'],'completed',result); self.assertEqual(result['targets']['mujoco']['runtime_validation'],'passed'); self.assertTrue(result['targets']['isaac']['generated'])
    def test_strict_missing_limits_and_concept_mass(self):
        a=read_input(SKILL/'assets'/'two-revolute.json'); a['export']={'mode':'strict','targets':['urdf']}
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'input.json'; p.write_text(json.dumps(a)); result=generate(p,Path(temp)/'out'); self.assertEqual(result['status'],'failed')
            a['export']={'mode':'concept','targets':['mujoco'],'limit_placeholders':{'effort':10,'velocity':1}}; a['physics']={'estimation':{'method':'uniform_density','density_kg_m3':800}}
            p.write_text(json.dumps(a)); result=generate(p,Path(temp)/'out'); self.assertEqual(result['status'],'completed'); n=read_input(Path(result['run_directory'])/'report'/'normalized_model.json'); self.assertFalse(n['missing_physics']); self.assertEqual(n['model_grade'],'concept_simulation')

if __name__=='__main__': unittest.main()
