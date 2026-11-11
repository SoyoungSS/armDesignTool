import copy, sys, tempfile, unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arm_sim.model import normalize,read_input
from arm_sim.geometry import build_geometry,mass_properties,stl
from arm_sim.surfaces import rounded,beam_frame,proportion_plan,template
from arm_sim.validation import mesh_checks
from arm_sim.math3d import forward
SKILL=Path(__file__).resolve().parents[2]

class ShellTests(unittest.TestCase):
    def test_closed_rounded_surfaces_and_extent(self):
        with tempfile.TemporaryDirectory() as temp:
            for kind,size in [('cylinder',[.06,.04]),('chamfer',[.10,.08,.4]),('chamfer',[.025,.025,.002])]:
                v,f=rounded(kind,size); stl(Path(temp)/(kind+str(size[-1])+'.stl'),v,f)
                self.assertGreater(len(f),400)
                self.assertLessEqual(np.ptp(v[:,2]),size[-1]+1e-12)
            self.assertTrue(mesh_checks(Path(temp))['closed_and_oriented'])
    def test_global_scale_and_zero_span_modules(self):
        n=normalize(read_input(SKILL/'assets/seven-axis-driven.json'))
        p=proportion_plan(n)
        self.assertGreater(p['widths_m'][1],.05)
        self.assertGreater(p['widths_m'][1],p['widths_m'][-1])
        self.assertLessEqual(p['module_diameter_caps_m'][3],.9*.0825+1e-12)
    def test_frame_and_template_axis_relations(self):
        e=np.array([.2,-.1,.3]); a=np.array([1,1,0])/2**.5; R=beam_frame(e,a)
        np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12)
        np.testing.assert_allclose(R[:,2],e/np.linalg.norm(e))
        self.assertAlmostEqual(np.linalg.det(R),1)
        up={'type':'revolute','axis':[0,0,1]}
        down={'type':'revolute','axis':[1,0,0],'origin':{'xyz_m':[0,0,0],'rpy_rad':[0,0,0]}}
        self.assertEqual(template(up,down,np.zeros(3),.1),'cross_axis_wrist')
        down['axis']=[0,0,1]
        self.assertEqual(template(up,down,np.zeros(3),.1),'coaxial_wrist')
    def test_detail_switch_preserves_physics_collision_and_fk(self):
        raw=read_input(SKILL/'assets/skew-prismatic.json'); signatures=[]; masses=[]; poses=[]; counts=[]
        for detail in [0,1]:
            a=copy.deepcopy(raw); a.setdefault('dimensions',{})['appearance']={'detail_level':detail}; n=normalize(a)
            before=forward(n,n['poses']['initial'])
            with tempfile.TemporaryDirectory() as temp:
                g=build_geometry(n,Path(temp)); mass_properties(n,g)
            after=forward(n,n['poses']['initial'])
            for k in before: np.testing.assert_array_equal(before[k],after[k])
            signatures.append([(link,p['name'],p['collision_kind'],p.get('collision_size',p['size']),p['T']) for link,v in n['geometry'].items() for p in v['parts'] if p['collision']])
            masses.append(n['mass_properties']); counts.append(sum(len(v['parts']) for v in g.values()))
        self.assertEqual(signatures[0],signatures[1]); self.assertEqual(masses[0],masses[1]); self.assertGreater(counts[1],counts[0])
    def test_supplied_dimensions_and_envelope(self):
        raw=read_input(SKILL/'assets/seven-axis-driven.json')
        raw['dimensions']={'joints':{'J3':{'diameter_m':.12,'axial_width_m':.055}}}
        n=normalize(raw)
        with tempfile.TemporaryDirectory() as temp: build_geometry(n,Path(temp))
        d=n['geometry']['J3_link']['dimensions']; self.assertEqual(d['diameter_m']['value'],.12); self.assertEqual(d['diameter_m']['source'],'design')
        self.assertTrue(any('explicit module diameter' in x for x in n['notes']))
        raw['dimensions']['links']={'J3_link':{'max_envelope_m':[.001,.001,.001]}}
        with tempfile.TemporaryDirectory() as temp, self.assertRaisesRegex(ValueError,'max_envelope'): build_geometry(normalize(raw),Path(temp))

if __name__=='__main__': unittest.main()
