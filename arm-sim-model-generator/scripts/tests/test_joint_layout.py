import copy, sys, tempfile, unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arm_sim.model import normalize,read_input
from arm_sim.geometry import build_geometry,stl
from arm_sim.joint_layout import classify,annulus,packing_checks
from arm_sim.math3d import forward,tf
from arm_sim.validation import mesh_checks
SKILL=Path(__file__).resolve().parents[2]

def make(raw):
    n=normalize(raw)
    with tempfile.TemporaryDirectory() as t: g=build_geometry(n,Path(t))
    return n,g

class JointLayoutTests(unittest.TestCase):
    def setUp(self): self.raw=read_input(SKILL/'assets/seven-axis-driven.json')
    def test_roles_and_frame_preservation(self):
        original=normalize(self.raw); expected=forward(original,original['poses']['initial'])
        n,g=make(self.raw); roles=n['joint_layout']['roles']
        for name in ['J1','J5','J7']: self.assertEqual(roles[name]['role'],'axial_twist')
        for name in ['J2','J6']: self.assertEqual(roles[name]['role'],'transverse_bend')
        self.assertEqual(n['model'],original['model']); self.assertEqual(n['components'],original['components'])
        for k,T in forward(n,n['poses']['initial']).items(): np.testing.assert_array_equal(T,expected[k])
        self.assertEqual([p['joints'] for p in n['joint_layout']['pairs']],[['J1','J2'],['J5','J6']])
        self.assertEqual(packing_checks(n)['status'],'passed')
    def test_fixed_and_output_ownership_under_motion(self):
        n,g=make(self.raw); q=copy.deepcopy(n['poses']['initial']); P=forward(n,q)
        q['J2']+=.2; Q=forward(n,q)
        np.testing.assert_array_equal(P['J1_link'],Q['J1_link'])
        self.assertGreater(np.max(np.abs(P['J2_link']-Q['J2_link'])),.01)
        owners={p['name']:link for link,data in g.items() for p in data['parts']}
        self.assertEqual(owners['J1_stator'],'base_link'); self.assertEqual(owners['J1_output_shaft'],'J1_link')
        self.assertEqual(owners['J2_stator'],'J1_link'); self.assertEqual(owners['J2_output_shaft'],'J2_link')
        self.assertEqual(owners['J5_stator'],'J4_link'); self.assertEqual(owners['J6_stator'],'J5_link')
        for link in ['J1_link','J2_link','J5_link','J6_link']: self.assertFalse(any(p['name']=='housing' for p in g[link]['parts']))
    def test_packing_ignores_engine_filters_and_detects_overlap(self):
        n,g=make(self.raw); first=packing_checks(n)
        n['collision_exclusions']=[{'body1':'base_link','body2':'J2_link'}]
        self.assertEqual(first,packing_checks(n))
        bad=copy.deepcopy(n); a=bad['joint_layout']['motor_envelopes'][0]
        bad['joint_layout']['motor_envelopes'][1]={**a,'joint':'J2'}
        self.assertEqual(packing_checks(bad)['status'],'needs_review')
    def test_retreat_constraints_units_and_installation(self):
        self.raw.setdefault('dimensions',{})['joints']={'J1':{'module_retreat_mm':130,'role':'axial_twist'}}
        self.raw['dimensions']['appearance']={'assembly_clearance_mm':3}
        self.raw['installation']['base_in_world']={'xyz_m':[2,-1,.5],'rpy_rad':[.4,.8,-.3]}
        n,g=make(self.raw); self.assertEqual(n['joint_layout']['pairs'][0]['retreat_m'],.13)
        role=n['joint_layout']['roles']['J1']
        self.assertEqual(role['axis_frame'],n['base_link'])
        np.testing.assert_allclose(role['axis_base_zero'],[0,0,1])
        self.assertGreaterEqual(packing_checks(n)['min_paired_gap_m'],.003-1e-8)
        self.raw['dimensions']['joints']['J1']['module_retreat_mm']=1
        with self.assertRaisesRegex(ValueError,'too small'): make(self.raw)
        self.raw['dimensions']['joints']['J1']['module_retreat_mm']=1000
        with self.assertRaisesRegex(ValueError,'insufficient axial'): make(self.raw)
    def test_coaxial_stack_and_ambiguous_cluster(self):
        a=read_input(SKILL/'assets/two-revolute.json'); js=a['model']['joints']
        js[0]['origin']={'xyz_m':[0,0,.4],'rpy_rad':[0,0,0]}; js[0]['axis']=[0,0,1]
        js[1]['origin']={'xyz_m':[0,0,0],'rpy_rad':[0,0,0]}; js[1]['axis']=[0,0,1]
        n,g=make(a); self.assertEqual(n['joint_layout']['pairs'][0]['template'],'coaxial_stack'); self.assertEqual(packing_checks(n)['status'],'passed')
        self.raw['model']['joints'][2]['origin']['xyz_m']=[0,0,0]
        n,g=make(self.raw); self.assertEqual(n['joint_layout']['status'],'needs_review')
        self.assertTrue(n['joint_layout']['unresolved'])
    def test_hollow_sleeve_is_closed(self):
        V,F=annulus(.06,.03,.08)
        self.assertGreaterEqual(np.linalg.norm(V[:,:2],axis=1).min(),.03-1e-12)
        with tempfile.TemporaryDirectory() as t:
            stl(Path(t)/'sleeve.stl',V,F); self.assertTrue(mesh_checks(Path(t))['closed_and_oriented'])
        tri=V[F]; volume=np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2])).sum()/6
        self.assertGreater(volume,0); self.assertLess(volume,np.pi*.06**2*.08)
    def test_reversed_axis_and_crossing_shaft_clearance(self):
        n,g=make(self.raw)
        for pair in n['joint_layout']['pairs']:
            a,b=[next(j for j in n['model']['joints'] if j['name']==name) for name in pair['joints']]
            va=next(p['vertices'] for p in g[a['child']]['parts'] if p['name']==a['name']+'_output_shaft')
            vb=next(p['vertices'] for p in g[b['child']]['parts'] if p['name']==b['name']+'_output_shaft')
            O=tf(b['origin']); vb=vb@O[:3,:3].T+O[:3,3]; along=-np.array(pair['direction_local'])
            self.assertGreaterEqual(float(np.min(vb@along)-np.max(va@along)),.002-1e-10)
        self.raw['model']['joints'][0]['axis']=[0,0,-1]
        reversed_n,reversed_g=make(self.raw)
        # Reversing the input axis must not move the physical stator center.
        np.testing.assert_allclose(np.array(n['joint_layout']['motor_envelopes'][0]['T'])[:3,3],np.array(reversed_n['joint_layout']['motor_envelopes'][0]['T'])[:3,3])
        self.assertEqual(packing_checks(reversed_n)['status'],'passed')

if __name__=='__main__': unittest.main()
