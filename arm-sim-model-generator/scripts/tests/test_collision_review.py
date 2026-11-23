"""Run from the skill folder: python -m unittest discover -s scripts/tests -p test_collision_review.py -v"""
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_sim.collision_review import review
from arm_sim.geometry import build_geometry
from arm_sim.model import normalize, read_input
from arm_sim.validation import cases

ASSETS = Path(__file__).resolve().parents[2] / 'assets'


class CollisionReviewTests(unittest.TestCase):
    def test_legal_shaft_and_interfering_shells_on_the_same_link_pair(self):
        raw = read_input(ASSETS / 'seven-axis-driven.json')
        model = normalize(raw)
        with tempfile.TemporaryDirectory() as tmp:
            meshes = Path(tmp)
            build_geometry(model, meshes)
            # Isolate one genuine generated shaft/bore, retaining its original
            # ownership and transform. Place two shell proxies on those same links.
            first = model['model']['joints'][0]
            parent, child = first['parent'], first['child']
            keep = (first['name'] + '_stator', first['name'] + '_output_shaft')
            for link, g in model['geometry'].items():
                g['parts'] = [p for p in g['parts'] if p['name'] in keep]
            clean = review(model, meshes, cases(model))
            self.assertFalse(clean['findings'])
            self.assertEqual(clean['mating_pairs'][0]['parts'], [parent+'/'+keep[0], child+'/'+keep[1]])
            self.assertTrue(clean['mating_pairs'][0]['bore_radius_m'] > 0)
            from arm_sim.math3d import forward
            frames = forward(model, model['poses']['initial'])
            for owner in (parent, child):
                pose = np.linalg.inv(frames[owner])
                pose[:3,3] += pose[:3,:3] @ [2.,0.,0.]
                model['geometry'][owner]['parts'].append(dict(name='test_housing',collision_kind='box',
                    collision=True,size=[.1,.1,.1],T=pose.tolist(),mesh='test_visual.stl'))
            result = review(model, meshes, cases(model))
            collision = next(f for f in result['findings'] if all(x.endswith('/test_housing') for x in f['parts']))
            self.assertEqual(result['status'],'failed')
            self.assertEqual(collision['method'],'box_sat_overlap')
            self.assertGreater(collision['overlap_evidence_m'],.09)
            self.assertIn('q', collision)

    def test_flange_and_declared_recessed_proxy(self):
        model = normalize(read_input(ASSETS / 'two-revolute.json'))
        with tempfile.TemporaryDirectory() as tmp:
            meshes = Path(tmp)
            build_geometry(model, meshes)
            baseline = review(model, meshes, cases(model))
            self.assertEqual(baseline['status'], 'passed')
            self.assertTrue(all(s['status']=='passed' for s in baseline['contact_surfaces']))
            flange_id = baseline['contact_surfaces'][0]['part']
            model['contact_surfaces'] = [dict(part=flange_id, point_link_m=baseline['contact_surfaces'][0]['point_link_m'],
                                               tolerance_m=.00001, source='fixture drawing tolerance')]
            flange = next(p for p in model['geometry'][flange_id.split('/')[0]]['parts'] if p['name']=='flange')
            r, h = flange['size']
            flange['collision_size'] = [r,h/2]
            result = review(model, meshes, cases(model))
            self.assertEqual(result['status'], 'failed')
            self.assertGreater(result['contact_surfaces'][0]['proxy_signed_distance_m'],.00001)

    def test_interpolation_detects_slider_shell_and_preserves_guide_fit(self):
        model = normalize(read_input(ASSETS / 'skew-prismatic.json'))
        with tempfile.TemporaryDirectory() as tmp:
            meshes = Path(tmp)
            build_geometry(model, meshes)
            result = review(model, meshes, cases(model))
        self.assertEqual(result['status'], 'failed')
        self.assertGreater(result['coverage']['segments'][1]['subdivisions'], 1)
        self.assertTrue(any(r['kind']=='guide_and_slider' for r in result['mating_pairs']))
        self.assertTrue(any(set(f['parts']) == {'shoulder_link/beam','slide_link/slider'}
                            and f['status']=='failed' for f in result['findings']))

    def test_side_support_and_fairing_clear_the_opposite_output_plate(self):
        model = normalize(read_input(ASSETS / 'seven-axis-driven.json'))
        with tempfile.TemporaryDirectory() as tmp:
            meshes = Path(tmp)
            build_geometry(model, meshes)
            result = review(model, meshes, cases(model))
        reported = [f['parts'] for f in result['findings']]
        for a,b,carrier,output in [('J1','J2','J1_link','J2_link'),('J5','J6','J5_link','J6_link')]:
            plate = output + '/' + b + '_output_plate'
            for part in (a + '_output_fairing', b + '_support', b + '_stator_lug'):
                self.assertNotIn([carrier + '/' + part, plate], reported)
        self.assertFalse(result['named_pose_results']['geometric_zero']['in_joint_ranges'])
        self.assertTrue(result['named_pose_results']['initial']['in_joint_ranges'])
