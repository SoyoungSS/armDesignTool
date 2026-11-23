"""Check scoped handoff and retain reproducible out-of-scope interference."""
import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_sim.model import normalize, read_input
from arm_sim.pipeline import generate
from check_path import check
from default_model import DEFAULTS, assess


class DefaultsTests(unittest.TestCase):
    def test_payload_is_an_independent_worksheet_condition(self):
        bare = normalize(read_input(DEFAULTS/'worksheet.json'))
        loaded = normalize(read_input(DEFAULTS/'worksheet-with-3kg.json'))
        for n, expected in ((bare, 3.612), (loaded, 6.612)):
            self.assertAlmostEqual(sum(c['mass_kg'] for cs in n['components'].values() for c in cs), expected)
        self.assertEqual(bare['model'], loaded['model'])
        self.assertTrue(bare['physics']['fixed_base_massless'])
        self.assertNotIn('payload', bare['physics'])

    @unittest.skipUnless(importlib.util.find_spec('mujoco'), 'requires MuJoCo')
    def test_default_nominal_motion_and_confirmed_interference(self):
        for arm in ('worksheet','fr3'):
            with self.subTest(arm=arm), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                source = DEFAULTS / (arm+'.json')
                run = Path(generate(source, root, runtime=True)['run_directory'])
                result = assess(arm,source,run,root)
                self.assertEqual(result['status'],'passed')
                self.assertEqual(result['full_range_collision'],'failed')
                bad = result['full_range_worst_case']
                csv_path = root / 'bad.csv'
                with csv_path.open('w',newline='') as f:
                    writer = csv.DictWriter(f,fieldnames=['cycle','phase','time_s',*bad['q']])
                    writer.writeheader()
                    writer.writerow(dict(cycle=1,phase=bad['pose'],time_s=0,**bad['q']))
                replay = check(run,csv_path)
                self.assertEqual(replay['status'],'failed')
                self.assertIn(bad['parts'],[x['parts'] for x in replay['findings']])
                self.assertEqual(replay['findings'][0]['phase'],bad['pose'])
