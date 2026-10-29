import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('workspace_import', Path(__file__).parents[1] / 'import_urdf.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

URDF = '''<robot name="test"><link name="base"/><link name="arm"/><link name="tip"/>
<joint name="joint" type="revolute"><parent link="base"/><child link="arm"/>
<origin xyz="0 0.17 0" rpy="0.1 0.2 0.3"/><axis xyz="0 -1 0"/>
<limit lower="-1" upper="2"/></joint>
<joint name="tool" type="fixed"><parent link="arm"/><child link="tip"/>
<origin xyz="0 0 -0.6"/></joint></robot>'''


class ImportTests(unittest.TestCase):
    def test_serial_path_keeps_installed_origin_axis_limits_and_fixed_tool(self):
        out = module.convert(URDF, 'base', 'tip', 100, 7)
        self.assertEqual(out['model']['joints'][0]['origin']['xyz_m'], [0, .17, 0])
        self.assertEqual(out['model']['joints'][0]['axis'], [0, -1, 0])
        self.assertEqual(out['model']['joints'][0]['limits'], [-1, 2])
        self.assertEqual(out['model']['joints'][1]['type'], 'fixed')
        self.assertNotIn('axis', out['model']['joints'][1])
        self.assertEqual(out['sampling']['samples'], 100)

    def test_unknown_or_reversed_base_tip_path_rejected(self):
        for base, tip in [('missing', 'tip'), ('tip', 'base'), ('base', 'base')]:
            with self.assertRaises(ValueError):
                module.convert(URDF, base, tip)

    def test_mimic_continuous_entities_and_nonfinite_data_rejected(self):
        for data in [URDF.replace('type="revolute"', 'type="continuous"'),
                     URDF.replace('<limit', '<mimic joint="other"/><limit'),
                     '<!DOCTYPE robot>' + URDF,
                     URDF.replace('lower="-1"', 'lower="nan"')]:
            with self.assertRaises(ValueError):
                module.convert(data, 'base', 'tip')


if __name__ == '__main__':
    unittest.main()
