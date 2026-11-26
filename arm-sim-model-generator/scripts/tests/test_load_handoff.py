"""Verify generator output preserves complete active/fixed load interfaces."""
import json
from pathlib import Path
import sys
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

SCRIPTS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SCRIPTS))
from arm_sim.pipeline import generate, validate
from arm_sim.load_handoff import check_xml


class HandoffTests(unittest.TestCase):
    def test_generated_fixed_and_active_sections_survive_compile_and_relocation(self):
        import mujoco as mj
        source=json.loads((SCRIPTS.parent/'assets/two-revolute-driven.json').read_text())
        source['model']['joints'].append(dict(name='tool_mount',type='fixed',origin=dict(xyz_m=[.1,0,0],rpy_rad=[0,0,0]),
                                            components=[dict(name='mount',mass_kg=.1,com_m=[.02,0,0],inertia_frame='link',
                                                             inertia_kg_m2=[[.001,0,0],[0,.001,0],[0,0,.001]])]))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'input.json'; p.write_text(json.dumps(source))
            result=generate(p,Path(tmp)/'output',['mujoco'])
            self.assertNotIn('error',result)
            run=Path(result['run_directory']); h=json.loads((run/'report/load_handoff.json').read_text())
            self.assertEqual(h['active_joint_names'],['J1','J2']); self.assertEqual(h['fixed_joint_names'],['tool_mount'])
            m=mj.MjModel.from_xml_path(str(run/'mujoco/robot.xml'))
            self.assertEqual(m.njnt,2); self.assertEqual(m.nsensor,6)
            self.assertEqual(m.body(int(m.site_bodyid[m.site('load_site_tool_mount').id])).name,'tool_mount_link')
            relocated=Path(tmp)/'relocated'; shutil.copytree(run/'mujoco',relocated)
            moved=mj.MjModel.from_xml_path(str(relocated/'robot.xml'))
            self.assertEqual(moved.nsensor,6)
            n=json.loads((run/'report/normalized_model.json').read_text()); tree=ET.parse(run/'mujoco/robot.xml')
            self.assertEqual(check_xml(n,tree)['section_count'],3)
            tree.getroot().find('sensor').remove(tree.getroot().find('sensor/force'))
            with self.assertRaisesRegex(ValueError,'Missing or mismatched'): check_xml(n,tree)
            self.assertNotEqual(validate(run).get('error'),'resource inventory or SHA256 mismatch')
            # The handoff itself is fingerprinted, not only the robot mesh/XML.
            (run/'report/load_handoff.json').write_text('{}')
            self.assertEqual(validate(run)['status'],'failed')


if __name__=='__main__': unittest.main()
