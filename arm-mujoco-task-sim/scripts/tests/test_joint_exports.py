import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from joint_exports import export, STATE, SECTION, DRIVE


class JointExportTests(unittest.TestCase):
    def fixture(self, root):
        root = Path(root)
        for directory in ['data', 'reports', 'input', 'figures']:
            (root/directory).mkdir()
        meta = dict(joints=[dict(name='hinge', id=0, type='hinge', position_unit='rad', effort_unit='N m'),
                            dict(name='slide', id=1, type='slide', position_unit='m', effort_unit='N')],
                    load_sections=[dict(joint=n, joint_type=t) for n, t in [('hinge', 'revolute'), ('slide', 'prismatic'), ('mount', 'fixed')]],
                    load_coverage=dict(expected_arm_joints=['hinge', 'slide']),
                    actuators=[dict(name='motor', transmission_type=0, transmission_ids=[0, -1])],
                    load_section_semantics=dict(frame='site', sign='child towards parent'))
        (root/'data/metadata.json').write_text(json.dumps(meta))
        (root/'input/task_contract.json').write_text(json.dumps(dict(task_description='<script>unsafe</script>', assumptions=['trial'])))
        (root/'reports/result.json').write_text(json.dumps(dict(status='incomplete_or_failed', reason='timeout')))
        def table(name, fields, rows):
            with (root/('data/'+name+'.csv')).open('w', newline='', encoding='utf-8-sig') as f:
                w = csv.DictWriter(f, fields); w.writeheader(); w.writerows(rows)
        rows, loads, drives = [], [], []
        for i, time in enumerate(['0.0', '0.10000000000000001']):
            lead = dict(time_s=time, cycle='1', phase=['initial', 'hold'][i])
            for name, kind in [('hinge', 'hinge'), ('slide', 'slide')]:
                row = dict(lead, joint=name, type=kind, **{s: '0.0' for s in STATE})
                row.update(position=str(-.1+i*.3), velocity='0.5', effort=['-2.0', '4.0'][i], power_W=['-1.0', '2.0'][i])
                rows.append(row)
            for name in ['hinge', 'slide', 'mount']:
                loads.append(dict(lead, joint=name, site='load_'+name, **{s: '1.2345678901234567' for s in SECTION}))
            drives.append(dict(lead, actuator='motor', **{s: '-0.12500000000000001' for s in DRIVE}))
        table('joints', ['time_s', 'cycle', 'phase', 'joint', 'type']+STATE, rows)
        table('joint_loads', ['time_s', 'cycle', 'phase', 'joint', 'site']+SECTION, loads)
        table('actuators', ['time_s', 'cycle', 'phase', 'actuator']+DRIVE, drives)
        (root/'reports/load_statistics.csv').write_text('kind,joint,signal,scope,time_rms\ngeneralized,hinge,effort,all,3.5\n')
        (root/'figures/000-hinge_position.svg').write_text('<svg xmlns="http://www.w3.org/2000/svg"><text>hinge | position</text><path d="M0 0L1 1"/></svg>')
        return root

    def rows(self, path):
        with path.open(encoding='utf-8-sig', newline='') as f:
            return list(csv.DictReader(f))

    def test_all_modules_lossless_join_fixed_interface_and_offline_html(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(tmp)
            raw = (run/'data/joints.csv').read_bytes()
            result = export(run)
            self.assertEqual(result['status'], 'exported')
            self.assertEqual(result['frames'], 2)
            self.assertEqual([m['joint'] for m in result['modules']], ['hinge', 'slide', 'mount'])
            for entry in result['modules']:
                rows = self.rows(run/entry['state_csv'])
                self.assertEqual(len(rows), 2)
                self.assertEqual(rows[1]['time_s'], '0.10000000000000001')
                self.assertEqual(rows[1]['force_x_N'], '1.2345678901234567')
                if entry['joint'] == 'mount':
                    self.assertEqual(rows[0]['position'], '')
                    self.assertEqual(rows[0]['state_available'], '0')
                    self.assertEqual(rows[0]['section_available'], '1')
                elif entry['joint'] == 'hinge':
                    self.assertEqual(rows[0]['effort'], '-2.0')
                    self.assertEqual(rows[0]['drive_ctrl'], '-0.12500000000000001')
                    self.assertAlmostEqual(float(rows[0]['position_deg']), -.1*180/3.141592653589793)
                    self.assertEqual(entry['effort_time_rms'], 3.5)
                    self.assertEqual(len(self.rows(run/entry['actuator_csv'])), 2)
                else:
                    self.assertEqual(rows[0].get('position_deg'), '')
                    self.assertEqual(rows[0]['drive_ctrl'], '')
            wide = self.rows(run/result['wide_csv'])
            self.assertEqual(wide[1]['hinge.effort'], '4.0')
            html = (run/result['html']).read_text()
            for name in ['hinge', 'slide', 'mount']:
                self.assertIn('data-joint="'+name+'"', html)
            self.assertIn('data:image/svg+xml;base64,', html)
            self.assertNotIn('src="../figures', html)
            self.assertNotIn('<script>unsafe', html)
            self.assertIn('&lt;script&gt;unsafe', html)
            self.assertIn('incomplete_or_failed', html)
            self.assertEqual((run/'data/joints.csv').read_bytes(), raw)

    def test_missing_joint_is_partial_not_silent_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(tmp)
            meta = json.loads((run/'data/metadata.json').read_text())
            meta['load_coverage']['expected_arm_joints'].append('missing')
            (run/'data/metadata.json').write_text(json.dumps(meta))
            result = export(run)
            self.assertEqual(result['status'], 'partial')
            joint = next(m for m in result['modules'] if m['joint'] == 'missing')
            self.assertEqual(joint['status'], 'missing_state')
            self.assertEqual(joint['state_samples'], 0)
            self.assertEqual(self.rows(run/joint['state_csv'])[0]['effort'], '')

    def test_misalignment_replaces_stale_success_with_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(tmp); export(run)
            p = run/'data/joint_loads.csv'
            p.write_text(p.read_text(encoding='utf-8-sig').replace('0.10000000000000001', '0.2'))
            with self.assertRaisesRegex(ValueError, 'not exactly aligned'):
                export(run)
            self.assertEqual(json.loads((run/'reports/joint_exports.json').read_text())['status'], 'failed')
            self.assertIn('整理失败', (run/'reports/report.html').read_text())

    def test_missing_actuator_tail_is_visible_as_partial(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(tmp)
            p = run/'data/actuators.csv'
            lines = p.read_text(encoding='utf-8-sig').splitlines()
            p.write_text('\n'.join(lines[:-1])+'\n')
            result = export(run)
            self.assertEqual(result['status'], 'partial')
            joint = result['modules'][0]
            self.assertEqual(joint['status'], 'missing_actuator')
            rows = self.rows(run/joint['state_csv'])
            self.assertEqual(rows[1]['drive_ctrl'], '')

    def test_multiple_actuators_are_kept_separate_without_averaging(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(tmp)
            p = run/'data/metadata.json';meta = json.loads(p.read_text())
            meta['actuators'].append(dict(name='second', transmission_type=0, transmission_ids=[0, -1]))
            p.write_text(json.dumps(meta))
            p = run/'data/actuators.csv';rows = self.rows(p)
            with p.open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0]));writer.writeheader()
                for row in rows:
                    writer.writerow(row);writer.writerow({**row, 'actuator':'second', 'ctrl':'5.0'})
            result = export(run);entry = result['modules'][0]
            self.assertEqual(result['status'], 'exported')
            self.assertEqual(entry['actuator_samples'], 4)
            self.assertEqual(len(self.rows(run/entry['actuator_csv'])), 4)
            state = self.rows(run/entry['state_csv'])[0]
            self.assertEqual(state['actuator_count'], '2')
            self.assertEqual(state['drive_ctrl'], '')


if __name__ == '__main__':
    unittest.main()
