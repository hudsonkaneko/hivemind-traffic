"""Supervisor must not turn pre-close evidence or a native crash into success."""
import json
from pathlib import Path
import tempfile
import unittest

from scripts.demo_loop_showcase import read_probe_result, runtime_failure_lines


class ShowcaseSupervisorTests(unittest.TestCase):
    def test_missing_result_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertFalse(read_probe_result(Path(directory))['passed'])

    def test_partial_success_is_never_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'pre-close-result.json').write_text(json.dumps(
                dict(passed=True,completed_passes=7,cleanup_verified=False)))
            result=read_probe_result(root)
            self.assertFalse(result['passed'])
            self.assertFalse(result['cleanup_verified'])
            self.assertEqual(result['completed_passes'],7)
            self.assertEqual(result['incomplete_result_source'],'pre-close-result.json')

    def test_completed_result_has_priority(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'pre-close-result.json').write_text('{"passed": false}')
            (root/'probe-result.json').write_text('{"passed": true}')
            self.assertTrue(read_probe_result(root)['passed'])

    def test_fatal_is_not_masked_by_batch_wrapper_exit_zero(self):
        lines=runtime_failure_lines('normal\n[Warning] mild\n[Fatal] native access violation\n[Error] exception')
        self.assertEqual(len(lines),2)
        self.assertIn('[Fatal]',lines[0])


if __name__=='__main__':unittest.main()
