import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'retention', Path(__file__).resolve().parents[2] / 'scripts/maintenance/prune_test_results.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RetentionTest(unittest.TestCase):
    def test_budget_preserves_active_latest_pinned_and_incomplete(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in ('old', 'active', 'pinned', 'incomplete', 'latest'):
                directory = root / name
                directory.mkdir()
                (directory / 'log').write_bytes(b'x' * 100)
                if name != 'incomplete':
                    (directory / 'exit-code.txt').write_text('0')
            (root / 'pinned/.keep').touch()
            result = module.prune(root, max_bytes=1, keep_latest=0,
                                  active_names=['active', 'latest'], apply=True)
            self.assertEqual(result['removed'], ['old'])
            self.assertTrue(result['over_budget'])
            self.assertTrue((root / 'incomplete').exists())
            self.assertTrue((root / 'pinned').exists())

    def test_dry_run_never_deletes(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / 'old'
            directory.mkdir()
            (directory / 'exit-code.txt').write_text('0')
            result = module.prune(temp, max_bytes=0, keep_latest=0)
            self.assertEqual(result['removed'], ['old'])
            self.assertTrue(directory.exists())


if __name__ == '__main__':
    unittest.main()
