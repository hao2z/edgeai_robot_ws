"""Package imports work without the old navigation scripts sys.path bridge."""
import ast
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]


class ModuleBoundaries(unittest.TestCase):
    def test_installed_packages_expose_task_and_geometry_modules(self):
        from edgeai_tasks.cooperative_rescue import CooperativeRescue
        from edgeai_tasks.normal_rescue import Rescue
        from edgeai_search.frustum_search import Camera
        from edgeai_nav.geometry import compose
        self.assertTrue(CooperativeRescue and Rescue and Camera)
        self.assertEqual(compose([0,0,0],[1,2,0]),[1.,2.,0.])

    def test_navigation_never_imports_task_layer(self):
        for path in (ROOT/'src/edgeai_navigation').rglob('*.py'):
            for node in ast.walk(ast.parse(path.read_text())):
                names=[node.module or ''] if isinstance(node,ast.ImportFrom) else [n.name for n in node.names] if isinstance(node,ast.Import) else []
                self.assertFalse(any(n.startswith('edgeai_tasks') for n in names),str(path))

    def test_s1_and_explicit_heading_have_different_planners(self):
        import sys
        sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
        from goal_policy import POLICIES
        self.assertNotEqual(POLICIES['heading'][1],POLICIES['rescue'][1])


if __name__=='__main__':unittest.main()
