"""Validate normals and mesh data before handing buffers to the graphics driver."""
import math
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'App'))
from gimbal_mesh import MATERIALS,build_assembly

class MeshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.assembly=build_assembly()

    def test_mesh_is_finite_and_has_normalised_surface_normals(self):
        for part in self.assembly.parts:
            self.assertIn(part.material,MATERIALS)
            self.assertEqual(len(part.vertices)%18,0,part.name)
            for i in range(0,len(part.vertices),6):
                values=part.vertices[i:i+6]
                self.assertTrue(all(math.isfinite(v) for v in values),part.name)
                self.assertAlmostEqual(sum(v*v for v in values[:3]),1,places=5,msg=part.name)

    def test_mechanical_parts_attach_to_the_correct_joint(self):
        parts={p.name:p for p in self.assembly.parts}
        self.assertEqual(parts['base pedestal'].group,'base')
        self.assertEqual(parts['upright casting'].group,'pan')
        self.assertEqual(parts['pitch stator'].group,'pan')
        self.assertEqual(parts['pitch hub'].group,'body')
        self.assertEqual(parts['payload main housing'].group,'body')
        self.assertEqual(parts['optical glass'].group,'body')

if __name__=='__main__':unittest.main(verbosity=2)
