import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('floor_filter', Path(__file__).resolve().parents[1] / 'commissioning/oak_bringup/floor_filter.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class FloorFilterTest(unittest.TestCase):
    def test_tilted_floor_removed_and_small_obstacle_retained(self):
        rng = np.random.default_rng(8)
        x = rng.uniform(-.7, .7, 4500)
        z = rng.uniform(.5, 1.8, 4500)
        normal = np.array([-.02, 1., .035]); normal /= np.linalg.norm(normal)
        y = (.24-normal[0]*x-normal[2]*z)/normal[1]
        floor = np.column_stack((x, y, z)) + rng.normal(0, .001, (4500, 3))
        obstacle = floor[:300].copy(); obstacle[:, 1] -= .06
        kept, removed, plane = m.split_cloud(np.vstack((floor, obstacle)))
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane[1], .24, delta=.005)
        self.assertEqual(len(kept), 300)
        self.assertEqual(len(removed), 4500)

    def test_no_floor_does_not_remove_wall(self):
        rng = np.random.default_rng(2)
        wall = np.column_stack((rng.uniform(-.7,.7,1200), rng.uniform(-.3,.5,1200), np.ones(1200)))
        kept, removed, plane = m.split_cloud(wall)
        self.assertIsNone(plane)
        self.assertEqual(len(kept), len(wall))
        self.assertEqual(len(removed), 0)

    def test_sparse_and_invalid_cloud(self):
        points = np.array([[0.,.24,1.],[np.nan,0.,1.]])
        kept, _, plane = m.split_cloud(points)
        self.assertIsNone(plane)
        self.assertEqual(len(kept), 1)

    def test_partly_visible_floor_and_three_cm_obstacle(self):
        rng = np.random.default_rng(12)
        x = rng.uniform(-.7, .7, 1800)
        z = rng.uniform(.5, 1.8, 1800)
        normal = np.array([0., 1., -.105]); normal /= np.linalg.norm(normal)
        y = (.26-normal[2]*z)/normal[1]
        floor = np.column_stack((x, y, z)) + rng.normal(0, .001, (1800, 3))
        wall = np.column_stack((rng.uniform(-.7, .7, 4500),
                                rng.uniform(.07, .8, 4500), np.full(4500, 1.9)))
        obstacle = floor[:100].copy(); obstacle -= .03*normal
        points = np.vstack((floor, wall, obstacle))
        self.assertIsNone(m.fit_floor(points))
        kept, removed, plane = m.split_cloud(points, min_support=.20)
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane[1], .26, delta=.005)
        self.assertGreater(len(removed), 1700)
        # All of the synthetic 3 cm obstacle must remain outside the floor band.
        self.assertTrue(np.all(np.abs(plane[1]-obstacle @ plane[0]) > .025))

    def test_wall_only_with_relaxed_support_still_passes_through(self):
        rng = np.random.default_rng(9)
        wall = np.column_stack((rng.uniform(-.7,.7,2000),
                                rng.uniform(.07,.8,2000), np.ones(2000)))
        kept, removed, plane = m.split_cloud(wall, min_support=.20)
        self.assertIsNone(plane)
        self.assertEqual(len(kept), len(wall))
        self.assertEqual(len(removed), 0)
