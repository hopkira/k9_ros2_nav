import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('scan_filter', Path(__file__).resolve().parents[1] / 'commissioning/ld06_bringup/filter_scan.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ScanFilterTest(unittest.TestCase):
    def test_mask_preserves_geometry_timing_and_raw_data(self):
        raw = SimpleNamespace(ranges=[0.025, 0.199, 0.20, 1.0, math.nan, math.inf],
                              range_min=0.03, range_max=12.0,
                              header={'stamp': 123, 'frame_id': 'base_laser'},
                              angle_increment=0.01, time_increment=0.002,
                              intensities=[1, 2, 3, 4, 5, 6])
        out = module.filter_scan(raw, 0.20)
        self.assertTrue(all(math.isnan(out.ranges[i]) for i in (0, 1, 4)))
        self.assertEqual(out.ranges[2:4], [0.20, 1.0])
        self.assertEqual(out.ranges[5], math.inf)
        self.assertEqual(raw.ranges[0], 0.025)
        self.assertEqual(raw.range_min, 0.03)
        self.assertEqual(out.range_min, 0.20)
        for name in ('header', 'angle_increment', 'time_increment', 'intensities', 'range_max'):
            self.assertEqual(getattr(out, name), getattr(raw, name))

    def test_respects_sensor_minimum(self):
        out = module.filter_scan(SimpleNamespace(ranges=[0.22, 0.3], range_min=0.25), 0.20)
        self.assertTrue(math.isnan(out.ranges[0]))
        self.assertEqual(out.ranges[1], 0.3)
        self.assertEqual(out.range_min, 0.25)
