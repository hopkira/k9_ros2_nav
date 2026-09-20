"""ROS integration check: missing input, reader recreation, recovery, no stale clouds.

Run with ROS sourced; uses unique topics and does not command robot hardware.
"""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest
import uuid


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'requires ROS 2')
class FloorInputRecoveryTest(unittest.TestCase):
    def test_missing_input_recovers_without_replaying_clouds(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header, String
        from rclpy.qos import qos_profile_sensor_data
        context = rclpy.context.Context()
        rclpy.init(context=context)
        suffix = uuid.uuid4().hex
        prefix = '/floor_test_' + suffix
        node = rclpy.create_node('floor_test_' + suffix, context=context)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(node)
        reports, clouds = [], []
        node.create_subscription(String, prefix+'/status',
                                 lambda m: reports.append(json.loads(m.data)), 10)
        node.create_subscription(PointCloud2, prefix+'/obstacles',
                                 lambda m: clouds.append(m), qos_profile_sensor_data)
        pub = node.create_publisher(PointCloud2, prefix+'/points', qos_profile_sensor_data)
        script = Path(__file__).resolve().parents[1]/'commissioning/oak_bringup/floor_filter.py'
        cmd = [sys.executable, str(script), '--ros-args', '-r', '__node:=filter_'+suffix]
        for old, new in [('points','points'),('obstacles','obstacles'),('floor','floor'),
                         ('floor_filter/status','status')]:
            cmd += ['-r', '/oak/'+old+':='+prefix+'/'+new]
        env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
        child = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        def until(predicate, seconds):
            end = time.monotonic()+seconds
            while not predicate() and time.monotonic()<end:
                executor.spin_once(timeout_sec=.1)
            self.assertTrue(predicate(), 'Timed out waiting for filter state')
        try:
            until(lambda: any(r.get('state')=='waiting_for_cloud' for r in reports), 10)
            self.assertEqual(len(clouds), 0)
            header = Header(frame_id='oak_rgb_camera_optical_frame')
            header.stamp = node.get_clock().now().to_msg()
            message = point_cloud2.create_cloud_xyz32(header, [(0.,.24,1.)])
            end = time.monotonic()+5
            while not clouds and time.monotonic()<end:
                pub.publish(message)
                executor.spin_once(timeout_sec=.1)
            self.assertTrue(clouds)
            self.assertEqual(clouds[-1].header.stamp, header.stamp)
            # Let already queued data drain, then wait for the next no-input status.
            until(lambda: any('source_stamp' in r for r in reports), 3)
            prior_waits = sum(r.get('state')=='waiting_for_cloud' for r in reports)
            until(lambda: sum(r.get('state')=='waiting_for_cloud' for r in reports)>prior_waits, 8)
            count = len(clouds)
            end = time.monotonic()+1
            while time.monotonic()<end:
                executor.spin_once(timeout_sec=.1)
            self.assertEqual(len(clouds), count, 'Old cloud was replayed while input was absent')
            self.assertIsNone(child.poll())
        finally:
            child.terminate()
            try:
                child.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); child.communicate(timeout=5)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)


if __name__ == '__main__':
    unittest.main()
