#!/usr/bin/env python3
"""Conservative, optical-frame floor rejection for stationary-mount commissioning."""
import json
import math
import time
import numpy as np


def fit_floor(points, seed=42):
    """Return (downward normal, camera height, support), or None on weak evidence."""
    candidates = points[(points[:, 1] > .06) & (points[:, 2] > .35)
                        & (points[:, 2] < 2.5) & (np.abs(points[:, 0]) < 1.2)]
    if len(candidates) < 250:
        return None
    rng = np.random.default_rng(seed)
    if len(candidates) > 1800:
        candidates = candidates[rng.choice(len(candidates), 1800, replace=False)]
    best = None
    for _ in range(100):
        a, b, c = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if normal[1] < 0:
            normal = -normal
        distance = float(normal @ a)
        if normal[1] < math.cos(math.radians(15)) or not .18 <= distance <= .34:
            continue
        mask = np.abs(candidates @ normal-distance) < .015
        count = int(mask.sum())
        if best is None or count > best[0]:
            best = count, mask
    if best is None or best[0] < max(250, .40*len(candidates)):
        return None
    floor = candidates[best[1]]
    if np.ptp(floor[:, 0]) < .30 or np.ptp(floor[:, 2]) < .35:
        return None
    centre = floor.mean(axis=0)
    _, _, vectors = np.linalg.svd(floor-centre, full_matrices=False)
    normal = vectors[-1]
    if normal[1] < 0:
        normal = -normal
    distance = float(normal @ centre)
    if normal[1] < math.cos(math.radians(15)) or not .18 <= distance <= .34:
        return None
    return normal, distance, best[0]/len(candidates)


def split_cloud(points, floor_band=.025):
    points = points[np.isfinite(points).all(axis=1)]
    # Bound work and visualisation to the forward camera region.
    points = points[(points[:, 2] >= .20) & (points[:, 2] <= 3.0)
                    & (np.abs(points[:, 0]) <= 1.5)]
    plane = fit_floor(points)
    if plane is None:
        # Never clear space from a failed or stale plane estimate.
        return points, np.empty((0, 3), dtype=np.float32), None
    normal, distance, support = plane
    height = distance-points @ normal
    floor = np.abs(height) <= floor_band
    # Retain below-plane anomalies too; these are not evidence of free space.
    return points[~floor], points[floor], plane


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String

    rclpy.init()
    node = Node('oak_floor_filter')
    band = float(node.declare_parameter('floor_band_m', .025).value)
    rate = float(node.declare_parameter('max_rate_hz', 5.0).value)
    if not 0 < band <= .05 or not 0 < rate <= 15:
        raise ValueError('floor_band_m must be (0,.05], max_rate_hz (0,15]')
    incoming = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    outgoing = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    obstacles = node.create_publisher(PointCloud2, '/oak/obstacles', outgoing)
    floor_pub = node.create_publisher(PointCloud2, '/oak/floor', outgoing)
    status = node.create_publisher(String, '/oak/floor_filter/status', outgoing)
    pending = [None]
    def receive(message):
        pending[0] = message
    subscription = node.create_subscription(PointCloud2, '/oak/points', receive, incoming)
    def process():
        message, pending[0] = pending[0], None
        if message is None:
            return
        if message.header.frame_id != 'oak_rgb_camera_optical_frame':
            status.publish(String(data=json.dumps({'valid': False, 'error': 'unexpected frame',
                                                   'frame': message.header.frame_id})))
            return
        started = time.monotonic()
        points = point_cloud2.read_points_numpy(
            message, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        kept, removed, plane = split_cloud(points, band)
        obstacles.publish(point_cloud2.create_cloud_xyz32(message.header, kept))
        if floor_pub.get_subscription_count():
            floor_pub.publish(point_cloud2.create_cloud_xyz32(message.header, removed))
        report = {'valid': plane is not None, 'kept': len(kept), 'floor_points': len(removed),
                  'processing_ms': round((time.monotonic()-started)*1000, 1),
                  'source_stamp': message.header.stamp.sec + message.header.stamp.nanosec/1e9}
        if plane is not None:
            normal, distance, support = plane
            report.update(height_m=round(distance, 4), support=round(support, 3),
                          tilt_deg=round(math.degrees(math.acos(float(np.clip(normal[1], -1, 1)))), 2))
        status.publish(String(data=json.dumps(report)))
    timer = node.create_timer(1/rate, process)
    node.get_logger().info('Publishing /oak/obstacles; floor band +/- %.3f m' % band)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
