#!/usr/bin/env python3
"""Mask close LD06 self-returns without claiming free space behind them."""
import copy
import math


def filter_scan(scan, minimum_range):
    result = copy.deepcopy(scan)
    cutoff = max(minimum_range, scan.range_min)
    result.range_min = cutoff
    result.ranges = [math.nan if r < cutoff else r for r in scan.ranges]
    return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node('ld06_range_filter')
    cutoff = float(node.declare_parameter('minimum_range', 0.20).value)
    if not math.isfinite(cutoff) or cutoff <= 0:
        raise ValueError('minimum_range must be finite and positive')
    publisher = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    subscription = node.create_subscription(
        LaserScan, '/scan_raw',
        lambda msg: publisher.publish(filter_scan(msg, cutoff)),
        qos_profile_sensor_data)
    node.get_logger().info(f'/scan_raw -> /scan: returns below {cutoff:.3f} m become NaN')
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
