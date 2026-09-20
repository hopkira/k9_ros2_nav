"""Sample live LD06 scans for 12 seconds and report transport/data sanity."""
import json
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('k9_ld06_scan_check')
frames = []

def receive(msg):
    finite = [x for x in msg.ranges if math.isfinite(x)]
    frames.append({
        'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        'frame': msg.header.frame_id, 'bins': len(msg.ranges),
        'finite': len(finite), 'min': min(finite) if finite else None,
        'max': max(finite) if finite else None,
        'out_of_bounds': sum(x < msg.range_min or x > msg.range_max for x in finite),
        'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
        'angle_increment': msg.angle_increment,
        'scan_time': msg.scan_time, 'time_increment': msg.time_increment,
        'age_sec': node.get_clock().now().nanoseconds / 1e9 -
                   (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9),
    })

sub = node.create_subscription(LaserScan, '/scan_raw', receive, qos_profile_sensor_data)
end = time.monotonic() + 12
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.25)
summary = {'messages': len(frames)}
if frames:
    summary.update(first=frames[0], last=frames[-1],
                   bin_counts=sorted(set(f['bins'] for f in frames)),
                   finite_min=min(f['finite'] for f in frames),
                   max_age_sec=max(f['age_sec'] for f in frames),
                   invalid_range_count=sum(f['out_of_bounds'] for f in frames))
if len(frames) > 1:
    dt = frames[-1]['stamp'] - frames[0]['stamp']
    summary['hz'] = (len(frames)-1)/dt if dt > 0 else None
    summary['monotonic'] = all(b['stamp'] > a['stamp'] for a,b in zip(frames, frames[1:]))
print(json.dumps(summary, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if len(frames) >= 20 and summary.get('monotonic') and summary['finite_min'] > 0 else 1)
