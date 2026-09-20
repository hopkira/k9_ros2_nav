# OAK-D Lite initial commissioning

Pi Ubuntu 24.04 / Jazzy, depthai_ros_driver_v3 installed from apt.
Detected device ID 1844301061EDC21200, model OAK-D-LITE; IMU unavailable.
Device USB permissions use vendor 03e7, mode 0660, group plugdev.

Depth-only pipeline, no neural networks, IR and IMU disabled. Stereo publication
is set to 15 Hz. Requested width/height are 640x400 but observed output is
320x200: do not assume requested dimensions match actual output. The camera
reported USB HIGH (USB 2) after boot. This configuration is an initial test,
not a validated collision-avoidance sensor configuration.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ~/k9_ws/oak_bringup/oak.launch.py
```

Use the same ROS networking settings as the rest of K9 (domain 9). At initial
deployment a standalone launch is running as PID 5045. Do not start a duplicate.
Logs: `~/k9_ws/oak_bringup/live.log` on the Pi.

Topics:
- `/oak/stereo/image_raw`: 16UC1 depth, measured 15.002 Hz over 100 frames.
- `/oak/stereo/camera_info`: camera calibration.
- `/oak/points`: uncoloured XYZ from depth_image_proc in the camera container.

The driver publishes calibrated internal camera frames attached to `oakd_link`;
the shared K9 description owns that mounting frame. For standalone RViz use
`oak_rgb_camera_optical_frame` as fixed frame, add PointCloud2 `/oak/points`,
Best Effort / Volatile QoS. For robot-relative inspection the shared description
must be running; then use `base_link` as fixed frame.

Outstanding: verify cloud geometry and CameraInfo correspondence, measure CPU
and USB load, check USB 3 cable/link, reconcile model camera height (32.44 cm
above nominal floor) against owner's intended 26 cm, and verify startup/shutdown
before integration into standard bring-up. The first bounded driver shutdown
required signal escalation; graceful shutdown remains to be investigated.

Initial cloud check: 60 messages, 320x200 XYZ points, 1,024,000 bytes/message,
about 6.15 Hz observed at the Python subscriber. This is lower than the measured
depth-only rate; cloud conversion/transport needs profiling before collision use.
