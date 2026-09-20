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

Use the same ROS networking settings as the rest of K9 (domain 9). After enabling speckle filtering, the standalone launch was restarted as PID
6386 (commissioning snapshot; check current processes before launching). Do not
start a duplicate. Logs: `~/k9_ws/oak_bringup/speckle.log` on the Pi.

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

## Adaptive floor rejection (temporary mount)

Run `oak_floor_filter.launch.py` alongside the camera. It is also installed at
`share/k9_ros2_nav/oak/` when this package is built. A standalone filter was
restarted on the Pi as PID 6995 for commissioning; avoid starting a second copy.

- `/oak/points` remains unchanged.
- `/oak/obstacles` contains finite forward-region points outside the floor band.
- `/oak/floor` shows rejected floor points when subscribed.
- `/oak/floor_filter/status` reports fit validity, height, tilt, point counts and
  processing time as JSON in a String message.

Outputs use Reliable / Volatile QoS, compatible with RViz Reliable or Best
Effort. Fixed frame remains `oak_rgb_camera_optical_frame` for camera-only views.
The output retains input frame and acquisition timestamp. No TF is changed.

The filter selects a near-horizontal RANSAC plane, constrained to 18–34 cm below
the optical origin and at most 15 degrees tilt, with minimum support and spatial
extent. It refits each processed cloud, at most 5 Hz, with a one-message input
queue. It removes points within +/-2.5 cm of the plane; smaller protrusions may
be lost. It retains below-plane anomalies. Forward ROI: 0.2–3 m optical Z,
+/-1.5 m optical X. This is not full-camera or full-robot coverage.

If a fit fails it passes the finite ROI through without floor removal, rather
than reusing a stale plane. Missing input yields no new output. An unexpected
frame produces an error status and no output. Downstream stale-data monitoring
is still required; this node is not wired into collision control.

Final intended mount is level, with optical centre 26 cm above floor. Current
physical mount is temporary (~24 cm with ~2.3-degree floor-relative tilt). Do not
use the uncorrected model camera height for floor-height rejection.

Five synthetic tests cover tilted floor with a 6 cm obstacle, wall-only input,
sparse/invalid input, and a partly visible floor with a 3 cm obstacle. Live visual confirmation with actual obstacles is
still required, including loss of floor visibility and thin/low objects.

## Camera speckle filtering

Enabled `stereo.i_enable_speckle_filter` in oak.yaml and verified the live
parameter is true. The driver reports its default speckle range as 50. Spatial
and temporal filters remain disabled; the adaptive floor settings are unchanged.
Configuration was copied to the Pi standalone and package source locations and
the navigation package rebuilt.

After restart, 34 obstacle clouds arrived at 4.99 Hz. The latest floor fit was
invalid and therefore passed through the finite ROI; the owner confirmed the
camera or scene had changed. Repeat the floor/book comparison in the previous
view before judging noise reduction or small-object retention.

The driver's stop service returned success but its component container then
crashed (exit -11). Restarting the standalone launch restored the stream.
Graceful shutdown remains unresolved.

## Floor retune for the changed view

The observed floor occupied about 23–26% of the original fitting region, below
its 40% acceptance threshold. The commissioning launch now loads
`floor_filter.yaml`, setting `min_floor_support: 0.20`; direct script invocation
without this file retains the conservative 0.40 default. RANSAC now checks 500
candidate planes rather than 100. Height limits (18–34 cm), tilt limit (15°),
minimum 250 inliers, spatial extent requirements, and +/-2.5 cm rejection band
are unchanged. Lower support permits fitting amid more clutter but also raises
the risk of accepting another horizontal surface; this is temporary-view tuning.

Live verification after restart: 62/62 valid estimates, 5.04 Hz. Height estimates
ranged 0.2354–0.2644 m; latest height 0.2519 m, tilt 6.38°, support 0.314,
8,063 floor points removed, processing 59.5 ms. Five synthetic tests passed,
including partial floor visibility with a 3 cm obstacle and wall-only rejection
at the lower support threshold. Recheck the physical book: synthetic retention
does not establish small-object performance with this observed fit variation.
