# OAK-D Lite initial commissioning

Pi Ubuntu 24.04 / Jazzy, depthai_ros_driver_v3 installed from apt.
Detected device ID 1844301061EDC21200, model OAK-D-LITE; IMU unavailable.
Device USB permissions use vendor 03e7, mode 0660, group plugdev.

Depth-only pipeline, no neural networks, IR and IMU disabled. Stereo publication
is set to 15 Hz. Requested width/height are 640x400 but observed output is
320x200: do not assume requested dimensions match actual output. The camera
reported USB HIGH (USB 2) after boot. This configuration is an initial test,
not a validated collision-avoidance sensor configuration.

The standard system launch now owns camera and floor processing on Pi/all:

```bash
source /opt/ros/jazzy/setup.bash
source ~/k9_ws/install/local_setup.bash
# Use the normal K9 domain 9/CycloneDDS environment.
ros2 launch k9_system_pkg k9.launch.py platform:=pi
```

`enable_oak:=false` skips camera and floor processing;
`enable_oak_floor_filter:=false` skips only floor processing. Jetson launches
neither. Stop the previous standard/standalone sessions before starting another.
The verified integrated launch is PID 3500 (a commissioning snapshot, not a
persistent PID). Log: `~/k9_ws/oak_bringup/recovery-startup.log` on the Pi.
Standalone oak.launch.py remains available for commissioning only.

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
after integration into standard bring-up. The first bounded driver shutdown
required signal escalation; graceful shutdown remains to be investigated.

Initial cloud check: 60 messages, 320x200 XYZ points, 1,024,000 bytes/message,
about 6.15 Hz observed at the Python subscriber. This is lower than the measured
depth-only rate; cloud conversion/transport needs profiling before collision use.

## Adaptive floor rejection (temporary mount)

Run `oak_floor_filter.launch.py` alongside the camera. It is also installed at
`share/k9_ros2_nav/oak/` when this package is built. The standard Pi launch now includes this filter; do not start another copy.
Standalone use requires disabling it in the standard launch first.

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
than reusing a stale plane. Missing input yields no new obstacle/floor cloud. After five seconds without
input the reader is recreated, at most once every five seconds, and status
reports `waiting_for_cloud`. An unexpected
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

## Light spatial-filter trial

Owner confirmed real book detection after the floor retune, but reported little
improvement in speckling from the camera speckle filter alone. Enabled spatial
filtering: alpha 0.7, delta 10, hole-filling radius 0, one iteration. Speckle
filter stays enabled and temporal filtering stays disabled. Floor tuning is
unchanged. Live parameters verified all these settings. The installed driver
uses `i_spatial_filter_iterations`, unlike the `num_iterations` spelling in the
current online parameter documentation:
https://docs.luxonis.com/software-v3/depthai/ros/parameters

After camera restart: 30/30 valid floor estimates, 3.00 Hz status/output processing
rate, latest processing time 154.9 ms, height 0.246 m and tilt 6.64 degrees.
This sample was slower than the previous ~5 Hz; the cause needs profiling before
collision use. Visual noise improvement and physical book retention with spatial
filtering are still unconfirmed. Launch PID 7417; spatial.log records startup.

## Spatial trial reverted

Owner observed improved distant detections but no useful reduction of nearby
speckles, so the lower output rate was not worthwhile. Disabled spatial filtering
again; its tuning values remain inert in the config for reproducibility. Speckle
filtering remains enabled, temporal filtering remains disabled, and floor tuning
is unchanged. Both Pi configuration copies were updated and the package rebuilt.
Camera restarted as PID 7894; log is spatial-disabled.log.

Post-revert live check: spatial parameter false; 49/49 valid floor fits, but
sampled output remained 2.74 Hz (latest processing 114.4 ms). Disabling spatial
filtering did not restore the earlier ~5 Hz, so the slowdown cannot be attributed
to that filter alone. One camera container and one floor filter were running;
CPU snapshot showed camera ~88%, floor filter ~38%, kiosk video ~45% (per-core
percentages), temperature 57.3°C. Throughput needs separate profiling; no other
services or floor parameters were changed.

## Median decimation trial

Enabled NON_ZERO_MEDIAN camera decimation. Factor 2 produced 320x200, matching
the previous cloud size: the driver explicitly applies the factor to its
640x400 input. Set factor 4 to target 160x100, half the previous width and height
and one quarter of the points. Spatial and temporal filters remain off; speckle
filter and floor parameters are unchanged. The floor process needed restarting
after the first camera restart (it was alive but published no status during the
measurement); camera/floor commissioning PIDs are now 8748/8749.

Median decimation may suppress small noise but also removes detail. Repeat the
physical 3 cm book checks centrally, left/right and near the nose before judging
this trial successful. CameraInfo dimensions and floor geometry are checked live.

Live result: 160x100 cloud, 256,000 bytes/message versus 1,024,000 previously;
15.00 Hz cloud reception. CameraInfo is 160x100 with focal lengths 113.1202,
half the previous 226.2405. Floor output 4.54 Hz, 81/91 valid fits; failed fits
passed through without floor removal. Latest valid height 0.2651 m, tilt 5.51°,
support 0.207, processing 62.9 ms. Intermittent floor rejection and physical
book/noise performance remain to be assessed. No floor thresholds were loosened.

## Restore previous resolution

Owner reported periodic horizontal floor lines at 160x100 and only marginal
book visibility. Live diagnostic found 71/93 valid floor fits at 20% support;
18% support increased this to 88/93 but accepted variable height estimates up to
32.8 cm. That lower threshold was not deployed and the floor band was not widened.
At owner's request, disabled the explicit decimation filter to restore the
previous 320x200 output. Speckle stays enabled, spatial/temporal off; floor tuning
unchanged. Both Pi configuration copies updated and package rebuilt. Restarted
camera/floor processes as 9211/9212; log restored-resolution.log.

Verified restored cloud is 320x200; floor fitting succeeded in 56/56 sampled
frames, output 3.16 Hz. Latest height 0.2419 m, tilt 6.87 degrees, processing
75.0 ms. Floor settings unchanged; nearby-speckle filtering remains unresolved.

## Standard system launch integration

`k9_system_pkg` now includes the installed camera and floor-filter launches on
Pi/all, in a scoped group to keep driver launch arguments local. Both enable
switches default true. Six role/switch combinations passed a launch-construction
test; system package builds and --show-args succeeded on Pi and Jetson.
Stopped the old standard Pi session and standalone OAK/floor sessions, then
started a fresh standard Pi launch (PID 10016). One publisher per topic verified.
Pi reception: /scan 9.82 Hz, /oak/points 12.63 Hz (320x200), /oak/obstacles 4.93 Hz;
79/79 valid floor reports. Jetson reception after the fresh launch: /scan 9.88 Hz,
/oak/points 11.38 Hz, /oak/obstacles 4.74 Hz; 79/79 valid floor reports. Every
received cloud/scan had a distinct acquisition timestamp. These are short
subscriber measurements, not guaranteed sustained rates or latency tests.
Camera mount TF and nearby speckles remain outstanding before navigation use.

## Startup input recovery after Pi reboot

After a Pi reboot the camera and 320x200 raw cloud worked at 15 Hz on both hosts,
but the live floor-filter reader received no data and produced no status or
obstacle clouds. Its parameter services worked, use_sim_time was false and
publisher/subscriber QoS were compatible. New diagnostic readers received data;
the underlying DDS/startup cause is not established.

Added a bounded recovery: after 5 seconds without input, recreate the cloud
subscription (at most every 5 seconds), report waiting_for_cloud and count
subscription_reconnects. No stale cloud or stale plane is reused. Floor geometry,
thresholds and camera settings are unchanged. Five geometry tests and a ROS
integration test passed; the latter checks absent input, reader recreation,
resumption on synthetic data and no replay while input is missing.

Fresh standard launch PID 3500 reconnected once during camera startup. Jetson
received /scan at 9.84 Hz, /oak/points at 13.87 Hz and /oak/obstacles at 2.76 Hz
(44 clouds with distinct acquisition stamps). 50/51 sampled status reports were
valid; the waiting status is included in that denominator. Throughput still
varies with load; this is recovery of input delivery, not a performance fix.
