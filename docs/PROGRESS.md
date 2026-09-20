# Commissioning progress — 2026-09-20

## Completed

- SSH access established to Pi and Jetson as `hopkira`.
- Confirmed Jazzy on both; Pi user has dialout access and `/dev/lidar360` resolves
  to `/dev/ttyUSB0`.
- Cloned Myzhar LD lidar driver and its SDK, built release on Pi.
- Created separate K9 configuration and manual lifecycle-managed launch.
- Fixed upstream missing SDK library installation, remapped-topic subscriber
  detection, and serial reader shutdown ordering. Reproducible patches included.
- Pi final capture: 120 messages at 9.9635 Hz, 455 bins, increasing timestamps,
  minimum 397 finite returns per scan.
- Jetson independently received 119 messages at 9.9411 Hz via ROS 2.
- Final SIGINT shutdown was clean; serial port released.
- Owner confirmed physical scan orientation in RViz: +X forward, +Y left,
  +Z up, with target returns agreeing with the left/right directions. This
  validates scan orientation, not the sensor mounting transform in robot TF.
- Owner confirmed measured distances agree with the 1 m RViz grid squares.
  This is an approximate range-scale check, not a quantified accuracy calibration.

Automated tests establish hardware communication and ROS transport. The owner
subsequently confirmed physical scan orientation and approximate range scale.
Quantified distance accuracy, per-ray timing and navigation readiness remain
unverified. Driver timestamps
have not been verified against acquisition time. Upstream lint tests were not
run because their clang-format dependency was absent; live tests were performed.

## Known nearby robot returns

Owner reports buttons directly ahead of the LD06, approximately 2.5 cm away.
These plausibly explain the observed 2–3 cm raw ranges, including values below
the configured 3 cm minimum. Keep raw observations for inspection. Exclude
out-of-range and known self-returns from navigation; masked directions are
occluded, not confirmed clear behind the buttons/head.

## Next steps

1. Identify button/head self-return angles in the raw scan; approximate range scale is confirmed.
2. Connect the sensor to the shared robot TF and check the real mounting pose.
3. Determine button/head angular masks and publish a filtered navigation scan.
4. Validate wheel odometry and single TF ownership.
5. Bring up SLAM Toolbox on Jetson and map a manually driven loop.
6. Commission OAK-D depth processing and Pi Collision Monitor, including stale
   sensor and lost-command/network behaviour, before autonomous motion.
7. Add the ear serial bridge and angle-aware floor filtering.

No automatic startup, SLAM, Nav2 or robot movement was enabled during the LD06
bring-up. The Pi commissioning files remain at `~/k9_ws/ld06_bringup`.

## Self-return filter refinement

Owner identified close returns as buttons and sloping back panel, all within
10 cm, then requested a 12 cm threshold based on tape measurement. Implemented
configurable minimum-range filter: `/scan_raw` remains unchanged; `/scan` masks
ranges below 0.12 m as NaN. Two regression tests cover boundary behaviour,
invalid values, sensor minimum, unchanged raw input and retained metadata.
Visual confirmation of the chosen threshold is pending.

## Cutoff raised to 20 cm

An 80-scan live sample with the 12 cm filter showed 7–21 remaining close
returns per scan, ranging from 12 to 16.1 cm, primarily 30–39 degrees left of
forward. No readings below 12 cm survived. The owner selected a 20 cm cutoff,
noting the robot's 50 cm base dimension. The default is now 0.20 m; raw scans
remain unchanged and masked space remains unknown. This does not independently
validate full-footprint collision coverage.

Owner confirmed the 20 cm filtered scan is consistently clean in RViz.

## Shared system bring-up and TF

Added robot-description and LD06 launches to `k9_system_pkg/k9.launch.py`,
enabled by default only on Pi/all. Explicit enable flags allow commissioning
instances to remain separate. Both machines rebuilt successfully.
The shared current Gazebo description was deployed on the Pi and a standalone
robot_state_publisher was started (launch PID 9240) without other hardware nodes.
Live TF verified base_link -> base_laser (-0.112, 0, 0.53) m, zero rotation;
nominal floor height 0.5994 m. Stop the standalone publisher before full startup.
Pi's and Jetson's existing launch customisations were preserved when adding the
new blocks; their complete launch files differ from the Mac checkout.
The Pi description deployment is a snapshot of the current local Gazebo source,
including uncommitted model corrections; it is not a clean upstream Git clone.

## OAK-D Lite initial bring-up

Driver installed and USB access confirmed. Depth-only stream measured at 15 Hz,
320x200, 16UC1. No IMU available; no neural networks enabled. Booted USB link is
HIGH (USB 2). Added standalone depth/XYZ cloud launch for RViz commissioning;
cloud reception measured about 6.15 Hz in the initial 60-frame sample. See
`commissioning/oak_bringup/README.md` for frames, launch and outstanding checks.
Not yet integrated into standard system launch or collision avoidance.

## Adaptive OAK floor filter

Added separate `/oak/obstacles` and optional `/oak/floor` outputs, keeping the
raw cloud intact. Constrained per-cloud plane fitting supports the temporary
mount; no fixed TF changes. A +/-2.5 cm floor band and bounded forward region
are documented in the camera README, including small-object limitations.

Three synthetic tests passed on the Pi. Live sample: 26/26 valid floor fits,
4.995 Hz obstacle clouds; latest report height 0.2372 m, tilt 3.16 degrees,
15,876 points removed, 36,023 retained, processing 28.2 ms. Values vary with
scene/noise. Navigation package rebuilt successfully; standalone filter PID
5596 remains active for RViz. Real obstacle retention awaits user inspection.
