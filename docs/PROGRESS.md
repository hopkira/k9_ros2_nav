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

Automated tests establish hardware communication and ROS transport. The owner
subsequently confirmed physical scan orientation. Distance accuracy, per-ray
timing and navigation readiness remain unverified. Driver timestamps
have not been verified against acquisition time. Upstream lint tests were not
run because their clang-format dependency was absent; live tests were performed.

## Known nearby robot returns

Owner reports buttons directly ahead of the LD06, approximately 2.5 cm away.
These plausibly explain the observed 2–3 cm raw ranges, including values below
the configured 3 cm minimum. Keep raw observations for inspection. Exclude
out-of-range and known self-returns from navigation; masked directions are
occluded, not confirmed clear behind the buttons/head.

## Next steps

1. Check range accuracy against measured target distances in an unobstructed sector.
2. Connect the sensor to the shared robot TF and check the real mounting pose.
3. Determine button/head angular masks and publish a filtered navigation scan.
4. Validate wheel odometry and single TF ownership.
5. Bring up SLAM Toolbox on Jetson and map a manually driven loop.
6. Commission OAK-D depth processing and Pi Collision Monitor, including stale
   sensor and lost-command/network behaviour, before autonomous motion.
7. Add the ear serial bridge and angle-aware floor filtering.

No automatic startup, SLAM, Nav2 or robot movement was enabled during the LD06
bring-up. The Pi commissioning files remain at `~/k9_ws/ld06_bringup`.
