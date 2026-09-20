# K9 ROS 2 navigation

Navigation bring-up and commissioning for K9: Ubuntu 24.04, ROS 2 Jazzy,
Raspberry Pi 5 and Jetson Orin NX.

## Current status

The LD06 driver has been built and tested on the Pi. Raw scans reach both the
Pi and Jetson at approximately 10 Hz, with 455 fixed angular bins. The driver
shuts down cleanly after the local fixes recorded here.

This repository currently contains commissioning files, not an installable
ROS package. SLAM, autonomous navigation and collision avoidance are planned
but have not been commissioned by this work.

- [LD06 launch, configuration and reproduction instructions](commissioning/ld06_bringup/README.md)
- [Architecture and sensor geometry](docs/ARCHITECTURE.md)
- [Progress and next steps](docs/PROGRESS.md)

## Hardware

| Host | Role | Workspace |
| --- | --- | --- |
| `k9-ros2-ubuntu.local` | Pi 5: sensors, odometry, drive and local collision monitoring | `/home/hopkira/k9_ws` |
| `k9-ros2-jetson.local` | Orin NX: SLAM, Nav2 and heavier perception | `/home/hopkira/k9_ws` |

LD06 USB serial device: `/dev/lidar360` (existing udev rule).

## Run the LD06 on the Pi

With the patched driver built and this repository cloned locally:

```bash
source /opt/ros/jazzy/setup.bash
source ~/k9_ws/install/local_setup.bash
ros2 launch /path/to/k9_ros2_nav/commissioning/ld06_bringup/ld06.launch.py
```

Outputs: `/scan_raw` and `/scan` (`sensor_msgs/msg/LaserScan`), frame `base_laser`.
The filtered `/scan` replaces returns below 0.20 m with NaN, preserving unknown
space behind close self-returns.
Stop with Ctrl+C. No boot service is installed. For initial RViz inspection,
use `base_laser` as the fixed frame; full robot TF integration is still pending.

## Related repositories

- [K9 Gazebo model](https://github.com/hopkira/k9-gazebo): robot geometry and simulation.
- [K9 drive package](https://github.com/hopkira/k9_driver_pkg): real drive integration.
- [Upstream LD lidar driver](https://github.com/Myzhar/ldrobot-lidar-ros2): pinned revision and fixes are documented in the commissioning folder.

The robot description remains owned by the Gazebo/description repository;
this repository documents the intended shared frames rather than duplicating it.
