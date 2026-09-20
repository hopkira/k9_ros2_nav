# K9 LD06 commissioning — 2026-09-20

Installed on `hopkira@k9-ros2-ubuntu.local` in `/home/hopkira/k9_ws`.
Ubuntu 24.04 / ROS 2 Jazzy. USB serial `/dev/lidar360`, 230400 baud.

## Run on the Pi

```bash
source /opt/ros/jazzy/setup.bash
source ~/k9_ws/install/local_setup.bash
ros2 launch ~/k9_ws/ld06_bringup/ld06.launch.py
```

Stop with Ctrl+C. This is a manual commissioning launch, not a boot service.
It starts only the lidar component and its lifecycle manager, not robot motors.
Raw LaserScan output: `/scan_raw`, frame `base_laser`, 455 fixed bins.
The full robot description must supply the mounting TF. For standalone RViz
inspection use `base_laser` as the fixed frame. No standalone TF publisher is
started, avoiding conflicts with the eventual shared robot description.

## Reproducibility

Upstream: https://github.com/Myzhar/ldrobot-lidar-ros2
Commit: `4ee53a8b176037cf418a54b25007075bb2b1d3a0`
SDK submodule: `09d1003efe0ff5b6095a6f250ccf82d87a1a41cf`

Local fixes (saved as patches alongside this document):
- `driver-fixes.patch`: install SDK shared library; count subscribers on the
  actual publisher so topic remapping does not suppress output.
- `sdk-shutdown.patch`: join serial reader before closing its file descriptor.
  Apply this patch inside the SDK submodule, not the parent repository.

```bash
cd ~/k9_ws/src/ldrobot-lidar-ros2
git apply ~/k9_ws/ld06_bringup/driver-fixes.patch
git -C ldlidar_component/ldlidar_stl_sdk apply ~/k9_ws/ld06_bringup/sdk-shutdown.patch
cd ~/k9_ws
colcon build --packages-up-to ldlidar_node --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

Patches are already applied on the Pi; do not reapply there. Runtime dependencies
were present. BUILD_TESTING=OFF avoids an absent upstream clang-format test
package; no system apt changes were required.

## Verified

- Hardware communication and automatic lifecycle activation.
- Final Pi capture: 120 messages, 9.9635 Hz, consistently 455 bins, monotonically
  increasing timestamps and at least 397 finite returns per scan.
- Jetson capture over ROS 2: 119 messages at 9.9411 Hz.
- Final bounded run stopped cleanly with SIGINT, without forced termination.
- Pi results and launch log are in `~/k9_ws/ld06_bringup/`.

## Still to commission

Raw readings include values below the advertised 0.03 m range minimum. Filter
out-of-range returns before navigation; do not reinterpret rejected returns as
clear space. Physical orientation, scan ordering/timing semantics, true distance
accuracy, mounting TF and the head-obstructed sector still need validation.
No head mask, SLAM, Nav2, motor motion or automatic startup has been enabled.
Transport timestamp checks do not establish sensor acquisition-time accuracy.
The checker is a transport sanity check, not a navigation acceptance test.
