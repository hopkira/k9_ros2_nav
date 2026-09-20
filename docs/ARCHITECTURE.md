# Agreed navigation architecture

## Processing allocation (planned)

Pi 5: motor control/watchdog, wheel odometry, optional wheel/IMU EKF, LD06 driver,
OAK-D driver and reduced XYZ obstacle cloud, ear serial bridge, command
arbitration and Nav2 Collision Monitor. Sensor-to-stop processing remains local.

Orin NX: SLAM Toolbox, Nav2 costmaps/planning/controller/behaviour tree and
velocity smoothing, plus heavier vision workloads. Final velocity commands
return to the Pi through Collision Monitor. Command expiry and drive watchdogs
must stop motion if the Jetson or network disappears.

LD06 is the primary 2D mapping sensor. OAK-D supplies forward depth coverage,
especially in the head-obstructed laser sector; ears supplement front/side
coverage. Initial mapping does not require RGB-D SLAM.

TF ownership: SLAM/localisation owns `map -> odom`; wheel odometry or EKF owns
`odom -> base_link`; the robot description and joint states own sensor frames.
Only one publisher should own each transform.

## Sensor geometry and outstanding discrepancies

The active Gazebo source is
`src/k9_robot/src/description/k9.urdf.xacro` in `k9-gazebo`.
`base_link` is 0.0694 m above the nominal floor.

- LD06 `base_laser`: (-0.112, 0, 0.53) m relative to `base_link`, horizontal.
  Owner reports about 30 degrees forward occluded by the head. The angular
  mask must be measured, not assumed from this approximate description.
- OAK-D `oakd_link`: (0.26, 0, 0.255) m, level. This implies 0.3244 m above
  ground in the model, versus the owner's intended 0.26 m. Not yet corrected.
  Nose projects approximately 0.30 m ahead of the physical camera.
- Ear pivots: (0.27, +/-0.0425, 0.67) m relative to `base_link`.
  Confirmed servo axes tilt 7 degrees away from vertical. Each sensor has a
  further 30-degree downward mounting angle. Neutral forward beam pitch is
  therefore 37 degrees down. Local Gazebo model was corrected accordingly;
  that repository's changes are separate from this navigation commit.
- New sensor frames: `l_ear_sensor_link`, `r_ear_sensor_link`, with the existing
  7.5 mm local forward offset from each rotating ear and 30-degree pitch.

## Ear floor filtering

For servo angle q (zero forward), downward unit-ray component is:

```
d(q) = sin(7 deg)*cos(30 deg)*cos(q) + cos(7 deg)*sin(30 deg)
r_floor = h / d(q)
z_return = h - measured_range*d(q)
```

These equations assume a level base and floor, using the sensor origin height
h. In implementation, transform each reading's endpoint using its timestamp,
measured joint angle and actual sensor TF; account for chassis attitude and
floor plane. A positive height threshold identifies potential obstacles.
Missing readings are unknown; a long return alone does not establish a drop-off.

Keep the tested Espruino firmware for smooth servo motion and VL53L0X ranging.
The proposed Pi bridge consumes newline JSON such as:

```json
{"type":"LIDAR","sensor":"l_ear","distance":1.23,"angle":0.42}
```

Distance is metres; angle is radians from exposed potentiometer feedback.
Publish measured joint states, Range messages and filtered obstacle points.
Firmware currently samples the potentiometer before a single range measurement,
provides no acquisition timestamp and suppresses invalid readings. Receipt-time
stamping is an initial approximation, not precise acquisition synchronisation.
Reported firmware ranges are left 0..+45 degrees and right 0..-45 degrees;
calibration and Gazebo servo limits still need reconciliation. Position telemetry
is absent when no valid range messages are transmitted. Existing Pi command
transport also needs inspection before replacing it.
