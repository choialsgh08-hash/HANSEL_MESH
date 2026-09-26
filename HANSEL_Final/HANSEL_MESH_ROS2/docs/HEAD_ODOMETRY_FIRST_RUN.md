# Head odometry first run without measured CPR

This first version deliberately uses the existing `encoder_cpr: 400.0` setting.
It is suitable for checking data flow, heading direction, trajectory shape and
relay marker behavior. It is not a 1 cm-accuracy claim.

## Data flow

```text
rear left/right encoder
  -> Arduino Nano signed CPS
  -> /hansel/head/state/wheels
  -> nominal wheel distance

BNO085 Game Rotation Vector
  -> startup-relative yaw
  -> /hansel/head/imu/data

wheel distance + IMU yaw
  -> /hansel/head/odometry
  -> /hansel/head/path

/hansel/system/state/active_chain
  -> freeze node1/node2 position along the recorded Head path
  -> /hansel/localization/relay_pose
  -> /hansel/localization/relay_markers
```

Only the rear encoders are used. The front wheel encoders are intentionally
ignored. Track width is not required in this version because BNO085 supplies
heading; the encoders supply forward distance.

The sensor is mounted with BNO085 `+Y` toward the robot front and `+Z` upward.
Output follows the ROS convention: `base_link +X` is forward, `+Y` is left and
`+Z` is up. Heading is zeroed at startup, so Head must remain stationary for the
first second.

## Initial parameters

File: `ros2_ws/src/hansel_bringup/config/head_odometry.yaml`

```yaml
wheel_diameter_m: 0.08
encoder_cpr: 400.0
left_encoder_sign: 1.0
right_encoder_sign: -1.0
left_distance_scale: 1.0
right_distance_scale: 1.0
relay_offsets:
  - node1=0.20
  - node2=0.40
provisional_calibration: true
```

The right sign does not change motor direction or Arduino firmware. It only
normalizes the previously observed forward telemetry (`left > 0`, `right < 0`)
before calculating distance.

## Head installation

Wire the BNO085 to the Raspberry Pi 4:

```text
BNO085 VIN -> Pi physical pin 1 (3.3 V)
BNO085 GND -> Pi physical pin 6
BNO085 SDA -> Pi physical pin 3 (GPIO2)
BNO085 SCL -> Pi physical pin 5 (GPIO3)
```

Enable and inspect I2C:

```bash
sudo raspi-config nonint do_i2c 0
sudo apt install -y i2c-tools python3-pip
sudo i2cdetect -y 1
```

The sensor normally appears as `4a` or `4b`. Install the Python driver into the
repository-local vendor directory instead of modifying the OS Python packages:

```bash
cd /home/hansel/HANSEL_MESH_ROS2
./scripts/install_head_odometry_deps.sh
```

For Raspberry Pi stability, set the I2C bus to 400 kHz once and reboot:

```bash
grep -q '^dtparam=i2c_arm_baudrate=400000$' /boot/firmware/config.txt || \
  echo 'dtparam=i2c_arm_baudrate=400000' | \
  sudo tee -a /boot/firmware/config.txt
sudo reboot
```

## Build

```bash
cd /home/hansel/HANSEL_MESH_ROS2
source /home/hansel/ros2_lyrical/install/setup.bash
cd ros2_ws
colcon build --symlink-install \
  --packages-up-to hansel_localization hansel_bringup
source install/setup.bash
```

## Run with the drive stack

Keep the existing unit controller, operator launch and keyboard input running.
Open one additional Head terminal for odometry:

```bash
cd /home/hansel/HANSEL_MESH_ROS2
./robot/run_head_odometry.sh
```

When running with a local monitor and desktop session, add the dedicated
top-down RViz view:

```bash
./robot/run_head_odometry.sh use_rviz:=true
```

Do not move the robot until this appears:

```text
BNO085 zero heading ready; ROS odom +X is robot forward
```

Check the streams from another ROS terminal:

```bash
source /home/hansel/ros2_lyrical/install/setup.bash
source /home/hansel/HANSEL_MESH_ROS2/ros2_ws/install/setup.bash

ros2 topic hz /hansel/head/imu/data
ros2 topic hz /hansel/head/odometry
ros2 topic echo /hansel/head/odometry --once
ros2 topic echo /hansel/localization/relay_pose
```

To reset the origin, stop the robot and call:

```bash
ros2 service call /hansel/head/localization/zero std_srvs/srv/Trigger '{}'
```

Keep Head stationary until zero-heading collection finishes again.

## Direction validation

Perform this test with the wheels lifted and low speed.

1. Press `W`. `/hansel/head/odometry` linear X must increase.
2. Turn Head physically left. Yaw must increase.
3. Turn Head physically right. Yaw must decrease.

If yaw is reversed, set `imu_yaw_sign: -1.0`. If forward odometry stays near
zero, inspect `/hansel/head/state/wheels`. Use opposite left/right encoder signs
only in the odometry YAML; do not change motor direction merely to fix pose.

## Distance correction without CPR measurement

After the direction test, mark a straight 2 m course, reset localization and
drive to the end. Let `E` be the X distance reported by odometry and `D=2.0 m`
the measured distance:

```text
new_distance_scale = old_distance_scale * D / E
```

Example: odometry reports 1.84 m after an actual 2.00 m run:

```text
new_distance_scale = 1.0 * 2.00 / 1.84 = 1.087
```

Set both `left_distance_scale` and `right_distance_scale` to that value and
repeat. This directly corrects the combined nominal CPR/wheel-diameter scale.
It does not remove floor slip, tire compression or BNO085 yaw drift.

Keep `provisional_calibration: true` during these tests. Change it to `false`
only after repeated straight, curved and 5 m validation runs are within the
project's accepted error.

## Relay position behavior

When the detach coordinator adds a unit to `relay_assumed_units`, the odometry
node freezes its estimated location on the previously recorded Head trajectory:

- node1: 0.20 m behind Head;
- node2: 0.40 m behind Head;
- node3: 0.60 m behind Head, reserved for future use.

The marker is a software estimate of the commanded separation location. It does
not prove that physical separation succeeded. With assumed CPR and no external
absolute reference, 1 cm accuracy over 5 m is not yet realistic; radar matching,
fiducials or another absolute correction source will be needed for that target.
