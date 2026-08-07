# HANSEL single-PC validation

## What this validates

- keyboard-only operation through `operator_input`
- Head/rear command routing
- four editable RQT values only: Straight RPM, Turn RPM, Up limit, Down limit
- Head logical range up to `-180 deg .. +180 deg`
- RQT video panel using a built-in JPEG test pattern
- dummy motor, PID, state, detach and safety paths

It does not validate Nano wiring, motor direction, encoder polarity, physical servo travel, BATMAN-adv radio quality, or a real camera stream.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/HANSEL_MESH_ROS2/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

## Recommended one-command test

Run in a real interactive terminal:

```bash
cd ~/HANSEL_MESH_ROS2
./controller/run_single_pc_test.sh
```

The script starts the dummy graph and RQT in the background, then keeps
`operator_input` in the foreground so single-key input reaches the node.
Do not run the keyboard process inside `ros2 launch`; launch-managed processes
usually do not own the terminal TTY.

Expected RQT contents:

- Camera video test pattern
- Straight RPM
- Turn RPM
- Head Up limit: `0..180`
- Head Down limit: `0..180` entered as a positive magnitude
- read-only unit, network and diagnostic status

For `Up=180`, `Down=180`, the logical range is exactly `-180..+180 deg`.
`U` and `J` move by the YAML-fixed 2-degree step; `K` returns to 0 degrees.

## Manual multi-terminal method

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/HANSEL_MESH_ROS2/ros2_ws/install/setup.bash
ros2 launch hansel_bringup dummy_system.launch.py \
  use_rviz:=false use_rqt:=true \
  use_camera_receiver:=false use_dummy_camera:=true \
  use_adapter_stubs:=false
```

Terminal 2 (must be interactive and focused):

```bash
cd ~/HANSEL_MESH_ROS2
./controller/run_keyboard_control.sh
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/HANSEL_MESH_ROS2/ros2_ws/install/setup.bash
ros2 topic echo /hansel/head/state/wheels
```

Terminal 4:

```bash
source /opt/ros/jazzy/setup.bash
source ~/HANSEL_MESH_ROS2/ros2_ws/install/setup.bash
ros2 topic echo /hansel/head/state/front_angle
```

## Key checks

- `W`: Head and rear units move forward
- `A/D`: Head spins; rear units stop
- `Q/E`: Head curves; rear units use slow straight motion
- `X` or Space: stop
- `U/J`: Head logical angle changes by 2 degrees
- `K`: Head returns to 0 degrees
- `Up=180`, `Down=180`: repeated U/J clamps at +180/-180

## Common failure

When `operator_input` prints `interactive POSIX terminal required`, it has no
TTY. Start it from a normal terminal with `run_keyboard_control.sh` or use
`run_single_pc_test.sh`; do not start it as a background launch node.
