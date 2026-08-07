# HANSEL_MESH ROS 2 integration

This repository keeps the operational directory layout close to the original
`HANSEL_MESH` project while isolating the ROS 2 packages in `ros2_ws/`.

## Directory layout

```text
HANSEL_MESH_ROS2/
├── common/       shared contracts and architecture notes
├── configs/      role-specific BATMAN-adv and metrics environment files
├── controller/   operator laptop entry points and keyboard control
├── docs/         deployment, wiring, validation and architecture documents
├── listen/       standalone inspection/listener utilities
├── monitor/      HANSEL_MESH-compatible metrics_agent.py
├── robot/        unit-side launch helpers and Arduino Nano firmware
├── scripts/      BATMAN-adv setup, installation and project helper scripts
├── sensors/      Radar adapter entry points and sensor integration notes
├── services/     systemd service templates
├── tests/        repository-level Python contract and control tests
├── tools/        local smoke-test utilities
└── ros2_ws/
    └── src/      ROS 2 packages
```

The upstream repository keeps operational code at the root in directories such
as `controller/`, `robot/`, `monitor/`, `sensors/`, `configs/`, `scripts/` and
`services/`. This project follows the same separation, while ROS-specific
packages remain under the standard colcon workspace path `ros2_ws/src/`.

## Build the ROS 2 workspace

```bash
cd ~/HANSEL_MESH_ROS2
./scripts/build_ros2.sh
source ros2_ws/install/setup.bash
```

Manual equivalent:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/HANSEL_MESH_ROS2/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## One-PC test

```bash
cd ~/HANSEL_MESH_ROS2
./scripts/test_workspace.sh

source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hansel_bringup dummy_system.launch.py \
  use_rviz:=false use_rqt:=true \
  use_camera_receiver:=false use_adapter_stubs:=false
```

In another terminal:

```bash
cd ~/HANSEL_MESH_ROS2
./controller/run_keyboard_control.sh
```

## Raspberry Pi network setup

The network setup now lives at the same root-level paths used by the upstream
project:

```bash
cd ~/HANSEL_MESH_ROS2
sudo ./scripts/install_network_bundle.sh head
```

Replace `head` with `base`, `node1`, `node2` or `node3`. The installer copies
`configs/`, `scripts/`, `services/` and `monitor/` to the runtime directory and
enables the role-specific systemd services.

## Unit execution

```bash
cd ~/HANSEL_MESH_ROS2
./robot/run_unit.sh head /dev/hansel_nano
```

For a rear unit:

```bash
./robot/run_unit.sh node1 /dev/hansel_nano
```

## Hardware firmware

Flash this file to every Arduino Nano:

```text
robot/firmware/hansel_nano_bridge/hansel_nano_bridge.ino
```

Read `docs/WIRING_AND_NANO.md` before powering the motors or expanding the
servo range.

## Responsibility map

- `controller/`: keyboard-first operator input and operator stack wrappers.
- `robot/`: unit controller startup and Nano firmware.
- `monitor/`: BATMAN-adv metrics collection and UDP reporting.
- `sensors/`: mission-log-based Radar integration.
- `ros2_ws/src/hansel_operator`: ROS routing, detach coordination and RQT.
- `ros2_ws/src/hansel_unit_control`: CPS target mapping, PID, safety and serial hardware bridge.
- `ros2_ws/src/hansel_network_adapter`: UDP metrics to ROS topics.
- `ros2_ws/src/hansel_radar_adapter`: mission JSONL to PointCloud2/OccupancyGrid.

See `docs/DIRECTORY_LAYOUT.md` for a detailed file-to-device deployment map.

## Correct single-PC keyboard test

After building `ros2_ws`, run:

```bash
./controller/run_single_pc_test.sh
```

This preserves an interactive terminal for keyboard input and also publishes a
dummy camera image to the RQT video panel. RQT has only four editable values:
Straight RPM, Turn RPM, Head Up limit and Head Down limit. Up/Down can each be
set to 180 degrees, yielding a logical servo range of -180 through +180 degrees.
