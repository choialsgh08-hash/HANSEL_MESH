# Directory layout and upstream mapping

## Why `ros2_ws/` is separated

The original HANSEL_MESH repository places operational programs at the root:
`controller/`, `robot/`, `monitor/`, `sensors/`, `configs/`, `scripts/` and
`services/`. ROS 2, however, expects packages below a colcon workspace `src/`
directory. This repository therefore uses both conventions:

```text
root operational directories  -> deployment and non-ROS entry points
ros2_ws/src/                   -> ROS 2 packages
```

## Path mapping

| Upstream-style role | This repository | ROS implementation |
|---|---|---|
| Keyboard controller | `controller/` | `ros2_ws/src/hansel_operator` |
| Robot motor server | `robot/` | `ros2_ws/src/hansel_unit_control` |
| Mesh metrics | `monitor/metrics_agent.py` | `ros2_ws/src/hansel_network_adapter` |
| Radar capture bridge | `sensors/` | `ros2_ws/src/hansel_radar_adapter` |
| Role network config | `configs/*.env` | independent of ROS parameters |
| Mesh setup | `scripts/` | system-level BATMAN-adv setup |
| Autostart | `services/` | systemd templates |
| Shared messages | `common/` | `ros2_ws/src/hansel_interfaces` |

## Device deployment

### Operator laptop

Use:

```text
controller/
listen/
ros2_ws/src/hansel_interfaces
ros2_ws/src/hansel_operator
ros2_ws/src/hansel_network_adapter
ros2_ws/src/hansel_camera_bridge
ros2_ws/src/hansel_bringup
```

For development, copy and build the whole repository instead of selecting
individual packages.

### Base Raspberry Pi

Use:

```text
configs/base.env
scripts/
services/
monitor/metrics_agent.py
```

The Base is a Mesh gateway/relay and metrics source. It does not run a drive
`unit_controller` in the current chain definition.

### Head Raspberry Pi

Use the whole repository and run:

```text
robot/run_unit.sh
monitor/metrics_agent.py
sensors/run_radar_adapter.sh
```

The original Radar capture process owns the Radar UART and writes mission
JSONL. The ROS Radar adapter only reads that log.

### Rear Node Raspberry Pi

Use the whole repository and run:

```text
robot/run_unit.sh
monitor/metrics_agent.py
```

### Arduino Nano

Only flash:

```text
robot/firmware/hansel_nano_bridge/hansel_nano_bridge.ino
```
