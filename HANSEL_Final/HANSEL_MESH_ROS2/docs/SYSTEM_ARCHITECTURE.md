# ROS 2 system architecture

```mermaid
flowchart LR
  KB[operator_input\nreal keyboard TTY]
  RQT[RQT\nparameters + status only]
  ROUTER[command_router]
  DETACH[detach_coordinator]
  NET[network_adapter\nmetrics_agent UDP JSON]
  RADAR[radar_adapter\nmission JSONL tail]

  KB -->|MotionCommand| ROUTER
  KB -->|U/J/K HeadServoCommand| HEAD
  KB -->|detach service| DETACH
  RQT -->|ROS parameter services| HEAD
  RQT -->|ROS parameter services| NODES

  ROUTER -->|original semantic command| HEAD[Head unit_controller]
  ROUTER -->|normalized rear command| NODES[Rear unit_controllers]
  DETACH -->|safe-stop + trigger services| HEAD
  DETACH -->|safe-stop + trigger services| NODES

  HEAD -->|USB serial| NANO[Arduino Nano firmware]
  NODES -->|USB serial| NANOS[Arduino Nano firmware per unit]
  NANO --> MOTORS[shared D5/D6 PWM\nindependent direction pins]
  NANO --> ENC[D2/D3 rear encoder interrupts]
  NANO --> SERVO[D9 Head servo / D10 detach]

  METRICS[HANSEL_MESH metrics_agent] -->|UDP JSON| NET
  CAPTURE[HANSEL_MESH radar_capture] -->|mission.jsonl| RADAR
  NET --> RQT
  RADAR -->|PointCloud2 / OccupancyGrid| RVIZ[RViz]
```

## Package responsibilities

- `hansel_operator`: keyboard input, semantic routing, detach coordination and
  parameter-only RQT.
- `hansel_unit_control`: semantic-to-CPS mapping, safety/watchdog,
  feed-forward/PID/ramp and Dummy/Nano hardware adapters.
- `hansel_network_adapter`: original metrics-agent UDP JSON to ROS status and
  optional detach recommendation.
- `hansel_radar_adapter`: original mission-log `radar_frame` to ROS cloud/map.
- `hansel_bringup`: single-PC dummy launch, operator launch, per-unit launch and
  adapter launch.
- `firmware`: physical Nano pin control and serial watchdog.
