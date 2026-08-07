# Common contracts

The original HANSEL_MESH project shares command and sensor contracts through
its common code. In this ROS 2 integration, the authoritative shared contracts
are generated from:

```text
ros2_ws/src/hansel_interfaces/msg/
ros2_ws/src/hansel_interfaces/srv/
```

Important messages include semantic motion commands, Head servo commands,
wheel/unit states, active-chain state and network/Radar status. All devices must
build the same revision of `hansel_interfaces` to avoid DDS type mismatches.
