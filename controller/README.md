# Controller

This directory corresponds to the upstream `controller/` role.

- `run_operator_stack.sh`: starts routing, detach coordination, logging,
  network adapter and optional RQT.
- `run_keyboard_control.sh`: starts the keyboard-first controller.

The actual ROS nodes are implemented in:

```text
ros2_ws/src/hansel_operator/
```

Keyboard commands retain the upstream semantic command model: W/S/A/D/Q/E/Z/C
select a persistent motion command, X or Space stops, and U/J/K are one-shot
Head servo commands.
