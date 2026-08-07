# Robot

This directory corresponds to the upstream `robot/` role.

The original motor-server responsibilities are split into:

```text
ros2_ws/src/hansel_unit_control/hansel_unit_control/core.py
  semantic command -> CPS targets, PID, feed-forward, ramp and safety

ros2_ws/src/hansel_unit_control/hansel_unit_control/hardware.py
  ROS controller <-> Arduino Nano serial protocol

ros2_ws/src/hansel_unit_control/hansel_unit_control/unit_controller.py
  ROS topics/services, control timer and state publication
```

The hardware firmware is under `robot/firmware/`.
