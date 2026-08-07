# Configs

Root-level `.env` files configure the Linux/BATMAN-adv runtime and follow the
original HANSEL_MESH layout.

```text
base.env, head.env, node1.env, node2.env, node3.env
```

ROS parameters remain in the ROS package so they are installed by colcon:

```text
ros2_ws/src/hansel_bringup/config/
```

Use `.env` for network interfaces/IPs and YAML for ROS control parameters.
