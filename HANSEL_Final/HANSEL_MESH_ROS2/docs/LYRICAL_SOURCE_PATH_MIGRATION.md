# ROS 2 Lyrical source-build path migration

This repository is configured for a source-built ROS 2 Lyrical installation.

Default ROS base:

```text
~/ros2_lyrical/install/local_setup.bash
```

Runtime/build wrappers source `scripts/ros_env.sh` instead of a system-installed ROS setup path.
The loader:

- resolves `~/ros2_lyrical` automatically;
- allows an override with `ROS2_LYRICAL_ROOT`;
- defaults `RMW_IMPLEMENTATION` to `rmw_cyclonedds_cpp`;
- sources `ros2_ws/install/local_setup.bash` for runtime wrappers;
- automatically sets `CYCLONEDDS_URI` to the repository `bat0` configuration when `bat0` exists.

Example override:

```bash
export ROS2_LYRICAL_ROOT=/home/hansel/ros2_lyrical
```

The mesh systemd templates use `@HANSEL_REPO_DIR@`; `scripts/enable_mesh_autostart.sh`
replaces it with the repository's actual path at installation time.
