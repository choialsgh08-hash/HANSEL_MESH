# Validation status

Performed in the artifact environment:

- Python compileall for all ROS Python packages, tests and tools
- XML and YAML parsing
- unit/contract tests
- ROS-independent semantic steering + PID/ramp + Head step smoke test
- ZIP integrity check

Not available in the artifact environment:

- ROS 2 `colcon build` and live ROS graph
- RQT/RViz rendering
- Arduino compilation/upload
- real encoder, H-bridge, servo, BATMAN-adv and TI mmWave hardware

Hardware validation must begin with wheels raised, conservative Head pulse
limits and a common ground check.
