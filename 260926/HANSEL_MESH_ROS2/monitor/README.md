# Monitor

`metrics_agent.py` collects `iw`, `batctl`, `ping`, neighbor and `bat0` traffic
statistics, then sends the HANSEL_MESH-compatible UDP JSON contract to the
operator laptop. The ROS `hansel_network_adapter` receives these packets.
