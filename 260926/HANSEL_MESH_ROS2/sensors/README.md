# Sensors

This directory corresponds to the upstream `sensors/` role.

The TI Radar UART must be owned by one capture process. The upstream capture
process writes mission JSONL records. The ROS adapter tails that file and
publishes PointCloud2 and OccupancyGrid topics; it does not open the UART a
second time.

Use `run_radar_adapter.sh <mission.jsonl>` after the capture process is running.
