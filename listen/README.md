# Legacy monitoring tools

This directory contains the original CSV/RSSI monitoring workflow.  It is kept
for old field-test data and is not the active dashboard stack.

For new deployments use:

- `monitor/metrics_agent.py` on each Pi
- `monitor/dashboard.py` on the operator PC
- `monitor/video_probe.py` for video quality

The legacy programs remain importable and can be run from the repository root:

```bash
python3 listen/monitor_node.py --help
python3 listen/monitor_laptop.py --help
python3 listen/h264_decode_fps.py --help
```
