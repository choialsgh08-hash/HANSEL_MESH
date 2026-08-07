# HANSEL_MESH network and Radar integration

## 1. Bundled network layer

The network bundle now follows the original repository layout directly at the project root:

```text
configs/                  # base/head/node1/node2/node3 role settings
scripts/                  # install/start/stop/check/routing/autostart
services/                 # hansel-mesh@ and hansel-metrics@ systemd units
monitor/metrics_agent.py  # iw/batctl/ping/bat0 UDP JSON collector
```

The role IP defaults match the public HANSEL_MESH layout:

```text
base  192.168.50.1
head  192.168.50.10
node1 192.168.50.11
node2 192.168.50.12
node3 192.168.50.13
```

The bundled `metrics_agent.py` gathers `iw`, `batctl`, `ip neigh`, `ping`, and
`bat0` counters and emits the UDP JSON contract consumed by the ROS adapter:

```text
node, mesh_if, bat_if, ts, links, end_to_end, bat0
```

## 2. Install network bundle on each Pi

Example for a Head Pi whose login account is `ngt`:

```bash
cd ~/HANSEL_MESH_ROS2
sudo ./scripts/install_network_bundle.sh head
```

The default target becomes `/home/ngt/HANSEL_MESH` when invoked with `sudo` by
`ngt`. Other roles use `base`, `node1`, `node2`, or `node3`.

Edit the role-specific interface and IP settings before starting it:

```bash
nano ~/HANSEL_MESH/configs/head.env
sudo nano /etc/hansel-mesh/metrics.env
```

Start and inspect:

```bash
sudo systemctl start hansel-mesh@head
sudo systemctl start hansel-metrics@head
ip -brief addr show bat0
sudo batctl n
sudo batctl o
journalctl -u hansel-mesh@head -u hansel-metrics@head -f
```

Use the matching role name on every Pi. The Base remains a BATMAN-adv gateway
and relay; it is not an application-level command dispatcher.

## 3. ROS network adapter

`hansel_network_adapter` listens for snapshots on UDP port 7100 and publishes:

```text
/hansel/network/status
/hansel/network/detach_recommendation
/diagnostics
```

Example `/etc/hansel-mesh/metrics.env` on each Pi:

```bash
METRICS_DEST="192.168.60.2:7100"
METRICS_INTERVAL="5"
METRICS_PING_TARGETS="base head node1 node2 node3"
```

A detach recommendation is only evidence supplied to `detach_coordinator`.
The coordinator's MANUAL/AUTO gate still decides whether automatic separation
is allowed.

## 4. Exact upstream refresh

The checked-in bundle is a reviewed integration subset rather than a claimed
byte-for-byte repository mirror. On an internet-connected development machine,
create an exact latest upstream snapshot with:

```bash
cd ~/HANSEL_MESH_ROS2
./scripts/sync_from_github.sh
```

This writes the exact fetched files and commit ID under:

```text
upstream_snapshot/HANSEL_MESH/
```

It does not overwrite the parent integration wrappers, because those wrappers
support arbitrary Linux usernames and connect the upstream metrics contract to
ROS 2.

## 5. Radar

Keep the original TI mmWave reader as the sole UART owner:

```text
TI radar UART -> HANSEL_MESH radar_capture.py -> mission.jsonl
                                             -> optional raw capture
mission.jsonl -> hansel_radar_adapter -> PointCloud2 / OccupancyGrid -> RViz
```

This avoids two programs opening the same UART. The ROS adapter validates the
mission-log wrapper and `radar_frame` schema, skips incomplete frames, and maps
native axes to ROS axes:

```text
ROS x forward = raw y
ROS y left    = -raw x
ROS z up      = raw z
```

Because both Arduino Nano and Radar may appear as `/dev/ttyUSB*`, create stable
udev names such as `/dev/hansel_nano` and `/dev/hansel_radar_data` before field
operation.

## 6. Deliberate exclusion

The public repository's separate `rescue-network/` directory is a standalone
store-and-forward rescue-request AP/web application. It is not needed to create
`bat0`, route ROS DDS, collect mesh metrics, or drive the robot, so it is not
installed by the core network bundle.
