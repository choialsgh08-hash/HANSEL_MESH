# Phase 0 Linux operation and service setup

This note covers the Phase 0 deployment changes.  Development on Windows is
supported, but the final runtime checks must be performed on Raspberry Pi OS
because BATMAN-adv, `iw`, systemd, GPIO, the camera stack, and the real network
interfaces are not available on Windows.

## Windows development rules

- Git enforces LF for `.sh`, `.service`, `.env`, and `.py` through
  `.gitattributes`. This protects Linux shebangs from CRLF conversion.
- Python syntax, parser tests, JSON protocol tests, and dry-run control tests
  can run on Windows.
- Bash syntax can be checked with Git Bash, WSL, or a Linux CI runner.
- Do not treat a Windows-only test as hardware validation.
- Before field use, deploy the exact commit to a spare Pi and run the Pi
  checklist at the end of this document.

## Install service files

On every Pi:

```bash
cd /home/hansel/HANSEL_MESH
sudo ./scripts/install_mesh.sh
```

This installs, but does not immediately start:

- `hansel-mesh@.service`
- `hansel-control@.service`
- `hansel-metrics@.service`
- `hansel-camera.service`

`hansel-control@ROLE` is intended for `head`, `node1`, `node2`, and `node3`.
The base does not enable a motor server.

The production motor service now fails closed if `RPi.GPIO` is unavailable.
GPIO-free tests must pass `--dry-run` explicitly; a missing hardware dependency
can no longer make a production service appear healthy while controlling
nothing.

## Control source allowlist

The installer creates `/etc/hansel-mesh/control.env` on first install with the
repository's default operator laptop address:

```text
HANSEL_CONTROL_ALLOW_SOURCES="192.168.60.2/32"
```

Review this file before enabling a motor service. Multiple IPs or CIDRs can be
comma-separated. Do not add the rescue AP subnet: victim phones must never be
able to reach the motor command service. Existing administrator changes are
preserved by later installer runs.

The systemd motor service uses `--require-source-allowlist` and therefore
refuses to start if this setting is missing or empty. The application check is
defense in depth, not a replacement for firewalling the rescue AP subnet away
from UDP port 7000. Production mode also requires an allowlist even when that
flag is omitted. The service starts through `start_control_server.sh`, which
binds UDP 7000 to the role's `IP_ADDR` on `bat0` instead of all interfaces.

## Acknowledged control-session startup

The operator and robot now use protocol version 1 JSON commands with a random
session ID, strictly increasing sequence number, monotonic-time TTL, and an
applied/rejected ACK. Because two machines' monotonic clocks have unrelated
origins, the first accepted packet in every `(source, session)` must be
`stop`. The official client sends that stop and waits for every selected
target's ACK before accepting operator input. A first motion command is
rejected with `session_requires_initial_stop`.

Within one running server session, excess-delay and duplicate non-stop
commands are rejected. A validly addressed `stop` remains fail-safe and is
applied even when duplicate or expired. This relative monotonic-time check is
not cryptographic replay protection: a previously captured initial `stop` and
following commands can be replayed after a server restart. Source allowlisting
also does not prevent IP spoofing or ACK forgery. Before field deployment, bind
commands to a server boot nonce and authenticate them (for example,
HMAC + boot nonce), in addition to isolating UDP 7000 with firewall rules.
Legacy plaintext control and raw detach actuation are disabled unless explicit
unsafe bench flags are used.

Safe detach is:

```text
all active targets stop + ACK
  -> released node relay_hold + ACK
  -> mapped front unit detach servo + ACK
  -> operator/sensor confirms physical separation
```

An actuator ACK confirms only software actuation, not physical separation.

## Persistent drive latch

The control service creates the root-owned `/var/lib/hansel-mesh` state
directory with mode `0700`. Each physical Pi stores its propulsion latch in
`drive-latch.json` using an atomic replacement. The file is deliberately not
role-specific, so changing a Pi from `node1` to `node2` cannot clear the hold.
Legacy `drive-latch-ROLE.json` records are read conservatively during
migration. After a successful
`relay_hold` or `drive_disable`, the disabled state survives both service and
Pi restarts; the server restores the hardware hold before opening its UDP
socket.

Before any `drive_enable`, the server durably creates
`drive-enable.pending`. Startup treats this transaction marker as disabled.
It is removed only after both the enabled state and the hardware enable have
succeeded, preventing a partial persistence failure from unlocking the next
boot.

A missing file means the unit has never been released and remains attached.
Invalid, corrupt, or unreadable state fails closed and restores
`relay_hold`. Do not delete a latch file to reset a released unit because a
missing file means enabled. Re-enable a bench unit explicitly with an
acknowledged `drive_enable` command.

`--unsafe-no-drive-state-persistence` is only an emergency bench escape hatch.
It must not be added to the production systemd service. `--dry-run` does not
read or write production latch state.

## Video-quality fail-closed behavior

Quality probes run in a background worker so ping/SSH/video parsing cannot
block the live motor refresh loop. Motion is suppressed with a zero speed cap
before the first sample (`NOT_READY`), after an update exception (`ERROR`),
when updates become stale (`STALE`), and in `DANGER`. A transient raw danger
also applies its zero cap immediately while hysteresis continues to govern
state reporting and detach escalation. Missing video/FPS evidence is danger,
but automatic detach is not armed until at least one usable `GOOD` or `WARN`
video sample has been observed; a camera that has not started cannot by itself
drop a unit. Starting live mode with `--auto-detach` also requires the operator
to type the exact preflight phrase `ARM AUTO DETACH` before any release servo
can be actuated.

WARN speed caps apply to the keyed front motor as well as wheel motion.
Encoder/PID loop exceptions latch a controller fault, stop all outputs, and
reject both motion and `drive_enable` until the controller is restarted and
inspected.

## Camera ownership

On the head, the production control server accepts a validated
`camera_profile` command only when `hansel-camera.service` is already active.
It changes only the profile through the root-owned `/run` override and queues
a non-blocking restart of that same service. Remote packets cannot replace the
administrator-controlled camera destination or transport, and an inactive
camera service is never started implicitly.

`--allow-camera-profile-restart` switches to the old unmanaged restart script.
That flag is an unsafe legacy bench option and must not be used alongside the
production camera service.

## Metrics configuration

Create an administrator-owned configuration on each Pi:

```bash
sudo install -d -m 0755 /etc/hansel-mesh
sudo cp configs/metrics.env.example /etc/hansel-mesh/metrics.env
sudo nano /etc/hansel-mesh/metrics.env
sudo chown root:root /etc/hansel-mesh/metrics.env
sudo chmod 0644 /etc/hansel-mesh/metrics.env
```

Set `METRICS_DEST` to the operator PC dashboard address. An empty destination
is allowed; snapshots will then be visible only in journald.

## Enable normal autostart

Run one role per Pi:

```bash
sudo ./scripts/enable_mesh_autostart.sh head
sudo systemctl start hansel-mesh@head
sudo systemctl start hansel-metrics@head
sudo systemctl start hansel-control@head
```

Replace `head` with the local role. The services depend on
`hansel-mesh@ROLE`, so the sensor/status and motor processes start only after
the mesh unit succeeds.

The control packets are versioned. Deploy the same commit to the operator PC
and every robot unit before starting control; an old client/server mixed with
this version is intentionally rejected. `scripts/deploy_from_base.sh`
refuses a dirty base checkout, records the exact Git revision, and preflights
every configured target before copying any files. It also refuses to deploy
while either a systemd control unit or a manual `mesh_control_server.py`
process is active. Support the robot, stop every control server first, and
leave them stopped until the coordinated deployment succeeds on all targets.
If deployment fails partway through, do not start control; correct the error
and rerun the same committed revision.

## Camera autostart is explicit opt-in

The camera service will not be enabled by the normal command. On the head:

```bash
sudo cp configs/camera.env.example /etc/hansel-mesh/camera.env
sudo nano /etc/hansel-mesh/camera.env
```

Set a real `CAMERA_DEST_IP`, review the port/transport/profile, and set:

```text
CAMERA_ENABLED="yes"
```

Only then enable it:

```bash
sudo ./scripts/enable_mesh_autostart.sh head --with-camera
sudo systemctl start hansel-camera.service
```

The optional service remains the only production camera owner. A validated
UDP `camera_profile` command now writes a single profile token under `/run`
and queues a non-blocking restart of `hansel-camera.service`; the destination
and transport still come only from the administrator-controlled
`camera.env`. Do not enable the legacy `--allow-camera-profile-restart` bench
path alongside this service, because that path starts an unmanaged streamer.

## Preserve a rescue AP on wlan1

Every role config now defaults to:

```text
STOP_GLOBAL_WIFI_SERVICES="no"
RESTART_GLOBAL_NETWORK_SERVICES="no"
```

`start_mesh.sh` releases interface-scoped services associated with `MESH_IF`
and asks a global `wpa_supplicant` to remove only that interface through its
global control socket or D-Bus API. It leaves global `hostapd`/`dnsmasq`
running, so a rescue AP on `wlan1` remains available while `wlan0` is rebuilt
as a mesh device. If `wpa_supplicant` still answers on `wlan0`, startup fails
closed instead of racing two owners of the same adapter.

Use `STOP_GLOBAL_WIFI_SERVICES=yes` only on a machine where no separate AP
must survive. It is a compatibility escape hatch for older single-interface
setups, not the normal configuration.

`stop_mesh.sh` also avoids restarting global network services by default.
If an old single-interface setup requires that behavior, set
`RESTART_GLOBAL_NETWORK_SERVICES=yes` in that role's config.

Both stop and status scripts accept either a role or a config path:

```bash
sudo ./scripts/stop_mesh.sh head
sudo ./scripts/stop_mesh.sh configs/head.env
./scripts/check_mesh.sh head
./scripts/check_mesh.sh configs/head.env
```

Calling them without an argument remains compatible with `wlan0`/`bat0`.

## Runtime logs

Field logs are now ignored under `logs/`, `report/`, and the two historical
root filenames. The already tracked root logs remain in Git history until an
intentional repository cleanup. To stop tracking them in a future cleanup
commit without deleting the local copies:

```bash
git rm --cached monitor_session.jsonl video_quality.jsonl
```

Review that change before committing. Do not run it during a field test.

## Raspberry Pi validation checklist

Run these checks on each Pi after deploying:

```bash
sudo systemd-analyze verify \
  /etc/systemd/system/hansel-mesh@.service \
  /etc/systemd/system/hansel-control@.service \
  /etc/systemd/system/hansel-metrics@.service \
  /etc/systemd/system/hansel-camera.service

sudo systemctl start hansel-mesh@head
./scripts/check_mesh.sh head
sudo batctl n
sudo batctl o

sudo systemctl start hansel-metrics@head
sudo journalctl -u hansel-metrics@head -n 30 --no-pager

sudo systemctl start hansel-control@head
sudo journalctl -u hansel-control@head -n 30 --no-pager
```

For each detachable role, issue an acknowledged `relay_hold`, restart
`hansel-control@ROLE`, and check the journal for the startup
`persistent-drive-latch` hold before attempting any lifted-wheel motion test.
Confirm that drive commands remain rejected until an explicit
`drive_enable`.

For AP coexistence, keep a phone connected to the `wlan1` rescue AP while
restarting `hansel-mesh@head`. Confirm that the AP SSID and portal remain
reachable, then verify that `wlan0` rejoined BATMAN.

If role-specific routing fails after `wlan0/bat0` was created, the role startup
script now calls `stop_mesh.sh` before returning failure so a half-configured
mesh is not left running.

Before moving real motors, lift the tracks/wheels, send a stop command, stop
the control service, and confirm that all outputs return to a safe state.
