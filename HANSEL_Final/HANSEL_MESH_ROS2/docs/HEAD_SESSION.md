# Integrated Session

Status: implemented and tested locally. Not uploaded to BASE/Head; no hardware
motion, UART recovery, Raspberry Pi build or real mesh throughput verified here.

The Korean step-by-step procedure, including the exact Nano sketch/header,
Arduino IDE selection, per-device backups/transfers/builds, first-run checks,
zeroing, controls and shutdown, is in
[HANSEL Final README](../../README.md).

## Source

Outer repository: `HANSEL_MESH`, branch `260926`, commit
`bbf2311f56d35cacae4269f5e3bc53cd74a95bdd`.
Radar lives in the nested `260926/HANSEL_MESH` repository restored from the
uploaded bundle at `10f0b337f12b887de83e3e240250accc475a1edc`.
Do not substitute the older outer-root radar files. This release includes both
updated source directories as ordinary files under HANSEL_Final; no nested Git
checkout or bundle restoration is needed.

## One PC Terminal

After the one-time deployment below, from this ROS repository on the PC:

```bash
python3 controller/run_head_session.py --dry-run
python3 controller/run_head_session.py
```

For Head + node1 + node2, use `--with-nodes` (also supported with `--dry-run`).
Check node1/node2 SSH addresses in the config before use. Their defaults are
192.168.50.11 and 192.168.50.12. BASE still owns keyboard/routing; the two nodes
run the team's `run_node.sh` GPIO wrapper, each with a 75% PWM limit. Existing
BATMAN/AP services are neither stopped nor reconfigured when a unit detaches.
Each full launch is a new mission with all selected units physically attached.
Detached state is intentionally not restored from a previous session; reconnect
the units before starting again with `--with-nodes`.
See [Drive and Detach](DRIVE_DETACH.md) for firmware and separation details.

PC needs Python >= 3.8, OpenSSH, browser and ffplay. It does not need ROS.
Passwords are entered in SSH, not stored by the launcher. Existing host-key
verification remains enabled. Configure addresses/ports in
`configs/head_session.json`. Defaults match the user's Head-only setup:
BASE `192.168.60.1`, Head `192.168.50.10`, PC video receiver `192.168.60.2`.

The launcher runs the team's wrappers, not a replacement motor controller:

| Host | Processes |
| --- | --- |
| BASE | operator stack and interactive keyboard, Head target only; RQT off |
| Head | unit/Nano driver, local input forwarder, odometry, map, radar, camera |
| PC | SSH multiplexing/tunnel, browser, ffplay |

Map opens at `http://127.0.0.1:8083/`; radar is left, trajectory right, +X up.
Camera opens separately in ffplay using the newly generated sender SDP, not an
old file copied from another session. Ports: radar 8081, combined map 8082,
PC tunnel 8083, RTP 5600. Camera UDP still needs the real PC network route;
the SSH HTTP tunnel does not tunnel RTP. `--no-camera` and `--no-browser` are
available. Logs go to `runtime/sessions/<session-id>/`.

SSH authentication finishes for all selected devices before any runtime starts.
The map tunnel opens after runtime startup, so an old browser tab cannot send
map requests while another device is still asking for its SSH password.
SSH transport errors are recorded separately in `ssh-base.log`, `ssh-head.log`,
`ssh-node1.log` and `ssh-node2.log` in the session directory. Password prompts
remain in the terminal; passwords are not recorded. A map tunnel failure warns
without stopping drive. This does not repair a dropped mesh connection.

PWM upper limit is 75%; keyboard speed scale is 0.75. For W/S, speed scale sets
the velocity target, not a fixed PWM duty, on all active drive units. Q/E and A/D
request the 75% limit on the Head wheels, using the existing PWM ramp; the router
sends STOP to active rear units for these four commands. The launcher starts at STOP, never sends G,
never clears E-stop and never sends a movement command itself.

Press X/Space to stop. P exits the keyboard and shuts down the session's owned
processes. Existing manual sessions are not killed or silently replaced: stop
their owning terminals/services before switching to this launcher. An IMU/radar
failure is logged and does not disable the motor driver. It also does not mean
the robot has valid localization or obstacle information.

F runs the front motors only until V, X/Space, or a normal drive key is pressed.
V stops the front motors; W/S/A/D/Q/E restore normal wheel control. The Nano's
shared PWM wiring cannot independently drive front and rear motors at different
speeds, so F is a front-only mode, not a parallel front/rear command.

## Mesh Bandwidth

Drive retains the team's CycloneDDS/bat0 setup in domain 0. Odometry/map use
domain 42 with DDS restricted to loopback. `head_local_inputs` receives only
existing Head wheel/diagnostic, active-chain and low-rate detach events in the mesh domain,
then forwards CDR payloads through a Unix datagram socket on Head. It does not
publish localization/path/IMU topics onto bat0. Delayed wheel datagrams are
discarded, not replayed as new measurements.

PC gets read-only HTTP data, not ROS subscriptions. Odometry polling is 2 Hz,
radar polling at most 5 Hz, with gzip HTTP responses. These rates reduce
duplicate snapshots but do not establish an actual BATMAN bandwidth budget.
Camera remains the existing 1.2 Mbit/s RTP stream. Real packet-loss/throughput
still needs on-site measurement.

To zero odometry while stationary, use a separate Head SSH shell:

```bash
cd ~/HANSEL_MESH_ROS2
source scripts/ros_local_env.sh
timeout 10s ros2 service call /hansel/head/localization/zero std_srvs/srv/Trigger '{}'
```

Do not source `ros_env.sh` after `ros_local_env.sh` in this shell. The existing
manual `run_head_odometry.sh`/`run_head_map.sh` wrappers are legacy entrypoints;
the integrated launcher deliberately does not use them. Keep ROS clock sync
working for BASE-to-Head commands. The launcher warns about clock differences;
it does not step clocks while the robot is running.

## Changes and Evidence

1. Repeated G: command watchdog used to clear drive enable. Runtime config now
   sets `watchdog_requires_enable: false`. Missing commands still zero motors;
   only a fresh, increasing-sequence command resumes motion. Manual disable,
   E-stop, detached relay state and non-transport faults remain latched. Parameter
   changes require node restart. Legacy latched mode is still available with
   `true`. Firmware's independent watchdog is unchanged.
2. Keyboard restart: sequence 0 with the fixed source `keyboard` could be
   rejected after an earlier long session. A per-session keyboard ID fixes this
   without removing sequence or timestamp checks. BASE needs this update too.
3. Radar port: explicit by-id paths are resolved against the enumerated device;
   Application/User interface and board serial identity are still checked.
4. Radar warm restart: user logs showed low-baud garbage and high-baud binary
   traffic, consistent with a board left streaming at 1250000. The loader
   probes both profile bauds, stops a detected stream at the working baud,
   and reapplies the complete profile. It does not accept any bytes as valid
   radar frames. Software recovery is tried without requiring xds110reset or a
   physical unplug. Exceptions and response tails are now in the manifest/log.
   This is a tested hypothesis, not a hardware-confirmed diagnosis of every
   earlier failure. TI protocol reference:
   https://software-dl.ti.com/ra-processors/esd/MMWAVE-L-SDK/05_05_00_02/exports/api_guide_xwrL64xx/MMWAVE_DEMO.html
   The October recovery update waits for a complete `Done` and the following
   prompt instead of advancing on either one alone. On normal session exit,
   capture closes its UART first, then the supervisor stops its own radar stream.
   A lost stop acknowledgement is checked against a fresh version reply and
   further frames, not accepted just because a prompt appeared.
   The Head wrapper verifies three complete frames before opening the viewer
   (about 0.6 s of frames at 5 Hz), retries failed attempts after 0.25 s up to
   1 s, and uses a separate 3 s CLI reply timeout. Healthy streaming is not
   periodically reset. A viewer exit restarts only the viewer; an unexpectedly
   exited live processing thread also makes the viewer exit for replacement,
   instead of leaving an HTTP server serving frozen data. A capture-process
   exit first tries the already verified stream once; missing frames or observed
   USB removal fall back to full profile application on the same board serial.
   The offline radar pane retries every second. This does not change motor
   enable, watchdogs, camera, IMU, odometry, profile, calibration or path planning.
   The managed radar process has a longer exit grace period for UART cleanup;
   the other managed processes retain their existing shutdown timing.
   These changes need the radar Python files (including `monitor/radar_front.py`)
   and Head wrapper on Head,
   `scripts/run_managed.py` on Head, and `monitor/odometry_map/radar-retry.js`
   for the combined map. No ROS interface rebuild or Nano upload is needed
   for this radar-only update. Hardware power loss, a loose USB cable or a
   firmware that stops responding still cannot be repaired by UART retries.
5. IMU: logs with zero raw UART bytes were not valid IMU reception. Loose wires
   and a sensor recovered only by power cycling cannot be repaired in software.
   Diagnostics now distinguish zero bytes, stopped stream and invalid frames.
   Silent UART is held open for 5 seconds before the existing reconnect policy;
   the 0.35-second stale-input integration limit remains unchanged.
6. Radar processing: synthetic browser testing reproduced division by zero
   when the second line-fit refinement eliminated all support points, killing
   the radar data thread while HTTP survived. The refined support is now checked
   before fitting. This was reproduced locally; earlier field disconnects cannot
   all be attributed to it without their tracebacks.
7. Responsive route mode: the launcher sets `HANSEL_RADAR_PATH_MODE=responsive`.
   Three distinct nearby cells with current, confirmed point tracks >= 0.50
   confidence can enter collision geometry before one track reaches 0.85.
   Isolated, stale, duplicate or weaker points do not qualify. Path switching
   confirmation is two frames instead of three, and SPI reflectivity openings
   are checked against confirmed collision geometry. Existing near-field
   horizon remains 0.30 m. Synthetic left/right tests pass; foil multipath and
   route usefulness require recorded field replay. This does not steer motors.
8. Distance now uses signed cumulative rear encoder count differences, not
   repeated CPS integration. Nano telemetry adds counts and uptime; WheelState
   adds a validity flag, boot epoch and sample ID. Identical samples cannot add
   distance. Dropped packets retain the measured count delta at the next sample;
   MCU reset/reopen starts a new baseline, not a position jump. Counter wrap is
   handled. CPS remains available for PID and display only. A missing heading
   rebases counts on return; the path during an IMU outage cannot be recovered.
9. Retryable motor I/O failures no longer permanently latch drive off. The
   controller clears its old target and retries at one-second intervals. Nano
   is reopened using its configured by-id path when needed; fresh telemetry and
   a successful zero-output write are required before accepting renewed motion.
   GPIO reconnects to local pigpiod. Neither recovery path sends G or changes
   manual-disable, detached-unit or E-stop state. Servo/drop commands are not
   automatically replayed. Missing Nano feedback stops PID output until it
   returns; stale CPS must not be treated as a fresh speed measurement.
10. A detach command that was never dispatched releases temporary holds on all
    units. Hold-release failures are retried, and sequence tracking prevents an
    old release from cancelling a newer hold (or an old hold from reappearing).
    A dispatched detach with missing completion still keeps the possibly
    released tail stopped, without permanently faulting the remaining actuator
    unit. This is not evidence of mechanical separation. The keyboard now
    prints detach service results.

Known separate limitation: the head-servo callback still does not perform the
drive callback's age/E-stop checks; the new interpolation is not a complete
actuator safety certification. Approximate 1 m distance scaling remains provisional.
Keep the existing distance scales for the initial comparison, then repeat a
measured-distance run: the old CPS-based calibration is not new count-mode
accuracy evidence. No extra E-stop behavior was added in this revision.

## One-Time Deployment

Do this stationary, with manual launchers stopped. No deployment command has
been run by the assistant. Bundles contain only the changed runtime files,
not binaries, credentials or the whole repository. A separate `nano-firmware`
archive contains the updated sketch and header. Copying it does NOT flash Nano.

This revision changes `WheelState.msg`. Deploy and rebuild `hansel_interfaces`
on BASE, Head, and both rear nodes when using them, with the dependent packages
listed below. Stop old processes before replacing their message definitions.
Flash the new Nano telemetry firmware too. Old T packets may still serve drive
diagnostics, but are marked invalid for count odometry; there is no CPS fallback.
Regenerate the archives: previously generated deployment bundles are outdated.

On PC, in this ROS repository:

```bash
bash scripts/package_head_session.sh /tmp/hansel-session-update
scp /tmp/hansel-session-update/base-ros.* hansel@192.168.60.1:/tmp/
scp -o ProxyJump=hansel@192.168.60.1 /tmp/hansel-session-update/head-ros.* /tmp/hansel-session-update/head-radar.* hansel@192.168.50.10:/tmp/
```

Review the `.txt` manifests. These are patches for the uploaded snapshot, not
for a different or subsequently edited BASE/Head source version. Preserve any
new on-device edits before applying. The runtime YAML includes shared settings;
compare it with the current device YAML when those settings have changed.

On BASE, back up existing files (the helper is new), then apply and build:

```bash
cd ~/HANSEL_MESH_ROS2
BACKUP="$HOME/hansel-base-before-session-$(date +%Y%m%d-%H%M%S).tar.gz"
tar -czf "$BACKUP" controller scripts ros2_ws/src/hansel_operator ros2_ws/src/hansel_bringup ros2_ws/src/hansel_interfaces
tar -xzf /tmp/base-ros.tar.gz
source scripts/ros_env.sh
cd ros2_ws
colcon build --symlink-install --packages-select hansel_interfaces hansel_operator hansel_bringup
```

On Head:

```bash
cd ~
tar -czf "hansel-head-before-session-$(date +%Y%m%d-%H%M%S).tar.gz" HANSEL_MESH_ROS2/robot HANSEL_MESH_ROS2/scripts HANSEL_MESH_ROS2/monitor HANSEL_MESH_ROS2/ros2_ws/src HANSEL_MESH/sensors HANSEL_MESH/monitor HANSEL_MESH/scripts
tar -xzf /tmp/head-ros.tar.gz -C ~/HANSEL_MESH_ROS2
tar -xzf /tmp/head-radar.tar.gz -C ~/HANSEL_MESH
cd ~/HANSEL_MESH_ROS2
source scripts/ros_env.sh
cd ros2_ws
colcon build --symlink-install --packages-select hansel_interfaces hansel_unit_control hansel_bringup hansel_localization
```

Do not launch until builds succeed. The archive includes a Head-only setup;
node1/node2 do not receive changes automatically. For those units, deploy the
unit-control changes/config and rebuild separately when they are in scope.

## Verification

Local results: ROS-independent control/session suite 102 passed, with 6
subtests. Count-mode/reconnect tests include counter wrap, duplicated and
out-of-order samples, fragmented UART reads, Nano/GPIO reconnect, failed
detach hold release, and F/V persistence. The Nano sketch also compiles to an
ATmega328P firmware image with the installed legacy Arduino core: full compile
and link passed, 13,114 bytes program and 688 bytes static data. The verification
command supplied the core's missing digitalPinToInterrupt mapping for D2/D3.
This is not an Arduino IDE board-package build or firmware upload, and does not
measure peak stack use. The radar suite was rerun: 808 tests / 2 platform skips,
424 subtests. Desktop and mobile browser checks were also rerun and passed.
All of these use
synthetic/fake-device data, not physical robot tests.

Local tests use Python 3.13, matching the Head logs; the PC launcher also supports
Python 3.8. Test dependencies are pytest, pyserial, numpy and PyYAML. Run each
repository's tests separately to avoid their `tests` package-name collision.

```bash
python3 -m pytest -q tests
```

For explicit synthetic browser testing (not robot data), run the radar demo on
18081, map demo on 18083, then `tests/check_session_browser.py`. It checks desktop
and mobile bounds, nonblank canvas pixels, zoom/follow and JSON round-trip.
Real ROS cross-domain forwarding, remote SSH lifecycle, hardware UART recovery,
camera display and loss/recovery behavior still need an on-device acceptance
run. First test startup/stop with wheels clear and motor output disconnected;
then check controlled command loss and fresh-command recovery before driving.
