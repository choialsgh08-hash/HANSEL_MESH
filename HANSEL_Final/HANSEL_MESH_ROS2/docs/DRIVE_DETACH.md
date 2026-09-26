# Drive, Horn and Detach Update

Implemented locally, not deployed or physically tested. This changes BASE,
Head, rear-unit Python code AND the Nano sketch. All four deployment targets
must use matching code for a three-unit demonstration.

## Controls

| Keys | Behavior |
| --- | --- |
| W / S | Forward / backward with existing speed target and PID |
| Q | Left wheel forward at maximum PWM; right wheel stopped |
| E | Right wheel forward at maximum PWM; left wheel stopped |
| A / D | Opposing wheel directions for left / right spin at maximum PWM |
| U / J | Increase / decrease horn target by 5 logical degrees |
| K | Return to configured center angle, using the same smooth motion |
| F / V | Front-only forward / stop; held across control ticks |
| 2 | node1 microservo releases node2 |
| 1 | Head microservo releases node1 |
| X / Space | Stop |

Q/E intentionally name the driven wheel, not the turning direction. Rear
units no longer replace these commands with slow straight driving or STOP.
Z/C and mild-turn commands retain their previous behavior.

All deployment defaults are 75%. Firmware also clamps at 75% (191/255 duty).
The existing 100 percentage-points/second motor ramp is retained, so the output
rises to 75% over about 0.75 seconds rather than switching instantly. Fixed-PWM
steering has no CPS target; wheel state reports zero target CPS with measured
CPS and applied PWM. W/S still use closed-loop speed control.
F selects front-only mode until another drive key or V/X is pressed. Rear wheels
stay stopped in this mode because the front and rear motors share PWM pins.
Transient transport failures are retried without requiring G, but old movement
targets and unconfirmed servo/drop commands are not replayed on reconnect.

## Horn

The old Nano handler attached, jumped to target, blocked for 80 ms, and detached
on every key message. That can cause repeated torque interruptions and input
backlog. It also hard-capped motors at 40%, independent of ROS settings.

The new Nano handler stores a target and advances it every 20 ms, with a
90 logical-degrees/second speed limit and 360 degrees/second-squared acceleration.
U/J targets are 5 degrees apart; holding a key uses terminal key repeat as before.
Servo pulses stay enabled by default, including at the final angle. Existing
pulse calibration and mechanical limits are unchanged. `head_servo_hold=false`
remains available, but now releases only 500 ms after reaching the target.
Telemetry's moving flag describes commanded pulse interpolation, not feedback
from the physical servo. Power-supply, mechanical backlash and load-induced
jitter still require an on-hardware check; software cannot establish their absence.

## Separation

1. The coordinator stops the currently attached drive units temporarily.
2. It actuates the preceding unit's microservo: node1 for node2, Head for node1.
3. It waits for the actual press/rest command sequence to finish. Nano reports
   `D,<token>,done`; rear GPIO waits for the pulse worker and propagates errors.
4. Released units are removed from routing and latched into relay-only state.
   G cannot re-enable them. Remaining units retain their previous enable state
   and accept the next fresh drive command without G.
5. A small timestamped event reaches Head's local odometry and freezes a marker
   from its pose history. Delayed delivery does not use Head's newer location.

Use 2 then 1 for individual drops. If 1 is pressed first, node1 AND the still
attached node2 are released from Head, so BOTH are removed from drive. The
default chain is exactly Head/node1/node2; the stray node3 default is removed.
BATMAN, AP, camera and other network services are not killed by this sequence.
The relay latch lasts for the controller session and is also restored from the
coordinator's retained chain. Restarting the entire system intentionally starts
a NEW chain with all selected units attached; physically reconnect them first.

Pulse widths remain 1515 us press / 800 us rest, 350 ms press and 120 ms rest.
Head's pin is Nano D10; node1's is BCM6 (physical pin31) via pigpiod.
These match the uploaded software, not a new physical wiring verification.
No separation sensor exists: servo completion is NOT proof the coupling opened.
On missing completion, no false success/marker is published and the possibly
released tail stays stopped. Inspect the coupling/connection before retrying.
If the actuator service was unavailable or explicitly rejected before actuation,
the tail is not considered uncertain: all temporary holds are released. Failed
hold-release responses are retried and sequence-checked. A missing servo ACK
does not permanently latch the remaining actuator unit into a motor fault.

Markers use the configured 0.20 m / 0.40 m trajectory offsets. They are explicitly
estimates, particularly during reversing, articulation or wheel slip. If path
history is shorter than the offset, the initial heading is extrapolated behind
Head. Missing event-time history is reported rather than fabricating a position.
Odometry zero clears the current map; old retained detach events are ignored in
the new origin. Save a map snapshot before zero if old drop locations are needed.

## Deployment

The session packaging script now produces `base-ros`, `head-ros`, `head-radar`,
`node-ros` and `nano-firmware` archives plus file manifests. See HEAD_SESSION.md
for BASE/Head deployment and backups. For EACH rear node, copy `node-ros.*`,
back up its current source, extract in `~/HANSEL_MESH_ROS2`, then build:

```bash
cd ~/HANSEL_MESH_ROS2
source scripts/ros_env.sh
cd ros2_ws
colcon build --symlink-install --packages-select hansel_interfaces hansel_unit_control hansel_bringup
```

Flash `robot/firmware/hansel_nano_bridge/hansel_nano_bridge.ino` together with
its adjacent `servo_motion.h` to the Head Nano using the existing working board/
bootloader selection, with motor power disconnected and the controller stopped.
No automatic flashing or actuator test is performed by the launcher. Until
flashed, old Nano firmware retains the 40% cap and does not acknowledge detach
completion or provide cumulative encoder counts. Do NOT mix the new host-side
code with the old firmware. `WheelState.msg` also changed; rebuild the interface
and dependent packages on BASE, Head and both rear nodes used in the session.

After deployment, the PC starts all three drive units plus Head sensors with:

```bash
python3 controller/run_head_session.py --with-nodes --dry-run
python3 controller/run_head_session.py --with-nodes
```

Without `--with-nodes`, it remains a Head-only session. Validate servo direction,
coupling travel, Q/E wheel choice and physical release on a supported stationary
robot before the moving demo. Tests here use simulated I/O, not physical motion.
