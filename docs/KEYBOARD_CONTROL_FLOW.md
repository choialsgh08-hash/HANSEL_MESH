# Keyboard-to-actuator flow

```text
operator_input (single-key cbreak terminal)
  ├─ persistent W/S/A/D/Q/E/Z/C command, resent every 100 ms
  ├─ one-shot U/J/K Head servo command
  ├─ one-shot F/V front motor command
  └─ detach / E-stop / enable service requests
          │
          ▼
/hansel/system/command/motion
          │
          ▼
command_router
  ├─ Head receives original semantic command
  ├─ rear curve -> slow_forward/slow_backward
  ├─ rear spin -> stop
  └─ Head-only command -> Head only
          │
          ▼
unit_controller (50 ms)
  semantic command -> signed target CPS
  encoder CPS -> minimum-PWM feed-forward + PID
  PID output -> 220 %/s ramp -> signed PWM
          │
          ▼
Nano USB serial
          │
          ▼
Arduino Nano
  D5/D6 shared PWM + independent H-bridge directions
  D2/D3 encoder interrupts
  D9 Head servo / D10 detach servo
```

## Exact steering targets at speed scale `s`

Let full left/right speeds be `L` and `R` CPS.

| Command | Head left | Head right | Rear result |
|---|---:|---:|---|
| forward | `+L·s` | `+R·s` | same |
| backward | `-L·s` | `-R·s` | same |
| left | `-0.85L·s` | `+0.85R·s` | stop |
| right | `+0.85L·s` | `-0.85R·s` | stop |
| forward_left | `+0.45L·s` | `+1.0R·s` | `slow_forward` |
| forward_right | `+1.0L·s` | `+0.45R·s` | `slow_forward` |
| backward_left | `-1.0L·s` | `-0.45R·s` | `slow_backward` |
| backward_right | `-0.45L·s` | `-1.0R·s` | `slow_backward` |

Rear `slow_*` uses the configurable `node_slow_ratio`, default `0.45`.
