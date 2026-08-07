# Wiring and Arduino Nano contract

Source: `캡스톤_배선초안_0729.xlsx`, sheet `나노_핀맵_전체`.

## Pin map used by the firmware

| Function | Nano pin |
|---|---|
| Left PWM shared by front/rear drivers | D5 |
| Right PWM shared by front/rear drivers | D6 |
| Front-left direction | D7 / D8 |
| Front-right direction | D11 / D12 |
| Rear-left direction | D13 / A0 |
| Rear-right direction | A1 / A2 |
| Rear-left encoder A interrupt / B direction | D2 / D4 |
| Rear-right encoder A interrupt / B direction | D3 / A3 |
| 150 kg Head servo | D9 |
| SG90 detach servo | D10 |
| USB serial | D0 / D1, reserved |
| STBY | hardwired to 5 V |

The wiring sheet says one side must be reversed in software. The firmware starts
with right-side reversal enabled. Confirm the real forward direction with the
wheels lifted; change the four `*Reverse` constants in the `.ino` file if needed.

## Shared PWM consequence

D5 and D6 are physically branched to both motor drivers. Consequently:

- front-left and rear-left receive the same PWM magnitude;
- front-right and rear-right receive the same PWM magnitude;
- their direction inputs remain independent.

During normal driving the front directions follow the rear directions. For an
F/V front-only command, firmware momentarily puts the rear H-bridges in STOP and
uses the shared PWM for the front motors. The next normal motion frame restores
normal front/rear following.

## Encoder handling

Only rear-wheel encoders are connected. D2 and D3 are the Nano's external
interrupt pins; each ISR reads the corresponding B channel to determine count
direction. `encoder_counts_per_revolution` must be measured on the assembled
motor/gearbox rather than assumed.

## Servo handling

D9 and D10 use Arduino `Servo`, which owns Timer1. Motor PWM stays on D5/D6
(Timer0), matching the wiring-sheet timer allocation. The Head servo uses a
piecewise calibration around a logical center:

```text
[min logical, min pulse] -> [center logical, center pulse]
[center logical, center pulse] -> [max logical, max pulse]
```

Start with conservative pulse widths. Full `-180°..+180°` is only a software
coordinate and must not be used until the servo model and mechanism are proven
safe.

## Raspberry Pi pins kept free

The control backend uses Nano USB serial and therefore does not consume Pi GPIO
motor pins. The uploaded BNO085 mapping remains available:

- GPIO2 / physical pin 3: SDA
- GPIO3 / physical pin 5: SCL
- GPIO4 / physical pin 7: optional INT
- physical pin 1: 3.3 V only
- physical pin 6: GND

All grounds, including motor and external servo supplies, must be common.
