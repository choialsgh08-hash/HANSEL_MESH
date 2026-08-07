# RQT parameter scope

RQT is not a motion controller. Motion is keyboard-only.

The only editable values in the HANSEL RQT plugin are:

1. `Straight RPM` — W/S and the outer wheel of Q/E/Z/C.
2. `Turn RPM` — A/D in-place turn wheel magnitude.
3. `Head up limit` — positive logical upper limit, maximum +180 degrees.
4. `Head down limit` — positive magnitude converted internally to a negative lower limit, maximum 180 degrees.

All other control values are fixed in `ros2_ws/src/hansel_bringup/config/*.yaml` and are intentionally not shown in RQT: PID, PWM, encoder CPR, steering ratios, rear slow ratio, servo step, servo pulse calibration, serial settings, and safety timing.
