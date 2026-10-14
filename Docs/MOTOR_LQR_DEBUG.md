# Motor feedback and LQR debug

Motor addresses: left = 1, right = 2. No extra OLED page is added.

## Watch variables in Keil

Expand `g_motor_left` and `g_motor_right`:

- `feedbackSpeedRpm` / `measuredRpm`: signed shaft speed, rpm.
- `feedbackPositionDeg` / `measuredPositionDeg`: signed shaft position, degrees.
- `feedbackSpeedAt`, `feedbackPositionAt`: last valid response tick, milliseconds.
- `speedValid`, `positionValid`: a valid response has been received.
- `feedbackFrameCount`: total parsed speed/position responses.
- `rxCount`, `lastRxData`, `lastRxLength`, `replyFunction`, `replyCode`: raw receive diagnostics.
- `commandedCurrentMa`: last successfully transmitted current command, NOT measured current.
- `lastStatus`, `txFailureCount`: transmission diagnostics.

The protocol queries 0x35 (speed) and 0x36 (position). Current/temperature
feedback from the F4 DJI protocol is not fabricated for these motors.
Each control iteration queries one item, rotating left speed, right speed,
left position, right position. Nominal full rotation is 16 ms at a 4 ms period;
actual timing must be checked on hardware.

The obsolete asynchronous transmit bookkeeping has been removed.
Commands use bounded mailbox/transmit waits without waiting for command
response acknowledgements. Feedback older than 50 ms produces zero current and
continues querying; it does not latch FAULT 05. Fresh feedback resumes control.
IMU, excessive tilt/speed/displacement, controller disconnect and CROSS stop
protection remain. This is not a removal of every safety condition.

If either wheel's current command fails to transmit, both wheels are stopped
using the existing stop/retry path. This does not latch a communication fault,
but START is required to enable control again; it is not automatic recovery.

## What LQR needs

Watch `g_lqr.state`, `g_lqr.gain`, `g_lqr.rawCurrentA`, `g_lqr.currentMa`,
`g_lqr.feedbackAgeMs`, `g_lqr.feedbackProblem`, `g_lqr.overrunCount`.
The four states in the current implementation are position error (m), speed
(m/s), pitch (rad), pitch rate (rad/s). Control is current in amperes:
`u = -K * state`, then current limiting and conversion to milliamperes.

Before trusting the existing gain values, measure the actual wheel radius,
body mass, centre-of-mass height, body/wheel inertia, and the motor's
current-to-wheel-torque relationship (including any reduction ratio).
Choose consistent coordinates and signs, derive the upright linearised A/B
model with CURRENT as the input, discretise using the actual sampling period,
check controllability, choose Q/R, solve the discrete Riccati equation, compute
K, and check closed-loop poles. A force/torque-input gain must not be used
directly as a current-input gain without conversion.

The firmware already evaluates the state feedback; it does not identify your
mechanical parameters or solve the Riccati equation. Existing K values are not
validated for your assembled car. First verify motor direction and IMU signs
with wheels clear of the ground and a physical means to cut power; compilation
is not a hardware stability test.
