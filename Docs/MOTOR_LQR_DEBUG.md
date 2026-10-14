# Motor feedback and LQR debug

Motor addresses: left = 1, right = 2. No extra OLED page is added.

Boot now automatically enables control once initialization, fresh feedback,
safe stationary posture and a connected, centred controller permit it. START
is not required for this first enable. CROSS cancels pending startup even while
disabled. Manual stop, fault or failed transmission never automatically restarts;
START is required to enable again. Reset/reflash can cause motor activation.

## Watch variables in Keil

Expand `g_motor_left` and `g_motor_right`:

- `address`: motor ID (1 / 2).
- `measuredRpm`: signed shaft speed, rpm.
- `measuredPositionDeg`: signed accumulated shaft position, degrees; used to calculate LQR displacement.
- `speedUpdatedAt`, `positionUpdatedAt`: last valid feedback tick, milliseconds.
- `speedValid`, `positionValid`: a valid response has been received.
- `commandedCurrentMa`: last successfully transmitted current command, NOT measured current.

Only these eight fields remain. Duplicate measurements, raw frames, command
acknowledgements and query/transmit counters have been removed. The EMM position
query is not the F4 DJI raw encoder count; no fake `ecd` or measured current is
added. Required validity/timestamps remain to reject missing or stale feedback.

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

Watch `g_lqr.state`, `g_lqr.gain`, `g_lqr.positionReference`,
`g_lqr.rawCurrentA`, `g_lqr.currentMa` and `g_lqr.saturated`.
Feedback age is checked locally; age arrays, problem masks, timing counters and
intermediate reference-step fields are no longer stored in `g_lqr`.
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

## Other application structures

- `g_mpu6050`: acceleration, angular rate, gyro bias, pitch/roll/yaw and required initialization/health/timestamp fields. Raw sample copies, temperature, WHO_AM_I cache and communication configuration are removed from the public structure.
- `g_ps2`: buttons, button edges, axes, connection and update time. `g_ps2_diag` and the cached protocol mode are removed.
- `g_oled`: existing page selection only. The single display's I2C handle and write-failure gate are private driver state; the address is constant.
- `g_car`: filters, output targets and active startup/stop/safety state. `faultCode == 0` replaces the redundant fault flag.
- `LowPassFilter_TypeDef`: all three members participate in filtering and remain.

Global structures occupy static RAM, not task-local stack. Local snapshots also
use stack: the control task now checks feedback age directly in the short
integer-only part of its critical section instead of keeping two snapshot arrays.
Task stack allocations are unchanged. Keil's static call graph is not a runtime
stack high-water measurement and does not prove interrupt/context-switch margin.
