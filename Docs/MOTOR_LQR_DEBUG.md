# Motor feedback and LQR debug

Motor addresses: left = 1, right = 2. No extra OLED page is added.

## Manual tuning mode

Boot is disabled. Each START press toggles the software enable request; holding
START does not repeatedly toggle it. There is no automatic startup, fault latch,
CROSS stop, controller-loss stop, tilt/speed/displacement stop or CAN-error stop.
START does not depend on LQR posture, speed, centred axes or fresh feedback.
Successful boot gyro calibration is now required before START can enable the
motors. This is a startup validity check, not a new runtime auto-stop condition.
Hold the car stationary during the one-second settling delay and 500 samples
at five-millisecond intervals. Motion checks permit up to three attempts. A
failed calibration leaves `gyroCalibrated=0` and displays `IMU CAL FAILED`;
secure the car and reset. See `LQR_PARAMETERS.md` for thresholds and limitations.

LQR continues calculating while disabled for Watch observation. Once enabled,
it keeps sending bounded current commands using the stored motor/IMU values;
stale/missing feedback flags do not block output. Lost controller input freezes
the position-reference advance but does not disable the motor. Non-finite
calculation results produce zero current for that iteration without changing
enable state. Current clamping and bounded CAN waits remain.

This deliberately removes application-level automatic shutdown. A stale
measurement or failed wheel transmission may leave the motor driven; START
cannot work through a disconnected controller or broken CAN link. Secure the
car with wheels clear of the ground and use a physical power cutoff. The
software enable request is not actual motor-state feedback, and motor firmware
protections are separate. Failed manual disable requests are retried.

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

Commands use bounded mailbox/transmit waits without waiting for command
response acknowledgements. Failed sends do not invoke stop/disable or latch
an error; the next control iteration continues submitting commands. START
sends motor enable commands without the former startup heartbeat configuration.

## What LQR needs

The current parameter set and Chinese calibration guide are in
`LQR_PARAMETERS.md`. Pitch is displayed with one decimal place. The default
balance reference is +3.7 degrees in the existing corrected MPU frame, not an
extra installation offset. Tune `g_lqr.balancePitchDeg` while disabled first.

Watch `g_lqr.state`, `g_lqr.gain`, `g_lqr.positionReference`,
`g_lqr.rawCurrentA`, `g_lqr.currentMa` and `g_lqr.saturated`.
Feedback age is checked locally; age arrays, problem masks, timing counters and
intermediate reference-step fields are no longer stored in `g_lqr`.
The four states in the current implementation are position error (m), speed
(m/s), pitch minus `g_lqr.balancePitchDeg` (rad), pitch rate (rad/s). Control is current in amperes:
`u = -K * state`, then current limiting and conversion to milliamperes.

The current default `g_lqr.outputScale=1.0` multiplies `-K*state`
before the current +/-1500 mA clamp. `rawCurrentA` includes the multiplier.
Set the scale to 1 to restore prior effort. This does not multiply response
speed; nominal slow position modes do not improve. Zero is not a motor stop,
because yaw PID remains active. Validate only on a restrained rig.

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
- `g_car`: filters, current targets, software enable request and manual-stop retry state. Fault codes, latches and automatic-startup state are removed.
- `LowPassFilter_TypeDef`: all three members participate in filtering and remain.

Global structures occupy static RAM, not task-local stack. Local snapshots also
use stack: the control task now checks feedback age directly in the short
integer-only part of its critical section instead of keeping two snapshot arrays.
Task stack allocations are unchanged. Keil's static call graph is not a runtime
stack high-water measurement and does not prove interrupt/context-switch margin.

Software current limits now match at +/-1500 mA per motor (LQR formerly 500,
TX formerly 1000). The user reports setting the drive ceiling to 4400 mA;
firmware does not read back or rewrite it. ZDT manual p.51 defines F5 current
in mA; p.82 defines the X-firmware closed-loop ceiling, range 0-5000 mA.
OLED target-current fields use four digits. This staged increase is not a
continuous-current rating or evidence that position oscillation is fixed.

The former 2.5x output experiment was rolled back to 1.0 after the user
reported aggressive correction and shaking reduced by hand support. The
user-adjusted +3.7 degree balance reference and +/-1500 mA limits remain.
Hand support changes the plant and is not proof of free-standing stability;
use a restrained rig and physical cutoff, not hands, for further testing.
