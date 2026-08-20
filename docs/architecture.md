# MC Aero — As-Built Architecture

The current state of the system, end to end. Companion docs:
`plant_params.md` (numeric plant), `controls_state_space.md` /
`controls_pid_handoff.md` (design handoffs — the PID handoff is historical;
the flying controller is the LQI below), `mission_modes_spec.md` (mission intent).

---

## 1. Hardware / peripherals (on the craft computer)

- **5 Rotation Speed Controllers (RSCs)** = the 5 independent command channels:
  `mainLift`, `forwardBack`, `leftRight`, `yaw`, `upDown`.
- **8 propeller bearings** grouped under those RSCs (7 gyroscopic + 1 plain):
  main-lift L/R, translation L/R, tail-yaw L/R, up/down, forward/back. The gyro
  bearings self-stabilize roll/pitch and expose `getThrustVector` / `setManualTarget`.
- **Sensors:** `gimbal_sensor` (angles, body rates, accel, gravity),
  `altitude_sensor` (height, vertical speed, air pressure), 3× `velocity_sensor`
  (body-frame x/y/z velocity), `navigation_table` (heading/bearing/distance),
  `physics_assembler` (mass, inertia tensor, CoM).
- **2 monitors** (flight + systems), **linked typewriter** (manual keys),
  **ender modem** (telemetry broadcast + command receive).

Peripheral names/roles are in `aircraft/config.lua`.

---

## 2. Coordinate & body frame (frozen)

Body frame = the velocity-sensor frame, right-handed:
- **x** fore/aft (forward = −x), **y** up, **z** lateral (left = +z). Yaw about +y.
- Verified against the sensors; `x × y = z`.

World↔body rotation (fitted from flight data): `bodyVel = Rot(heading)·worldVel`
with a small offset, `θ = heading − 1.5°`. This single fitted rotation is used
consistently by the controller (position-error rotation), the guidance
(bearing-to-target), and the GPS CoM offset. Getting this rotation right is what
made position hold stop spiraling.

---

## 3. Control: discrete MIMO LQI (`aircraft/lqi.lua` + `plant_K.lua`)

Gyros hold roll/pitch; the computer regulates translation, altitude, heading,
and yaw rate. Grey-box plant built analytically (`model/build_plant.py`):

- **State** `x = [s_x s_y s_z ψ  v_x v_y v_z r  ω_1..ω_5]` (positions/heading errors,
  body velocities, yaw rate, per-channel actuator states) + 4 **integral** states
  on `[s_x, s_y, s_z, ψ]` → 17-state LQI.
- **A/B** from geometry (`r_i`), fixed thrust directions (`d_i`, read from
  `getThrustVector`), thrust/RPM slopes (`k_i`), actuator time constants (`τ_i`),
  mass, and yaw inertia. `CMD_GAIN ≈ 0.10` maps RSC "target speed" to the
  propeller RPM the `k_i` were fit against (measured from pulse tests).
- **K** solved offline (discrete algebraic Riccati) and shipped as a constant.
  Runtime cost is one 5×17 matrix-vector product per tick: `u = u_nominal − K·x_I`.
- **Tuning** via per-channel control weights `RHO_CH` (raise = gentler) and the
  Bryson state weights in `build_plant.py`. Controllability + closed-loop
  stability are checked at build time.

Runtime notes:
- Velocities and yaw rate come straight from the body-frame sensors (yaw rate =
  gimbal angular-rate **index 2**, about +y). Position error is the world error
  rotated into the body frame by the fitted rotation.
- Actuator states are propagated with the first-order lag (command-equivalent).
- **Integral / anti-windup:** the actuator-state feedback attenuates steady
  command authority ~30×, so altitude/position lean on the integral. Clamps are
  large (`xiMax`) and integration is **frozen while a channel is saturated**
  (conditional anti-windup) so a floored climb/cruise can't wind up. A new
  command clears only the horizontal integrals (altitude/heading integrals are
  preserved so the craft doesn't sag/twitch).

Config: `aircraft/lqi_config.lua` (hover feedforward, `tau`, limits, `xiMax`,
`headingOffsetDeg`, `enable` gates, keys).

---

## 4. State estimation / position (`aircraft/sensors.lua`)

- **Primary position: GPS** (`gps.locate`, ~0.6 ms, inline every tick). Clean
  world X/Z, robust at speed, no overhead pole singularity.
- **Fallback:** nav-table lodestone bearing/distance reconstruction (used only if
  GPS returns nil). This reconstruction breaks up at speed and near the overhead
  pole — hence GPS primary.
- **CoM referencing:** GPS gives the *computer's* world point, which is offset
  from the CoM (~10 blocks forward). `config.position.computerOffset` is rotated
  by heading (fitted rotation) and subtracted so the controller holds the true
  **center of rotation**, i.e. a yaw-in-place doesn't orbit the setpoint.
- **Altitude** comes from the altitude sensor; **heading** from the nav-table
  compass (both unaffected by the position issues).

---

## 5. Mission / guidance (`aircraft/guidance.lua`)

A reference generator that turns a goal `{x, z, altitude, heading}` into the
instantaneous LQI setpoint. One controller/gain throughout; behavior comes from
the setpoint + commanded heading, not from swapping matrices.

State machine:
```
HOLD ─goto→ CLIMB → ORIENT → CRUISE → ARRIVE → DESCEND → HOLD(goal)
```
- **CLIMB** to `cruiseAltitude` holding launch X/Z. Leaves once within `climbBand`
  of cruise alt **and** vertical speed has plateaued (`climbVyLow`) — so it doesn't
  wait on the slow asymptotic final climb; the altitude loop keeps pushing during
  cruise.
- **ORIENT** yaws in place to face the goal (nose-on, so it doesn't fly sideways).
- **CRUISE** holds heading = bearing-to-goal and drives the setpoint at the goal.
  `directCruise = true` hands the goal straight to the LQI ("floor it," decel via
  the controller's own damping); `false` uses a decel-limited moving setpoint.
- **ARRIVE** at `arriveRadius` (kept large so it exits cruise before the
  bearing-to-goal gets twitchy over the target) and **freezes heading** so it
  stops chasing the bearing and spinning.
- **DESCEND** to the requested altitude holding X/Z, then **HOLD** at the goal and
  final heading.

Config: `config.mission` (`cruiseAltitude`, `vCruise`, `aDecel`, `maxLead`,
`arriveRadius`, `climbBand`, `climbVyLow`, `directCruise`, altitude bounds,
`maxGotoDistance`, `headingOffsetDeg`, `bearingFlip`, `callsign`, `commandTtlMs`).

---

## 6. Comms (`aircraft/startup.lua` receiver + `ground/pocket.lua`)

- The flight computer runs the control loop and a **command receiver**
  concurrently via `parallel.waitForAny`. The receiver blocks on
  `rednet.receive(commandProtocol)`; the control loop drains the mailbox each
  tick. CC coroutines are cooperative, so there are **no data races** on the
  mailbox.
- Protocol `mc_aero.command.v1`: `goto` / `hold` / `cancel` / `ping`, with `ack`
  replies. Messages carry a monotonic `id` and a timestamp.
- Robustness: **dedupe** by monotonic id per sender (ids seeded from epoch so they
  survive pocket restarts), **staleness** drop (`commandTtlMs`), optional
  **callsign** filter (multi-craft), and a **mailbox cap**. A pocket crash leaves
  nothing on the channel (rednet is fire-and-forget); the craft keeps flying its
  last commanded state.
- The **pocket** runs the S3 relay and a command console concurrently; sending a
  command never stalls the relay. Waypoints persist to `/waypoints.lua`.

---

## 7. Telemetry & cloud (`aircraft/telemetry.lua`, `cloud/`)

Aircraft broadcasts a full snapshot (`mc_aero.telemetry.v1`). A ground station or
the pocket batches and POSTs it to a Lambda Function URL → S3 NDJSON
(`telemetry/YYYY/MM/DD/…`). Local analysis via `uv run` scripts in `cloud/` and
`model/`.

---

## 8. Key discoveries (why things are the way they are)

- **Velocity sensors are body-frame** (heading-independent). The early
  position-hold "spiral" was a frame-mixing bug: world velocity command minus
  body velocity, then a wrong rotation. Fixed by fitting `bodyVel = Rot(heading)·
  worldVel` from flight data and rotating the *position error* into the body frame.
- **A ~90° sign/rotation issue** (not a sign flip) caused the original translation
  runaway; no amount of sign flipping fixes a rotation.
- **Actuator command→thrust scale ≈ 0.10** (RSC target speed vs propeller RPM),
  recovered from pulse tests; without it `K` was ~10× too hot (bang-bang).
- **Manual altitude pulses are pilot-contaminated** — the pilot reacts to sink/
  climb, injecting a false negative sign. Open-loop channel pulses are the clean
  way to identify effectiveness.
- **CoM vs sensor point:** `getCenterOfMass` returns Create *sub-level* coords
  (not world) and doesn't track world motion, so it can't be a position source;
  the CoM is referenced instead via a measured body-frame offset + the fitted
  rotation.

---

## 9. Open problems / TODO

- **Long-range positioning (>~2000 blocks):** GPS hosts and the lodestone both
  have limited range. Beyond that there is no direct world fix. Candidate: dead-
  reckon world position by integrating the body-frame velocity sensors (rotated
  to world by heading), re-anchored by GPS/lodestone whenever in range. Needs
  drift management.
- **Deeper controller cleanup:** rebuild `K` **without** actuator states (12-state)
  to remove the steady-authority attenuation, if altitude/position tracking needs
  to be crisper than the integral currently gives.
- **Confirm `computerOffset`/`bearingFlip`** in flight (fwd ≈ 10; flip if orient
  faces away).
