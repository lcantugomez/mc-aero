# MC Aero — PID Control Design Handoff (gyros ON)

Purpose: give a controls engineer everything needed to design the first
closed-loop controller (altitude hold + horizontal position hold + heading
hold) for the VTOL, assuming the gyroscopic bearings remain active. All plant
values below are **measured** from the Aug-16 actuator sweep unless noted.

---

## 0. Key assumption: gyros ON

The 7 gyroscopic propeller bearings run their own attitude stabilization
(`getStabilizationStrength = 1`) and hold the airframe roughly level. So the
controller does **not** need an attitude inner loop; it commands the 5 Rotation
Speed Controllers (RSCs) and treats the vehicle as a nearly decoupled,
force/torque-per-axis plant. (Turning gyros off is a separate, harder problem
that would need an attitude-effectiveness characterization first — out of scope.)

---

## 1. Coordinate frame & sign conventions  [measured]

World axes: **X = East(+)/West(−), Y = Up(+)/Down(−), Z = South(+)/North(−)**.
Heading/bearing: `0° = +Z (south)`, `90° = +X (east)`, increasing clockwise.
Gravity ≈ **11** (units/s²), pointing −Y. At hover the accelerometer
(`getLinearAcceleration`) reads specific force ≈ `{0, +11, 0}`.

The forward/back and left/right thrusters are **body-fixed**: their thrust
directions rotate with vehicle heading. The horizontal loop must resolve desired
world-frame force into body axes using the current heading (a 2-D yaw rotation).

---

## 2. Rigid-body parameters (from `physics_assembler_0`)  [measured]

- Mass `m = 709`
- Weight `W = m·g = 709 × 11 ≈ 7799` (force units)
- Inertia tensor (row-major, symmetric; axes X,Y,Z):
```
      X          Y          Z
X [ 17184.0    1093.1      129.5 ]
Y [  1093.1   68407.7       -3.6 ]
Z [   129.5      -3.6    61756.5 ]
```
  Products of inertia are small → near-principal axes. **Yaw uses Iyy ≈ 68408**
  (rotation about +Y). Roll(Ixx)/pitch(Izz) are handled by the gyros.
- Center of mass: `{20481017.8, 128.47, 20517896.5}` — **X/Z are Create sub-level
  coordinates, not world**; only Y (≈128.5) is world-meaningful. Use consistent
  sub-level/body coordinates for any lever-arm math.

---

## 3. Actuator model  [measured]

### 3.1 Command → bearing rpm (per axis, first-order)
`bearing_rpm(s) / command(s) = G / (τ·s + 1)`, commanded via RSC `setTargetSpeed`.

| axis (RSC)         | G (rpm/command) | τ (s) | settle ≈3τ (s) |
|--------------------|:---------------:|:-----:|:--------------:|
| mainLift (RSC_1)   | 0.30            | 0.59  | 1.8 |
| forwardBack (RSC_2)| 0.30            | 0.51  | 1.5 |
| upDown (RSC_5)     | 0.30            | 0.44  | 1.3 |
| yaw (RSC_3)        | 0.30            | 0.32  | 1.0 |
| leftRight (RSC_4)  | 0.30            | 0.32  | 1.0 |

G is constant and linear across the full 0–256 command range.

### 3.2 rpm → thrust (per prop, exact linear, R²=1.000)
`thrust_i = k_i · bearing_rpm_i` (intercept ≈ 0):

| prop (role)                 | k_i     | driven by | facing |
|-----------------------------|:-------:|-----------|--------|
| gyro_1 main_lift_left       | −168.65 | mainLift  | +Y |
| gyro_2 main_lift_right      | +168.65 | mainLift  | +Y |
| propeller_bearing_1 fwd/back| +78.38  | forwardBack | +X (body forward) |
| gyro_5 up_down              | +42.67  | upDown    | +Y |
| gyro_3 translation_right    | −15.08  | leftRight | ±Z (body lateral) |
| gyro_6 translation_left     | +15.08  | leftRight | ±Z |
| gyro_0 yaw_left             | −15.08  | yaw       | ±Z |
| gyro_4 yaw_right            | +15.08  | yaw       | ±Z |

Sign = thrust direction vs rpm sign. Mirror pairs (main lift, translation, yaw)
are ± so a single RSC produces net force on its intended axis (and, for yaw, a
couple). Live thrust direction is also available per prop as `getThrustVector`.

### 3.3 Per-axis force/torque authority  [derived: F = Σ k_i·G·command]

| axis        | effect            | force (or moment) per command | max @ command 256 |
|-------------|-------------------|:-----------------------------:|:-----------------:|
| mainLift    | vertical thrust   | 2·168.65·0.30 ≈ **101.2 /cmd** | ≈ 25,900 |
| upDown      | vertical trim     | 42.67·0.30 ≈ **12.8 /cmd**     | ≈ 3,277 |
| forwardBack | body-fwd force    | 78.38·0.30 ≈ **23.5 /cmd**     | ≈ 6,019 |
| leftRight   | body-lateral force| 2·15.08·0.30 ≈ **9.05 /cmd**   | ≈ 2,317 |
| yaw         | yaw moment (couple)| per-prop 4.52 /cmd × lever ×2 | ≈ ±1,158 thrust/prop |

**Hover feedforward:** vertical thrust needed = W ≈ 7799. Using main lift only,
`command_hover ≈ 7799 / 101.2 ≈ 77` (of 256). Treat as a feedforward; let the
altitude integrator trim the rest (observed hover has been higher, ~100, due to
aero damping/other losses — tune empirically). Up/down RSC is a fine vertical
trim / fast-authority channel (smaller, faster τ).

Command range per axis: **−256 … +256** (mainLift/upDown effectively 0…256 for
lift). Commands are issued at 20 Hz; the driver skips re-writing an unchanged
target.

---

## 4. Command interface

- Onboard API: `actuators:setAxisTarget(axis, rpm)` and the autopilot hook
  `actuators:apply("autopilot", { axisTargets = { mainLift=.., forwardBack=..,
  yaw=.., leftRight=.., upDown=.. } })`.
- Axis keys: `mainLift`, `forwardBack`, `yaw`, `leftRight`, `upDown`.
- Values are RPM targets, clamped to ±256 (lift 0..256).
- Control loop cadence: **20 Hz** (0.05 s), the game tick rate.

---

## 5. Sensor interface (feedback)

| signal | source / method | rate | notes |
|--------|-----------------|------|-------|
| altitude Y | `altitude_sensor_0.getHeight` | 20 Hz | absolute world Y |
| vertical speed | `altitude_sensor_0.getVerticalSpeed` | 20 Hz | |
| world velocity x,y,z | 3× `velocity_sensor_*` (`getAxis`+`getVelocity`) | 20 Hz | one per axis |
| attitude (pitch,roll) | `gimbal_sensor_0.getAnglesRad` | 20 Hz | **2 values only** (gyro-held near 0) |
| body rates p,q,r | `gimbal_sensor_0.getAngularRatesRad` | 20 Hz | |
| heading (yaw) | `navigation_table_0.getHeading` (deg) | 20 Hz | + `getOrientation` quaternion `{x,y,z,w}` available |
| position x,y,z | reconstructed (nav-table lodestone + altimeter) | **slow/laggy** | outer loop only; see §6 |
| accel / gravity | `gimbal_sensor_0.getLinearAcceleration` / `getGravity` | 20 Hz | specific force |

Position (`sensors.position = {x,y,z,valid}`) is derived from bearing/distance to
a lodestone; it updates slowly, so use it for the **outer** position loop and
lean on the fast velocity sensors for the inner rate damping.

---

## 6. Proposed control architecture (to be designed)

Three loops, all outputting RSC commands, attitude left to the gyros:

1. **Altitude hold (vertical):**
   - Input: altitude error (`y_ref − getHeight`), `verticalSpeed`.
   - Output: mainLift command = hover feedforward (`≈W/101.2`) + PID(altitude,
     vspeed). Optionally split fast corrections to `upDown` (smaller τ).
2. **Horizontal position hold (translation):**
   - Outer: world position error (x,z) → desired world velocity → desired world
     horizontal force.
   - Rotate desired world force into **body** axes by current heading:
     `F_fwd = Fx·cosψ' + Fz·sinψ'`, `F_lat = ...` (define ψ' in the §1
     convention; verify signs against `getThrustVector`).
   - Map body forces to commands via §3.3 gains → `forwardBack`, `leftRight`.
   - Inner damping on the world velocity sensors.
3. **Heading hold (yaw):**
   - Input: heading error, yaw rate `r`.
   - Output: `yaw` command. Plant: `Iyy·ψ̈ = M_yaw(command)`, `Iyy ≈ 68408`.

Each loop should include: actuator first-order lag `τ` (consider derivative/lead
or just accept the lag at these τ), output saturation (±256), integral
anti-windup, and a manual-override/kill that returns to `manual` mode.

---

## 7. What we need from the controls agent

1. Loop structure (P/PI/PID or cascade) per channel, with rationale.
2. **Starting gains** for each loop derived from the plant above (G, τ, k, m,
   Iyy), plus recommended update rates and any filtering (esp. for the slow
   position signal).
3. Anti-windup and saturation handling.
4. Any gain-scheduling / feedforward beyond the hover term.
5. Notes on expected performance and how to trim gains safely on the first
   flights.

---

## 8. Phase 2 (later): fly-to-location mission

A guidance state machine stacked on the hover controller:
`TAKEOFF` (climb to cruise altitude) → `CRUISE` (go to target X/Z, hold altitude)
→ `DESCEND` (slow commanded descent) → `CONTACT` (touchdown detect: commanded-down
but `verticalSpeed ≈ 0` and `height` flat for N ticks) → `HOVER/IDLE`. Reuses the
hover loops for setpoints; no new plant identification needed.

---

## 9. Data & references

- Sweep dataset: `s3://mc-aero-telemetry-104633066595-us-east-2/telemetry/2026/08/16/`
- Fits reproduced by `cloud/analyze_telemetry.py` (G, k, τ) and
  `cloud/plot_telemetry.py` (thrust-vs-rpm, spin-up). All 8 props fit
  `thrust=k·rpm` at R²=1.000.
- Peripheral/method reference: `tools/peripheral_dump.lua` output.
