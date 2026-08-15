# MC Aero — Peripheral Outputs, Discoveries, and State-Space Handoff

Purpose: give a control-design agent everything known about the vehicle's
sensors, actuators, measured dynamics, and coordinate conventions so it can
formulate the state space and design the full controller. Items are tagged:

- **[M]** measured/confirmed empirically
- **[A]** assumed / from mod behavior, not yet fully verified
- **[TODO]** data still needed before the model is complete

---

## 1. Vehicle & software overview

- Minecraft VTOL built in Create Aeronautics, controlled by a CC:Tweaked
  computer running a 10 Hz Lua loop (`aircraft/startup.lua`).
- The flight computer reads every sensor/actuator each tick, broadcasts a full
  state snapshot over rednet (`mc_aero.telemetry.v1`), and a ground station
  relays it to AWS S3 (NDJSON) for offline analysis.
- **[M]** Loop rate: 10 Hz (`loopPeriod = 0.10`). Telemetry 10 Hz. Display 5 Hz.
- **[M]** The only actuator currently *commanded* by software is the main-lift
  Rotation Speed Controller (RSC) via `setTargetSpeed(rpm)`. Everything else
  (directional gearshifts, gyro tilt) is currently **read-only telemetry** and
  must be wired before it can be used as a control input (see §6, §8).

---

## 2. Coordinate frames & conventions  **[M]**

World axes (Minecraft):

| axis | + direction | − direction |
|------|-------------|-------------|
| X    | East        | West        |
| Y    | Up          | Down        |
| Z    | South       | North       |

Bearing/heading convention used throughout (and in the position transform):
`0° = +Z (south)`, `90° = +X (east)`, increasing **clockwise** viewed from above.

Angles from the navigation table (`getBearing`, `getHeading`) are in **degrees**.
Distances are in **blocks**. Rotational speed is in **RPM**. Thrust is in the
game's force units (user calls them "pixel-newtons", pN).

---

## 3. Telemetry snapshot schema

Broadcast each tick (`aircraft/startup.lua`), one JSON object per record in S3:

```
schema        string   "mc_aero.telemetry.v1"
version       string
timestampMs   number   os.epoch("utc") on the aircraft
computerId    number
sequence      number   increments per loop; resets to 0 on reboot
mode          string   "manual" (only mode wired today)
loopDurationMs number  time to sample inputs+sensors+actuators (excludes net/display/log)
manualInput   { pressed[], delta, targetLiftRpm, error?, commandError? }
sensors       { altitude, gimbal, velocity[], navigation, position, errors{} }
actuators     { liftController, bearings[], gearshifts[], errors{} }
errors        { <key>: <string> }   keyed per-method read failures
errorCount    number
telemetryError string?
```

---

## 4. Sensor peripherals & outputs

### 4.1 Altitude sensor (`altitude_sensor_0`)  **[M]**
| method | meaning | units |
|--------|---------|-------|
| `getHeight` | absolute world Y | blocks |
| `getVerticalSpeed` | vertical velocity | blocks/s |
| `getAirPressure` | air pressure (altitude proxy) | — |

Used as the authoritative **Y** source (GPS Y and nav vertical offset are unreliable).

### 4.2 Gimbal / IMU (`gimbal_sensor_0`)  **[M]**
| method | meaning |
|--------|---------|
| `getAngles` / `getAnglesRad` | attitude (pitch/roll/yaw) deg / rad |
| `getAngularRates` / `getAngularRatesRad` | body angular rates deg/s / rad/s |
| `getLinearAcceleration` | linear acceleration vector |
| `getGravity` | gravity vector |

This is the primary **attitude + rate** source for the inner loop. **[TODO]**
confirm exact component order/frame of `getAngles` (which element is pitch vs
roll vs yaw, body vs world).

### 4.3 Velocity sensors (`velocity_sensor_0..2`)  **[M]**
Each returns `getAxis` (which world axis it measures) and `getVelocity`
(blocks/s on that axis). Three sensors → full linear velocity vector. Observed
range in a maneuvering run: ±6–7 blocks/s on all axes.

### 4.4 Navigation table (`navigation_table_0`)  **[M]**
Driven by a **Lodestone Compass** bound to a lodestone at a known world position.
| method | meaning | notes |
|--------|---------|-------|
| `getHeading` / `getHeadingRad` | craft heading | deg / rad |
| `getOrientation` | orientation | [A] |
| `hasTarget` | lodestone bound? | gates the target methods below |
| `getBearing` / `getBearingRad` | bearing to lodestone | **relative to craft heading** [M] |
| `getDistanceToTarget` | **slant (3D)** range to lodestone | blocks [M] |
| `getVerticalOffsetToTarget` | `lodestoneY − craftY` | blocks [M] |
| `getRelativeAngle(Rad)`, `getClosureRate`, `getTargetType`, `getTargetMetadata` | — | [A] |

**Update rate is slow / laggy** [M] — good enough for a slow outer-loop position
hold, but the fast inner loop should lean on velocity + gimbal, not position.

---

## 5. Position reconstruction (implemented)  **[M]**

Absolute world position is reconstructed from the nav table + altimeter, no GPS
constellation needed at runtime (`aircraft/sensors.lua : computePosition`):

```
horiz = sqrt(distance² − verticalOffset²)        -- distance is slant range
θ     = heading − bearing + bearingOffsetDeg     -- bearingOffsetDeg ≈ 0 (calibrated)
x     = lodestoneX − horiz · sin(θ)
z     = lodestoneZ − horiz · cos(θ)
y     = altimeter getHeight
```

Calibration evidence (two hover points at different headings):

| heading | residual `heading−bearing` vs true bearing |
|---------|--------------------------------------------|
| 82.0°   | ≈ −3° |
| 169.8°  | ≈ +1° |

The residual stayed ~0 across an 88° heading change while the "world-only" and
"heading+bearing" hypotheses swung 50–140°, confirming `worldBearing = heading −
bearing`. Reconstructed point is the **navigation table block's** location (~2–3
blocks from the modem/GPS point); that fixed body offset cancels in motion.

GPS note **[M]**: a CC GPS constellation gives correct X/Z but **wrong Y**; it is
not used at runtime (kept only as a calibration reference).

---

## 6. Actuator peripherals & outputs

### 6.1 Main lift — Rotation Speed Controller (`Create_RotationSpeedController_1`)  **[M]**
| method | meaning |
|--------|---------|
| `setTargetSpeed(rpm)` | **command** main-lift rotation speed (the one live control input) |
| `getTargetSpeed` | current target |
| `getSpeed` | actual RSC shaft speed |
| `hasSource`, `isOverstressed`, `getStressContribution`, `getStressImpact` | drivetrain/stress state |

Observed: `commandedSpeed == getTargetSpeed == getSpeed` in steady flight (the
RSC holds its target). Software preserves the existing target on startup and
does not zero it on shutdown.

### 6.2 Propeller bearings (6 gyroscopic + 2 plain)
Names: `gyroscopic_propeller_bearing_0..5`, `propeller_bearing_0..1`.

Static (constant, read once) **[A→M geometry source]**:
`getKind`, `getAxis`, `getFacingVector`, `getSailPower`, `getThrustHandedness`,
`isWoodenTop`. These define each prop's **thrust direction and geometry** and are
the basis for control allocation (§7).

Dynamic: `getThrust`, `getThrustVector`, `getAirflow`, `getAngularSpeed`,
`getRotationSpeed`, `getSpeed`, `isActive`, `isAssembled`, `isOverstressed`.
Gyro-only dynamic: `getTiltAngle`, `getStabilizationStrength`, `getManualTarget`
— **[A]** these imply a commandable tilt/attitude input on the gyro bearings that
has **not been exercised yet** (see §8).

### 6.3 Directional gearshifts (`directional_gearshift_1..5`)  **[M, read-only]**
`getMode`, `getSourceAxis`, `getSpeed`, `isLeftPowered`, `isRightPowered`,
`hasSource`, `isOverstressed`. In a maneuvering run all five were exercised at a
**fixed speed** (~±239–242 RPM). Their **directional roles and Redstone control
inputs are not yet mapped**, so they are currently telemetry-only.

### 6.4 Pilot input — linked typewriter (`linked_typewriter_0`)  **[M]**
`getPressedKeyCodes` → Up/Down step the RSC target by `liftStep` (1 RPM).

---

## 7. Measured dynamics & control-relevant discoveries

### 7.1 Thrust is exactly linear in rotation speed  **[M]**
Per-bearing least-squares fit `thrust = k · rpm` over a full maneuvering run:
**intercept ≈ 0, R² = 1.000** for every bearing (it is the game's deterministic
thrust law, not noisy data). `k` is the prop's thrust coefficient (pN/RPM):

| bearing | k (pN/RPM) | group |
|---------|-----------|-------|
| gyro_0 | −15.1 | small props |
| gyro_3 | +15.1 | small props |
| gyro_4 | −15.1 | small props |
| prop_0 | +15.1 | small props |
| gyro_5 | +42.7 | |
| prop_1 | −78.4 | |
| gyro_1 | −168.7 | large props (mirror pair) |
| gyro_2 | +168.7 | large props (mirror pair) |

Sign = thrust direction relative to rpm sign (mounting/handedness). Magnitude
scales with prop size/shape (different loads). Large props make ~5× the thrust
at <½ the rpm of the small ones.

### 7.2 Rotation speed has first-order spin-up dynamics  **[M, partially]**
- Thrust responds to *instantaneous* rpm with no lag (§7.1). The lag is in the
  rpm itself: bearing rpm approaches a new steady value along a **first-order,
  overdamped (no overshoot)** curve after a command step.
- Model: `omegȧ = (1/τ)(G · omega_cmd − omega)`, thrust `= k · omega`.
- **[M]** Steady-state gain example: RSC target 107 → bearing 32.1 RPM ≈ **0.30**.
  May be partly stress/power limited (the lift magnet was undertrimmed, feeding
  ~240 vs 256). **[TODO]** τ and a clean gain curve (see §8).
- **[M]** Main-lift saturates around ~3000 pN total at the top of its range.

### 7.3 Gearshifts are fixed-speed  **[M]**
Directional gearshifts run at a constant ~±240 RPM when engaged (bang-bang),
not variable. Directional authority is therefore likely on/off per axis until a
finer scheme is built.

---

## 8. Proposed state space (for the control agent)

Rigid-body 6-DOF, 12 states:

```
x = [ px py pz   vx vy vz   φ θ ψ   p q r ]ᵀ
     position    velocity   attitude  body rates
```

Measurements available (with rough bandwidth):
| signal | source | rate/quality |
|--------|--------|--------------|
| position px,py,pz | nav table + altimeter (§5) | **slow/laggy**, outer loop only |
| velocity vx,vy,vz | velocity sensors | 10 Hz, good |
| attitude φ,θ,ψ | gimbal `getAngles` | 10 Hz |
| body rates p,q,r | gimbal `getAngularRates` | 10 Hz |
| vertical speed | altimeter | 10 Hz (redundant w/ vz) |
| heading ψ | nav table `getHeading` | 10 Hz |

Inputs:
| input | status | model |
|-------|--------|-------|
| main-lift RSC target (scalar) | **live** (`setTargetSpeed`) | u → first-order rpm (τ, G) → thrust = Σ kᵢ·ωᵢ, vertical |
| directional gearshifts ×5 | **read-only, must be wired** | fixed ±240 RPM, on/off per axis [TODO roles] |
| gyro bearing tilt / manual target | **read-only, must be wired** | `getManualTarget`/`getTiltAngle` suggest attitude authority [TODO] |

Actuator → wrench (control allocation):
- Per bearing i: force magnitude `Fᵢ = kᵢ · ωᵢ`, direction from static
  `getFacingVector` × `getThrustHandedness`.
- Net force `Σ Fᵢ·d̂ᵢ`; net moment `Σ rᵢ × (Fᵢ·d̂ᵢ)`.
- **[TODO]** bearing positions `rᵢ` (lever arms) and confirmed direction frame
  are required to build the allocation matrix — not yet logged.

Recommended loop structure: fast inner attitude/rate loop on gimbal signals;
slow outer position/altitude loop on the laggy nav-table position + altimeter.

---

## 9. Open unknowns / tests still required  **[TODO]**

1. **RSC step response** — hover, step `setTargetSpeed` by a few RPM, capture
   bearing rpm slew. Gives τ (63.2% rise time) and the command→rpm gain G, and
   whether it is stress-limited (watch `isOverstressed`, `getStressImpact`).
2. **Gearshift mapping** — for each of the 5 directional gearshifts, apply its
   Redstone input and record which axis/moment it produces (rpm, thrust,
   resulting velocity/rate). Establishes directional control authority.
3. **Gyro manual target** — write `getManualTarget`/tilt and measure the
   resulting `getTiltAngle` and attitude effect; quantify attitude authority.
4. **Geometry for allocation** — log static bearing fields (`getAxis`,
   `getFacingVector`, `getThrustHandedness`, `getSailPower`) and the physical
   lever arms `rᵢ` of each bearing.
5. **Mass / inertia ID** — combine measured thrust with `getLinearAcceleration`
   and `getAngularRates` to estimate mass and moments of inertia.
6. **Attitude frame confirmation** — component order and body-vs-world frame of
   `getAngles` / `getFacingVector`.

---

## 10. Data & tooling

- Telemetry lands in S3: `s3://mc-aero-telemetry-104633066595-us-east-2/telemetry/YYYY/MM/DD/*.ndjson`
  (region us-east-2). Each object is one ~1 s batch of 10 Hz snapshots.
- Pull + analyze:
  ```bash
  aws s3 sync s3://mc-aero-telemetry-.../telemetry/ /tmp/mcaero_data --profile luis.cantugomez --region us-east-2
  uv run --with matplotlib --with numpy python cloud/plot_telemetry.py /tmp/mcaero_data analysis
  ```
  `cloud/analyze_telemetry.py` prints ranges + linear fits; `cloud/plot_telemetry.py`
  writes RSC, motion, velocity, bearing-rpm, and thrust-vs-rpm plots.
- Live inspection of any peripheral: `tools/nav_probe.lua`.
