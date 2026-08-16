# MC Aero — Mission-Mode Flight Controller Design

Purpose: replace the generic "hold this random spot" autopilot with a
purpose-built, mode-based controller for the real mission
(takeoff → orient → cruise/go-to → descend → hover/land), on top of the
measured plant and the gyros-on inner loops. This supersedes the position-hold
tuning; hold is just one mode.

---

## 1. Measurement frames & the apparent yaw/translation coupling

**Two distinct reference points — do not conflate them.** All of the "world
coordinates" the flight computer reads come from the nav/gimbal sensor, which is
NOT at the center of mass:

- `p_nav` — world position we actually read (nav-table reconstruction / gimbal).
  At the sensor/computer, offset from the CoM by a body-fixed lever arm.
- `p_com` — `getCenterOfMass()`, the true center of rotation.
- `v_nav` — velocity we read, also at the sensor.

Rigid-body kinematics relate them:
```
p_nav = p_com + R(heading) · r_body          -- r_body is constant in body frame
r_world = p_nav - p_com                        -- lever arm in world frame
v_nav   = v_com + omega x r_world              -- omega = yaw rate
v_com   = v_nav - omega x r_world
```

**Leading hypothesis: the coupling is a kinematics artifact, not a force.**
Commanding the yaw RSC alone showed ~1.7 b/s "translation" in the Aug-16 data,
but v/omega was ~constant (~25 blocks) — that is a lever arm (the omega x r term),
not a force signature. So pure rotation about the CoM is being *read* as
translation because the sensor is offset. The heading↔position "fight-loop"
(heading correction appears to shove the craft sideways, position loop fights it)
is then explained by the position loop chasing this phantom, sensor-offset motion,
not by an actuator side force.

**Primary fix: reference all horizontal feedback to the CoM.**
- Position feedback = `p_com` (its large sub-level offset cancels in position
  *error*; immune to the rotation artifact; likely less laggy than the nav
  reconstruction).
- Velocity feedback = `v_com = v_nav - omega x r_world`, or the time derivative
  of `p_com`.
- Hold the **CoM** at the setpoint, not the sensor. Holding the sensor position
  forces the CoM to orbit whenever heading changes — exactly the observed fight.
- Bonus: logging both `p_nav` and `p_com` yields `r_world = p_nav - p_com` every
  tick for free (no geometry guessing); its ~constant magnitude + rotation with
  heading is the validation that this is the artifact.

TODO (blocking confirmation): `getCenterOfMass` is currently read ONCE at startup,
so it is static in telemetry. Log it per tick, re-run a yaw-only test, and confirm
`p_com` stays fixed during pure yaw. If it does, the item below is unnecessary.

**Fallback (only if a real residual force survives CoM-referencing):** treat loops
as outputting a body wrench `[F_fwd, F_lat, F_up, M_yaw]` and add an `allocate()`
step that encodes the same-side yaw+translation pairing (Q+A / E+D) so a pure
`M_yaw` demand cancels the side force. Build it empirically (one yaw-only pulse →
induced CoM acceleration = force per unit yaw command) or analytically (prop thrust
directions from the dump + `k·G` + lever arms about CoM). Only pursue this if the
per-tick CoM test shows the CoM actually translates during pure yaw.

---

## 2. Flight philosophy / sequencing

- **No obstacle detection.** The only "known clear" direction is straight up, so
  the craft climbs first to clear, then moves. (Optionally wiggle/climb until a
  vertical path is clear — future.)
- **Heading first, then translate.** Orient toward the destination bearing, then
  move laterally while holding that heading.
- **Arrive gracefully.** Decelerate on approach so it stops on the target, never
  races past (the old proportional loop saturated velocity and overshot).
- **Descend, then hover or land.** Land if possible; for now hover at a set block
  height above the target.

---

## 3. Mode state machine

```
MANUAL ──(engage)──▶ TAKEOFF ──▶ ORIENT ──▶ GOTO ──▶ ARRIVE ──▶ DESCEND ──▶ HOVER/LAND
   ▲                                                                            │
   └──────────────────── manual override (any stick key / override) ◀──────────┘
```

Per-mode intent and setpoints (all share the inner loops + allocator):

| mode | vertical | horizontal | heading |
|------|----------|------------|---------|
| TAKEOFF | exponential spool of main lift up to hover(weight) thrust, then climb at `vyMax` to cruise altitude | hold launch X/Z | hold current |
| ORIENT | hold cruise altitude | hold position | slew heading to bearing-to-target (pure rotation) |
| GOTO/CRUISE | hold cruise altitude | decel-limited velocity toward target X/Z | hold bearing-to-target |
| ARRIVE | hold cruise altitude | decelerate to zero at target | hold |
| DESCEND | controlled descent to target/contact altitude | hold target X/Z | hold |
| HOVER | hold set altitude | hold X/Z | hold |
| LAND | descend until contact, then idle; else hover at set height | hold X/Z | hold |

---

## 4. Guidance laws

- **Takeoff spool:** `u_main → u_hover` with a first-order ramp (time constant
  ~1–2 s) so thrust builds exponentially to weight, then hand off to the altitude
  loop climbing at `vyCmdMax` toward cruise altitude.
- **Decel-limited approach (GOTO/ARRIVE):** command speed toward the target as
  `v_cmd = min(v_cruise, sqrt(2 · a_decel · distance))`, so it cruises at
  `v_cruise` then eases to ~0 exactly at the target. This is the "slow down
  gracefully" behavior; tune `a_decel`.
- **Orient/heading:** slew heading toward `atan2` bearing to target, using the
  wrench allocator so rotation stays pure (no translation kick). Hold that
  heading through cruise.
- **Contact detect (LAND):** commanded-down but `verticalSpeed ≈ 0` and `height`
  flat for N ticks → touchdown → idle. If landing disabled, hold `HOVER` at a
  configured block height.

---

## 5. Inner loops (already characterized / tuned)

- **Vertical:** altitude PID + pressure-scheduled hover feedforward — proven
  (0.02-block steady state). Reused by every mode's altitude setpoint.
- **Horizontal velocity:** clean (fast velocity sensors). Position becomes a
  decel-limited *guidance* layer feeding this, not a saturating proportional loop.
- **Heading:** heading → yaw-rate → `M_yaw`, realized through the allocator.
- All gains pressure-scheduled (`gainScale = 1/pressure`).

---

## 6. Open items to finalize

- Allocation: pick empirical (yaw-pulse) or analytical (needs prop positions).
- Per-mode parameters: cruise altitude, `v_cruise`, `a_decel`, arrival radius,
  takeoff spool time constant, hover block height, contact-detect thresholds.
- Confirm sign/gain of the heading loop once yaw runs through the allocator.

---

## 7. Definition of done

From a stop on the ground the craft can, in one engage: spool up and take off to
cruise altitude, orient to the destination, cruise to the waypoint and arrive
gracefully (no overshoot), descend, and hover (or land), holding heading
throughout, with manual override available at any instant.
