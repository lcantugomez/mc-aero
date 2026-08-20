# MC Aero — Plant Parameter Build Sheet (LQI model)

Companion to `Analytical_MIMO_LQR_LQI_Plant_Model_Plug_and_Chug.md`. This freezes
conventions and collects every numeric parameter the analytical `A`, `B` need.
Fill/verify each field, then the model build is literal plug-and-chug.

## 1. Coordinate convention (FROZEN — matches the velocity sensors)

Body frame = velocity-sensor frame, right-handed:

| axis | meaning | sign |
|---|---|---|
| x | fore/aft | **forward = −x**, aft = +x |
| y | vertical | up = +y |
| z | lateral | **left = +z**, right = −z |

- Right-handed: x × y = z (verified with x=aft, y=up, z=left).
- Yaw `ψ`, yaw rate `r` = rotation about **+y**. Yaw moment uses `(r_i × d_i)_y`.
- Velocity states map from sensor axes directly: `v_f = −v_x`, `v_l = −v_z` (right),
  `v_y = v_y`. (Or we keep the state in raw sensor axes; decide in §6.)

## 2. Global parameters

| symbol | value | source | verify |
|---|---:|---|---|
| m | 714 | physics assembler (assembled) | ✓ |
| I_ψ (Iyy) | 69116 | physics assembler I22 (assembled) | ✓ |
| P0 (nominal pressure) | 100 | — | confirm |
| Δt (control period) | 0.05 s (20 Hz) | loop | confirm |

Full inertia tensor (assembled): diag(Ixx,Iyy,Izz) = (17103, 69116, 62554),
products small (I12≈946, others <100) → near-principal, yaw uses Iyy.
CoM (sub-level, static): {20481017.6, 128.48, 20517896.5}.

## 3. Physical thrust points — COM-relative geometry (from craft, body frame)

Positions are relative to the chosen center block; **still need the CoM-vs-center-block
offset** applied (see §7). Paired entries use ±z (left = +, right = −).

d (body) read from `getThrustVector` at heading ≈89.85° where body ≈ world
(verified: FB prop reads world +X along body-x). tilt=0, blockNormal==thrustVector.

| # | role | r (x, y, z) | d (body) | k (mag) | τ | RSC channel | bearing id |
|---|---|---|---|---:|---:|---|---|
| 1 | translation left  | (0, 0, +11) | (0,0,+1) | 15.1 | 0.32 | leftRight | gyroscopic_propeller_bearing_7 |
| 2 | translation right | (0, 0, −11) | (0,0,−1) | 15.1 | 0.32 | leftRight | gyroscopic_propeller_bearing_8 |
| 3 | main lift left    | (0, 3, +11) | (0,+1,0) | 168.7 | 0.59 | mainLift | gyroscopic_propeller_bearing_1 |
| 4 | main lift right   | (0, 3, −11) | (0,+1,0) | 168.7 | 0.59 | mainLift | gyroscopic_propeller_bearing_2 |
| 5 | up/down (small)   | (0, 7, 0)   | (0,+1,0) | 42.7 | 0.44 | upDown | gyroscopic_propeller_bearing_5 |
| 6 | tail/yaw left     | (14, 0, +6) | (0,0,+1) | 15.1 | 0.32 | yaw | gyroscopic_propeller_bearing_0 |
| 7 | tail/yaw right    | (14, 0, −6) | (0,0,−1) | 15.1 | 0.32 | yaw | gyroscopic_propeller_bearing_4 |
| 8 | forward/back      | (21, 0, 0)  | (+1,0,0) | 78.4 | 0.51 | forwardBack | propeller_bearing_1 |

r_i origin is the block containing the CoM (+z = left of that block), so only the
sub-block fractional CoM offset remains — treat as ~0 for the first model, refine
if step-test residuals show a lever-arm bias.
Signs of force-per-command (handedness + gearing) are NOT fixed by d alone —
resolved by the per-channel RPM-step test (§6/§7). Note the yaw channel (6+7,
±z at x=14) produces coupled yaw + lateral force by construction; the pulse test
gives both `g_l` and `g_ψ` for that column.

## 4. Command channels (independent inputs, N = 5)

LQR can only command independent inputs. The 8 thrust points group into 5 RSCs.
For a group j: `G_F,j = Σ kᵢdᵢ`, `G_M,j = Σ kᵢ(rᵢ×dᵢ)_y`.

| channel | member points | τ | limit | hover nominal |
|---|---|---:|---:|---|
| mainLift | 3, 4 | 0.59 | 256 | `ω* = m·g / k_main(P)` |
| forwardBack | 8 | 0.51 | 256 | 0 |
| leftRight | 1, 2 | 0.32 | 256 | 0 |
| yaw | 6, 7 | 0.32 | 256 | 0 |
| upDown | 5 | 0.44 | 256 | 0 |

Plant size: n_x = 8 + 5 = 13; LQI = 13 + 4 = **17 states**.

## 5. Thrust directions d_i — READ, don't guess

Each gyro bearing exposes `getThrustVector`, `getBlockNormal`, `getFacingVector`,
`getThrustHandedness`. Plan:
1. At setup, `setManualTarget(getBlockNormal)` (or lock handedness) on every gyro so
   its output orientation is FIXED and known (doc §2.3).
2. Read `getThrustVector` per bearing → unit `d_i`. Convert to body frame using the
   craft heading at read time (or use the body-fixed block normal directly).
3. Store `d_i` in the table; recompute if the craft is rebuilt.

Sample dump (gyro_0): axis=south, blockNormal={0,0,1}, thrustVector={0,0,1},
handedness=left_handed. `{0,0,1}` is world +Z at that orientation — convert to body.

## 6. Decisions (resolved)

- **State axes: raw sensor frame** (x aft, y up, z left) is the body-frame
  foundation — that is what we measure, no extra remap. State uses `v_x, v_y, v_z`
  and `r` about +y.
- **GPS: available** via `gps.locate()`, TTL up to 2 s. Runs async (never blocks the
  loop); stale/failed fix = no correction. Estimator = velocity propagation +
  lodestone correction + async GPS correction.
- Effectiveness columns: measure per-channel accel pulses to populate, analytical
  `Σkᵢ(rᵢ×dᵢ)` as cross-check. (Confirm with controls agent.)

## 7. Still to pin

- [ ] mainLift force-per-command sign (held constant at 120 in the pulse run, not
      identifiable; assumed +up since it held the craft — confirm in a lift pulse)
- [ ] hover magnitude: model ω*_main = 23.3 but the craft held at cmd≈120 (~5x).
      Likely pressure scaling (P≈0.8) and/or RSC-speed-vs-force units — validate
      before trusting the hover feedforward (LQI integral will absorb steady offset)
- [x] per-channel signs from pulse run (2026/08/20): forwardBack +cmd→forward(−x);
      leftRight +cmd→right(−z); upDown +cmd→up; yaw G_M negative (yaw rate = gimbal
      angular-rate INDEX 1, about +y). mainLift assumed +.
- [x] `d_i` unit vectors (body frame) — from assembled dump (§3)
- [x] m = 714, I_ψ = 69116 (assembled)
- [x] r_i origin = CoM block (+z left); sub-block CoM offset ~0 for first model
- [x] bearing IDs (from config.lua bearings list)
- [x] GPS available (gps.locate, TTL 2 s, async)
- [x] body frame = raw velocity-sensor axes
- [x] command limits 256; sign conventions resolved above
