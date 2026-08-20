# MC Aero

Flight-computer software for a Minecraft **Create: Aeronautics** VTOL, running on
**CC:Tweaked** computers. It flies the craft manually or under a full-state
autopilot, streams telemetry to AWS S3, and accepts go-to-coordinate missions
from a pocket computer.

The autopilot is a **MIMO discrete LQI** controller (gyros handle roll/pitch; the
computer commands the 5 Rotation Speed Controllers). The plant model is derived
analytically from the craft geometry + measured actuator data and the gain matrix
`K` is solved offline; the flight computer just evaluates `u = −K·x_I` at 20 Hz.

---

## Repository layout

| Path | What it is |
|------|------------|
| `aircraft/` | Flight-computer software (sensors, actuators, LQI controller, guidance, telemetry, display, startup). Installed onto the craft's computer. |
| `ground/` | Base station (`station.lua`), relay (`relay.lua`), and pocket command console (`pocket.lua`). |
| `model/` | Offline plant/gain builder (`build_plant.py`, run with `uv`) and analysis scripts. Emits `aircraft/plant_K.lua`. |
| `cloud/` | AWS ingest (Lambda + S3) deploy/teardown + telemetry analysis/plot scripts. Runs on your machine. |
| `tools/` | In-game peripheral probes (`peripheral_dump.lua`, etc.). |
| `docs/` | Design + as-built documentation (start with `docs/architecture.md`). |

---

## Install

HTTP must be enabled and `raw.githubusercontent.com` allowed in the server's
CC:Tweaked config.

```lua
wget run https://raw.githubusercontent.com/lcantugomez/mc-aero/main/install.lua aircraft   -- flight computer
wget run https://raw.githubusercontent.com/lcantugomez/mc-aero/main/install.lua ground      -- base station + relay
wget run https://raw.githubusercontent.com/lcantugomez/mc-aero/main/install.lua pocket       -- pocket command console
wget run https://raw.githubusercontent.com/lcantugomez/mc-aero/main/install.lua all          -- aircraft + ground
```

Requirements:
- The craft's computer needs the flight peripherals wired (RSCs, gyro/prop
  bearings, gimbal, altitude, velocity ×3, navigation table, physics assembler,
  two monitors) and an **ender modem** (telemetry + commands).
- A **GPS constellation** in the world (`gps.locate`) for position. A bound
  lodestone on the nav table is used as a fallback. *(GPS has a ~2000-block host
  radius — see Known limitations.)*

Run:

```lua
aircraft/startup     -- flight computer
ground/station       -- base station
ground/pocket        -- pocket console
```

Stop with **Ctrl+T**. Shutdown leaves the current main-lift target untouched.

---

## Flying it

### Manual (default)

| Key | Action |
|-----|--------|
| ↑ / ↓ | main-lift RPM trim (persistent) |
| Space | main-lift on/off toggle |
| Left Shift / Left Ctrl | up / down (up-down prop) |
| W / S | forward / back |
| A / D | left / right |
| Q / E | yaw left / right |
| **P** | engage autopilot (hover-hold at current spot) |
| **O** | override back to manual (any stick key also overrides) |

### Autopilot

Engaging (P) captures the current position/heading/altitude and holds them.
`aircraft/lqi_config.lua` gates which holds are active (`altitude`, `heading`,
`position`) — all on by default.

### Go-to missions (from the pocket)

The pocket shows live link + mission status and sends commands over the ender
modem. Keys:

| Key | Action |
|-----|--------|
| **g** | go to typed coordinates (X, Z, optional altitude, optional final heading) |
| **w** | fly to a saved waypoint |
| **s** | save a named waypoint (stored on the pocket at `/waypoints.lua`) |
| **h** | hold (hover in place) |
| **c** | cancel (stop and hover) |

A goto takes over from manual and runs the guidance sequence:
**CLIMB** to a safe cruise altitude → **ORIENT** nose-on to the target → **CRUISE**
straight in (decelerates on approach) → **ARRIVE** → **DESCEND** to the requested
altitude → **HOLD** at the point and final heading.

---

## Configuration

Override installed defaults without editing tracked files:

- **`/aircraft/user_config.lua`** — returns a table merged over `aircraft/config.lua`
  (peripherals, `position`, `mission`, telemetry, etc.).
- **`aircraft/lqi_config.lua`** — controller runtime knobs (hover feedforward,
  integral clamps, enable gates, engage/override keys).
- **`config.mission`** — mission/guidance params: `cruiseAltitude`,
  `maxGotoDistance`, `arriveRadius`, `directCruise`, altitude bounds,
  `callsign`, etc.

Ground/pocket relay config is a Lua file returning `{ endpoint, apiKey }`; copy
`ground/relay_config.example.lua` to `/relay_config.lua` (or `/pocket_config.lua`
on the pocket). Keep the real one out of version control — it holds a secret. The
pocket only relays to S3 when the endpoint is configured; otherwise it's
command-only.

---

## Regenerating the controller gain

The gain matrix lives in `aircraft/plant_K.lua`, generated from the parameters in
`model/build_plant.py` (geometry, `k`, `τ`, mass, inertia, control weights):

```bash
uv run model/build_plant.py     # prints controllability/stability, writes aircraft/plant_K.lua
```

Tune aggressiveness via the per-channel control weights (`RHO_CH`) and state
weights in that file, then reinstall the aircraft. See `docs/plant_params.md` and
`docs/architecture.md`.

---

## Telemetry → AWS

The aircraft broadcasts Rednet telemetry (`mc_aero.telemetry.v1`). A ground
station or the pocket relays batches to a Lambda Function URL that writes NDJSON
to S3 (`cloud/`). Analysis/plot scripts run locally with `uv`
(`cloud/plot_telemetry.py`, etc.).

---

## Known limitations

- **GPS range (~2000 blocks):** `gps.locate` only works within ~2000 blocks of
  the GPS host computers, and the nav-table lodestone is similarly limited. Long-
  range world positioning beyond that is an open problem (dead-reckoning from the
  body-frame velocity sensors is the likely path). Tracked in `docs/architecture.md`.
- **Provisional actuator scale/signs:** the plant uses a measured command→thrust
  scale (`CMD_GAIN`) and pulse-verified signs. If a channel pushes the wrong way,
  flip its sign and regenerate `K`; `config.mission.bearingFlip` handles a reversed
  orient.
- **Actuator-state attenuation:** the LQI's actuator states attenuate steady
  command authority, so altitude/position rely on the integral (clamps raised +
  conditional anti-windup). A 12-state rebuild (no actuator states) is the deeper
  fix if needed.

See `docs/` for the plant model, coordinate conventions, and the full as-built
architecture.
