# MC Aero

Modular CC:Tweaked flight-computer software for a Minecraft VTOL using Create Avionics/Create Aero peripherals.

## Install on the aircraft computer

HTTP must be enabled and `raw.githubusercontent.com` must be allowed by the server's CC:Tweaked configuration.

```lua
wget run https://raw.githubusercontent.com/lcantugomez/mc-aero/main/install.lua aircraft
```

After installation, start the controller with:

```lua
aircraft/startup
```

The first module runs at 10 Hz, preserves the RSC's current target on startup, proxies linked-typewriter Up/Down input to the main-lift Rotation Speed Controller, reads all connected flight sensors and actuator telemetry, updates both monitor banks, and broadcasts Rednet telemetry on protocol `mc_aero.telemetry.v1` through the right modem.

Calibration CSV logging is disabled by default. Create `/aircraft/user_config.lua` to override settings without modifying installed files:

```lua
return {
    logging = { enabled = true },
    loopPeriod = 0.05,
}
```

Gearshifts are telemetry-only until their directional roles and Redstone input sides are mapped. GPS is not required. Stop the program with Ctrl+T; shutdown deliberately leaves the current lift target unchanged.

## Install a ground station

A ground station is a base computer with an ender/wireless modem and one or more monitors. It receives the aircraft's telemetry, renders it, and can relay it to an HTTPS endpoint.

```lua
wget run https://raw.githubusercontent.com/lcantugomez/mc-aero/main/install.lua ground
```

Start it with:

```lua
ground/station
```

Configure each station with `/station_config.lua` (choose which page it shows, and whether it relays). Only one station should relay so telemetry is not uploaded twice:

```lua
return {
    pages = { "signals" }, -- flight | systems | signals | raw (one per monitor)
    relay = true,          -- set on exactly one station
}
```

On the relaying station, put the endpoint and shared secret in `/relay_config.lua`:

```lua
return {
    endpoint = "https://<your-endpoint>/",
    apiKey = "<shared-secret>",
}
```

Use `install all` to install both the aircraft and ground packages on one computer.

The `cloud/` directory holds the AWS ingest (Lambda + S3) and its deploy/teardown scripts; those run on your machine with the AWS CLI, not in-game.
