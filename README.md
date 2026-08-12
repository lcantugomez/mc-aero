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
