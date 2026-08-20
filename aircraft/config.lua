local config = {
    version = "0.1.0",
    loopPeriod = 0.05,   -- control/input rate (20 Hz = one game tick, minimum latency)
    displayPeriod = 0.20,
    mode = "manual",

    manual = {
        minLiftRpm = 0,
        maxLiftRpm = 256,
        liftStep = 4,   -- main-lift RPM change per tick while Up/Down held
        moveRpm = 256,  -- RPM applied to a directional axis while its key is held

        -- Typewriter key bindings (CC key codes). Main lift is a persistent
        -- stepped target (holds when released, for hover); the other axes are
        -- momentary (return to 0 when released). Swap a pair to invert an axis.
        keys = {
            liftUp = keys.up,        liftDown = keys.down,     -- main lift RPM trim
            mainLiftToggle = keys.space,                       -- main lift on/off
            ascend = keys.leftShift, descend = keys.leftCtrl,  -- upDown RSC (up / down)
            forward = keys.w,        back = keys.s,            -- forwardBack RSC
            right = keys.d,          left = keys.a,            -- leftRight RSC
            yawRight = keys.e,       yawLeft = keys.q,         -- yaw RSC
        },
    },

    telemetry = {
        enabled = true,
        modemSide = "right",
        protocol = "mc_aero.telemetry.v1",
        period = 0.10,
    },

    logging = {
        enabled = false,
        directory = "/aircraft/logs",
        period = 0.10,
    },

    display = {
        textScale = 0.5,
    },

    -- Absolute position from the navigation table's lodestone + altimeter.
    -- X/Z are reconstructed from bearing/distance to the bound lodestone;
    -- Y comes from the altimeter (nav vertical offset / GPS Y are unreliable).
    -- No GPS constellation needed at runtime, so nothing blocks the loop.
    position = {
        enabled = true,
        lodestone = { x = -30, z = 1046 }, -- world X/Z of the bound lodestone
        -- worldBearing = heading - bearing + bearingOffsetDeg (calibrated ~0)
        bearingOffsetDeg = 0,
    },

    -- Offset used to rotate the nav-table world position into a CoM-referenced
    -- position (comX/comZ) that stays fixed during a yaw-in-place.
    -- fwd/right are the EMPIRICAL fit from the Aug-16 pure-yaw run (circle center
    -- -9.93, 1059.2; radius ~11 blocks), verified by the position-vs-time plot.
    -- They sit ~90 deg rotated from the physically-measured 9 fwd / 1 right: the
    -- nav table's getHeading zero is not aligned with the body-forward axis the
    -- transform assumes, so these numbers absorb that rotation. up is the physical
    -- vertical offset (nav table ~1 block below the CoM).
    comOffset = { fwd = -0.76, right = -10.83, up = -1 },

    -- Go-to-coordinate mission link + guidance. Requests arrive over rednet
    -- (ender modem, unlimited range) on commandProtocol. The guidance state
    -- machine (aircraft/guidance.lua) turns a goal into a moving setpoint fed to
    -- the LQI: climb to a safe height, orient nose-on, cruise straight in
    -- decel-limited, then descend onto the target. One K throughout.
    mission = {
        commandProtocol = "mc_aero.command.v1",
        cruiseAltitude = 190,      -- safe travel height; climbs here before moving
        maxGotoDistance = 10000,   -- reject a goto farther than this (blocks)
        vCruise = 8,               -- setpoint cruise speed (blocks/s)
        aDecel = 1.0,              -- arrival deceleration (blocks/s^2)
        maxLead = 5,               -- cap how far the moving setpoint leads the craft
        arriveRadius = 3,          -- cruise -> arrive within this of the goal (blocks)
        orientToleranceDeg = 8,    -- heading-aligned threshold to leave ORIENT
        altTolerance = 1.5,        -- altitude-reached threshold (blocks)
        altitudeFloor = 60,        -- reject goto below this
        altitudeCeiling = 250,     -- reject goto above this
        headingOffsetDeg = -1.5,   -- must match lqi_config (fitted world->body offset)
        bearingFlip = false,       -- set true if ORIENT faces away from the goal
    },

    peripherals = {
        altitude = "altitude_sensor_0",
        gimbal = "gimbal_sensor_0",
        navigation = "navigation_table_0",
        velocity = {
            "velocity_sensor_0",
            "velocity_sensor_1",
            "velocity_sensor_2",
        },
        typewriter = "linked_typewriter_0",
        flightMonitor = "monitor_0",
        systemMonitor = "monitor_1",
        physicsAssembler = "physics_assembler_0",

        -- Rotation Speed Controllers, one continuously-variable RSC per axis.
        rsc = {
            mainLift    = "Create_RotationSpeedController_1", -- gyro_1 + gyro_2 lift pair
            forwardBack = "Create_RotationSpeedController_2", -- propeller_bearing_1
            yaw         = "Create_RotationSpeedController_3", -- gyro_0 + gyro_4
            leftRight   = "Create_RotationSpeedController_4", -- gyro_7 + gyro_8
            upDown      = "Create_RotationSpeedController_5", -- gyro_5
        },

        -- Propeller bearings with their control role. gyro = true means it also
        -- exposes tilt/stabilization (setManualTarget/getTiltAngle).
        bearings = {
            { name = "gyroscopic_propeller_bearing_1", role = "main_lift_left",    gyro = true },
            { name = "gyroscopic_propeller_bearing_2", role = "main_lift_right",   gyro = true },
            { name = "gyroscopic_propeller_bearing_5", role = "up_down",           gyro = true },
            { name = "gyroscopic_propeller_bearing_8", role = "translation_right", gyro = true },
            { name = "gyroscopic_propeller_bearing_7", role = "translation_left",  gyro = true },
            { name = "gyroscopic_propeller_bearing_0", role = "yaw_left",          gyro = true },
            { name = "gyroscopic_propeller_bearing_4", role = "yaw_right",         gyro = true },
            { name = "propeller_bearing_1",            role = "forward_back",      gyro = false },
        },
    },
}

local function merge(destination, source)
    for key, value in pairs(source) do
        if type(value) == "table" and type(destination[key]) == "table" then
            merge(destination[key], value)
        else
            destination[key] = value
        end
    end
end

local overridePath = "/aircraft/user_config.lua"
if fs.exists(overridePath) then
    local ok, overrides = pcall(dofile, overridePath)
    if not ok then
        error("Could not load " .. overridePath .. ": " .. tostring(overrides), 0)
    end
    if type(overrides) ~= "table" then
        error(overridePath .. " must return a table", 0)
    end
    merge(config, overrides)
end

return config
