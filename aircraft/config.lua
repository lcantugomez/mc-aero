local config = {
    version = "0.1.0",
    loopPeriod = 0.10,
    displayPeriod = 0.20,
    mode = "manual",

    manual = {
        minLiftRpm = 0,
        maxLiftRpm = 256,
        liftStep = 1,
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
        liftController = "Create_RotationSpeedController_1",
        flightMonitor = "monitor_0",
        systemMonitor = "monitor_1",
        gyroscopicBearings = {
            "gyroscopic_propeller_bearing_0",
            "gyroscopic_propeller_bearing_1",
            "gyroscopic_propeller_bearing_2",
            "gyroscopic_propeller_bearing_3",
            "gyroscopic_propeller_bearing_4",
            "gyroscopic_propeller_bearing_5",
            "gyroscopic_propeller_bearing_6",
        },
        propellerBearings = {
            "propeller_bearing_1",
        },
        gearshifts = {
            "directional_gearshift_1",
            "directional_gearshift_2",
            "directional_gearshift_3",
            "directional_gearshift_4",
            "directional_gearshift_5",
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
