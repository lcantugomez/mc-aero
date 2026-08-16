-- MC Aero autopilot controller configuration (first PID design, gyros ON).
-- Values from the controls-agent spec derived off the Aug-16 sweep.
-- Gains suffixed 100 are defined at 100% air pressure and scaled at runtime.

return {
    dt = 0.05, -- nominal control period (20 Hz)

    mass = 709,
    gravity = 11,
    referencePressure = 0.7956, -- sweep condition (fraction), fallback if sensor missing

    -- getAirPressure() returns a fraction (~0.7956). Guard against tiny values.
    pressure = { minScale = 0.20, maxScale = 2.0 },

    -- Main-lift force per command at 100% pressure (for hover feedforward).
    mainLiftForcePerCmd100 = 127.20,

    -- Mode keys read from the linked typewriter.
    keys = {
        engage = keys.p,   -- manual -> autopilot
        override = keys.o, -- autopilot -> manual (any manual stick key also drops out)
    },

    -- Stage loops per the first-flight procedure (Test A first). Enable more via
    -- a /aircraft/user_config-style override or by editing here.
    enable = {
        altitude = true,
        horizontal = false,
        heading = false,
    },

    altitude = {
        Kh = 0.30,       -- outer altitude -> vertical-speed gain (1/s)
        Kvy100 = 3.29,   -- inner vertical-speed gain at 100% pressure
        Kih = 0.05,      -- slow integral trim on altitude error
        vyCmdMax = 2.0,  -- blocks/s
        integralMin = -1500,
        integralMax = 1500,
        mainLiftMin = 40,  -- software safety limits for first flights
        mainLiftMax = 150,
    },

    horizontal = {
        Kpos = 0.30,          -- position -> velocity gain (1/s)
        velocityCmdMax = 2.0, -- blocks/s
        Kvf100 = 12.24,       -- forward/back velocity gain at 100% pressure
        Kvl100 = 44.9,        -- left/right velocity gain at 100% pressure
        forwardBackMax = 100,
        leftRightMax = 100,
    },

    heading = {
        Kpsi = 0.40,        -- heading -> yaw-rate gain (1/s)
        yawRateCmdMax = 0.50, -- rad/s
        -- PROVISIONAL: yaw moment/command (K_M) not yet verified (spec section 28).
        -- Heading hold is disabled by default (enable.heading=false); tune before use.
        KrYaw = 40,
        yawRateIndex = 2, -- getAngularRatesRad component for yaw (about +Y)
        yawRateSign = 1,  -- flip if the sign test fails
        yawMax = 75,
    },
}
