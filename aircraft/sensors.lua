local Sensors = {}
Sensors.__index = Sensors

local GIMBAL_METHODS = {
    "getAngles",
    "getAnglesRad",
    "getAngularRates",
    "getAngularRatesRad",
    "getLinearAcceleration",
    "getGravity",
}

local NAVIGATION_BASE_METHODS = {
    "getHeading",
    "getHeadingRad",
    "getOrientation",
    "hasTarget",
}

local NAVIGATION_TARGET_METHODS = {
    "getBearing",
    "getBearingRad",
    "getClosureRate",
    "getDistanceToTarget",
    "getRelativeAngle",
    "getRelativeAngleRad",
    "getTargetMetadata",
    "getTargetType",
    "getVerticalOffsetToTarget",
}

function Sensors.new(config, util)
    local self = setmetatable({ config = config, util = util, warnings = {} }, Sensors)
    local names = config.peripherals

    local checks = {
        { names.altitude, "altitude_sensor", "altitude" },
        { names.gimbal, "gimbal_sensor", "gimbal" },
        { names.navigation, "navigation_table", "navigation" },
    }
    for _, name in ipairs(names.velocity) do
        checks[#checks + 1] = { name, "velocity_sensor", name }
    end

    for _, check in ipairs(checks) do
        local ok, reason = util.checkPeripheral(check[1], check[2])
        if not ok then
            self.warnings[#self.warnings + 1] = check[3] .. ": " .. reason
        end
    end
    return self
end

function Sensors:read()
    local names, util = self.config.peripherals, self.util
    local state = { errors = {}, velocity = {} }

    local height, heightError = util.call(names.altitude, "getHeight")
    local pressure, pressureError = util.call(names.altitude, "getAirPressure")
    local verticalSpeed, verticalSpeedError = util.call(names.altitude, "getVerticalSpeed")
    state.altitude = {
        height = height,
        airPressure = pressure,
        verticalSpeed = verticalSpeed,
    }
    if heightError then state.errors["altitude.getHeight"] = heightError end
    if pressureError then state.errors["altitude.getAirPressure"] = pressureError end
    if verticalSpeedError then state.errors["altitude.getVerticalSpeed"] = verticalSpeedError end

    local gimbal, gimbalErrors = util.readMethods(names.gimbal, GIMBAL_METHODS, "gimbal")
    state.gimbal = gimbal
    util.mergeErrors(state.errors, gimbalErrors)

    for index, name in ipairs(names.velocity) do
        local axis, axisError = util.call(name, "getAxis")
        local velocity, velocityError = util.call(name, "getVelocity")
        state.velocity[index] = { name = name, axis = axis, velocity = velocity }
        if axisError then state.errors[name .. ".getAxis"] = axisError end
        if velocityError then state.errors[name .. ".getVelocity"] = velocityError end
    end

    local navigation, navigationErrors = util.readMethods(
        names.navigation,
        NAVIGATION_BASE_METHODS,
        "navigation"
    )
    if navigation.hasTarget == true then
        local targetValues, targetErrors = util.readMethods(
            names.navigation,
            NAVIGATION_TARGET_METHODS,
            "navigation"
        )
        for key, value in pairs(targetValues) do navigation[key] = value end
        util.mergeErrors(navigationErrors, targetErrors)
    end
    state.navigation = navigation
    util.mergeErrors(state.errors, navigationErrors)

    state.position = self:computePosition(navigation, state.altitude.height)

    return state
end

-- Reconstruct absolute world position from the navigation table + altimeter.
-- Calibrated convention (0deg = +Z south, 90deg = +X east, clockwise):
--   worldBearing = heading - bearing + bearingOffsetDeg
--   horiz        = sqrt(distance^2 - verticalOffset^2)   (distance is slant range)
--   x = lodestone.x - horiz * sin(worldBearing)
--   z = lodestone.z - horiz * cos(worldBearing)
--   y = altimeter height
-- The reconstructed point is the navigation table's location on the craft.
function Sensors:computePosition(navigation, height)
    local cfg = self.config.position
    if not cfg or not cfg.enabled then return nil end

    local pos = { valid = false, y = height }

    -- Primary source: GPS. ~0.6 ms/locate, returns world coords directly, and
    -- (unlike the lodestone bearing/distance reconstruction) does not break up at
    -- speed or hit the overhead pole singularity. Hold the computer point as the
    -- control position -- heading is tightly held, so the yaw orbit is negligible;
    -- add a computer->CoM offset here if a yaw-in-place drift ever shows up.
    if cfg.useGps ~= false and gps then
        local gx, gy, gz = gps.locate(cfg.gpsTimeout or 0.2)
        if gx then
            pos.x, pos.z = gx, gz
            pos.gpsY = gy
            pos.valid = true
            pos.source = "gps"
            -- Reference the CoM: subtract the computer's body-frame offset rotated
            -- into world by the fitted rotation (th = heading + offset). Forward-world
            -- = (-cos, sin); right(starboard)-world = (-sin, -cos).
            local off = cfg.computerOffset
            local heading = tonumber(navigation.getHeading)
            if off and heading then
                local th = math.rad(heading + (cfg.headingOffsetDeg or -1.5))
                local ct, st = math.cos(th), math.sin(th)
                local fwd, right = off.fwd or 0, off.right or 0
                local rx = fwd * (-ct) + right * (-st)
                local rz = fwd * (st) + right * (-ct)
                pos.comX = gx - rx
                pos.comZ = gz - rz
                pos.comY = height - (off.up or 0)
            else
                pos.comX, pos.comZ, pos.comY = gx, gz, height
            end
            return pos
        end
        pos.reason = "gps unavailable"
    end

    if navigation.hasTarget ~= true then
        pos.reason = pos.reason or "no navigation target"
        return pos
    end

    local dist = tonumber(navigation.getDistanceToTarget)
    local vOff = tonumber(navigation.getVerticalOffsetToTarget)
    local bearing = tonumber(navigation.getBearing)
    local heading = tonumber(navigation.getHeading)
    if not (dist and vOff and bearing and heading) then
        pos.reason = "incomplete navigation data"
        return pos
    end

    local squared = dist * dist - vOff * vOff
    local horiz = squared > 0 and math.sqrt(squared) or 0
    local theta = math.rad(heading - bearing + (cfg.bearingOffsetDeg or 0))

    pos.x = cfg.lodestone.x - horiz * math.sin(theta)
    pos.z = cfg.lodestone.z - horiz * math.cos(theta)
    pos.y = height
    pos.horizontal = horiz
    pos.valid = true

    -- Reference to the center of mass. r is the body-frame offset from the CoM to
    -- the nav table (config.comOffset), rotated by heading (psi; 0=+Z, 90=+X, CW).
    -- Forward -> world (sin psi, cos psi); starboard -> (cos psi, -sin psi). The
    -- CoM position is p_nav - R(psi)*r, which is immune to yaw-in-place rotation.
    local off = self.config.comOffset
    if off then
        local psi = math.rad(heading)
        local sinp, cosp = math.sin(psi), math.cos(psi)
        local fwd, right = off.fwd or 0, off.right or 0
        local rx = fwd * sinp + right * cosp
        local rz = fwd * cosp - right * sinp
        pos.comX = pos.x - rx
        pos.comZ = pos.z - rz
        pos.comY = height - (off.up or 0)
    end
    return pos
end

return Sensors
