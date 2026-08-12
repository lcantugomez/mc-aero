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

    return state
end

return Sensors
