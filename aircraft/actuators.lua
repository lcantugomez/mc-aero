local Actuators = {}
Actuators.__index = Actuators

local RSC_METHODS = {
    "getTargetSpeed",
    "getSpeed",
    "hasSource",
    "isOverstressed",
    "getStressContribution",
    "getStressImpact",
}

local BEARING_STATIC_METHODS = {
    "getKind",
    "getAxis",
    "getFacingVector",
    "getSailPower",
    "getThrustHandedness",
    "isWoodenTop",
}

local BEARING_DYNAMIC_METHODS = {
    "getThrust",
    "getThrustVector",
    "getAirflow",
    "getAngularSpeed",
    "getRotationSpeed",
    "getSpeed",
    "isActive",
    "isAssembled",
    "isOverstressed",
}

local GYRO_DYNAMIC_METHODS = {
    "getTiltAngle",
    "getStabilizationStrength",
    "getManualTarget",
}

local GEARSHIFT_METHODS = {
    "getMode",
    "getSourceAxis",
    "getSpeed",
    "isLeftPowered",
    "isRightPowered",
    "hasSource",
    "isOverstressed",
}

local function append(destination, source)
    for _, value in ipairs(source) do destination[#destination + 1] = value end
end

function Actuators.new(config, util)
    local self = setmetatable({
        config = config,
        util = util,
        warnings = {},
        bearingStatic = {},
        targetLiftRpm = 0,
    }, Actuators)
    local names = config.peripherals

    local checks = {
        { names.liftController, "Create_RotationSpeedController", "lift controller" },
        { names.typewriter, "linked_typewriter", "typewriter" },
    }
    for _, name in ipairs(names.gyroscopicBearings) do
        checks[#checks + 1] = { name, "gyroscopic_propeller_bearing", name }
    end
    for _, name in ipairs(names.propellerBearings) do
        checks[#checks + 1] = { name, "propeller_bearing", name }
    end
    for _, name in ipairs(names.gearshifts) do
        checks[#checks + 1] = { name, "directional_gearshift", name }
    end
    for _, check in ipairs(checks) do
        local ok, reason = util.checkPeripheral(check[1], check[2])
        if not ok then self.warnings[#self.warnings + 1] = check[3] .. ": " .. reason end
    end

    local currentTarget, targetError = util.call(names.liftController, "getTargetSpeed")
    if targetError then
        self.warnings[#self.warnings + 1] = "lift target: " .. targetError
    elseif type(currentTarget) == "number" then
        self.targetLiftRpm = util.clamp(
            currentTarget,
            config.manual.minLiftRpm,
            config.manual.maxLiftRpm
        )
    end

    local allBearings = {}
    for _, name in ipairs(names.gyroscopicBearings) do
        allBearings[#allBearings + 1] = { name = name, gyroscopic = true }
    end
    for _, name in ipairs(names.propellerBearings) do
        allBearings[#allBearings + 1] = { name = name, gyroscopic = false }
    end
    self.bearings = allBearings

    for _, bearing in ipairs(allBearings) do
        local static, errors = util.readMethods(bearing.name, BEARING_STATIC_METHODS, bearing.name)
        static.name = bearing.name
        static.gyroscopic = bearing.gyroscopic
        self.bearingStatic[bearing.name] = static
        for key, value in pairs(errors) do
            self.warnings[#self.warnings + 1] = key .. ": " .. value
        end
    end

    return self
end

function Actuators:setLiftTarget(value)
    local manual = self.config.manual
    local target = self.util.clamp(tonumber(value) or 0, manual.minLiftRpm, manual.maxLiftRpm)
    local _, commandError = self.util.call(
        self.config.peripherals.liftController,
        "setTargetSpeed",
        target
    )
    if commandError then return false, commandError end
    self.targetLiftRpm = target
    return true, nil
end

function Actuators:updateManual()
    local pressed, inputError = self.util.call(
        self.config.peripherals.typewriter,
        "getPressedKeyCodes"
    )
    local input = { pressed = pressed or {}, delta = 0 }
    if inputError then input.error = inputError end

    if type(pressed) == "number" then pressed = { pressed } end
    if type(pressed) == "table" then
        for _, keyCode in pairs(pressed) do
            if keyCode == keys.up then input.delta = input.delta + self.config.manual.liftStep end
            if keyCode == keys.down then input.delta = input.delta - self.config.manual.liftStep end
        end
    end

    local target = self.targetLiftRpm + input.delta
    local ok, commandError = self:setLiftTarget(target)
    input.targetLiftRpm = self.targetLiftRpm
    if not ok then input.commandError = commandError end
    return input
end

function Actuators:apply(mode, command)
    if mode == "manual" then
        return self:updateManual()
    elseif mode == "autopilot" then
        if not command or type(command.liftRpm) ~= "number" then
            return {
                targetLiftRpm = self.targetLiftRpm,
                commandError = "autopilot lift command is missing",
                applied = false,
            }
        end
        local ok, commandError = self:setLiftTarget(command.liftRpm)
        return { targetLiftRpm = self.targetLiftRpm, commandError = commandError, applied = ok }
    end
    return { targetLiftRpm = self.targetLiftRpm, commandError = "unsupported mode: " .. tostring(mode) }
end

function Actuators:read()
    local util, names = self.util, self.config.peripherals
    local state = { errors = {}, bearings = {}, gearshifts = {} }

    local liftController, liftErrors = util.readMethods(
        names.liftController,
        RSC_METHODS,
        "liftController"
    )
    state.liftController = liftController
    state.liftController.commandedSpeed = self.targetLiftRpm
    util.mergeErrors(state.errors, liftErrors)

    for index, bearing in ipairs(self.bearings) do
        local methods = {}
        append(methods, BEARING_DYNAMIC_METHODS)
        if bearing.gyroscopic then append(methods, GYRO_DYNAMIC_METHODS) end
        local dynamic, errors = util.readMethods(bearing.name, methods, bearing.name)
        local result = {}
        for key, value in pairs(self.bearingStatic[bearing.name] or {}) do result[key] = value end
        for key, value in pairs(dynamic) do result[key] = value end
        state.bearings[index] = result
        util.mergeErrors(state.errors, errors)
    end

    for index, name in ipairs(names.gearshifts) do
        local values, errors = util.readMethods(name, GEARSHIFT_METHODS, name)
        values.name = name
        state.gearshifts[index] = values
        util.mergeErrors(state.errors, errors)
    end

    return state
end

return Actuators
