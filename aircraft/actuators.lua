local Actuators = {}
Actuators.__index = Actuators

-- One RSC per control axis: mainLift, forwardBack, yaw, leftRight, upDown.
local AXIS_ORDER = { "mainLift", "forwardBack", "yaw", "leftRight", "upDown" }

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
    "getBlockNormal",
    "getSailPower",
    "getThrustHandedness",
    "isWoodenTop",
}

local BEARING_DYNAMIC_METHODS = {
    "getThrust",
    "getThrustVector",
    "getAirflow",
    "getAngle",
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

local function append(destination, source)
    for _, value in ipairs(source) do destination[#destination + 1] = value end
end

function Actuators.new(config, util)
    local self = setmetatable({
        config = config,
        util = util,
        warnings = {},
        bearingStatic = {},
        commanded = {},
        targetLiftRpm = 0,
    }, Actuators)
    local names = config.peripherals
    self.rsc = names.rsc
    self.bearings = names.bearings

    for _, axis in ipairs(AXIS_ORDER) do
        local name = names.rsc[axis]
        if name then
            local ok, reason = util.checkPeripheral(name, "Create_RotationSpeedController")
            if not ok then self.warnings[#self.warnings + 1] = axis .. " rsc: " .. reason end
        else
            self.warnings[#self.warnings + 1] = axis .. " rsc: not configured"
        end
    end

    local ok, reason = util.checkPeripheral(names.typewriter, "linked_typewriter")
    if not ok then self.warnings[#self.warnings + 1] = "typewriter: " .. reason end

    for _, bearing in ipairs(names.bearings) do
        local expected = bearing.gyro and "gyroscopic_propeller_bearing" or "propeller_bearing"
        local checkOk, checkReason = util.checkPeripheral(bearing.name, expected)
        if not checkOk then
            self.warnings[#self.warnings + 1] = (bearing.role or bearing.name) .. ": " .. checkReason
        end
    end

    -- Preserve the main-lift RSC's current target on startup.
    local currentTarget, targetError = util.call(names.rsc.mainLift, "getTargetSpeed")
    if targetError then
        self.warnings[#self.warnings + 1] = "main lift target: " .. targetError
    elseif type(currentTarget) == "number" then
        self.targetLiftRpm = util.clamp(currentTarget, config.manual.minLiftRpm, config.manual.maxLiftRpm)
    end
    self.commanded.mainLift = self.targetLiftRpm

    -- Cache static bearing metadata (geometry) once.
    for _, bearing in ipairs(names.bearings) do
        local static, errors = util.readMethods(bearing.name, BEARING_STATIC_METHODS, bearing.name)
        static.name = bearing.name
        static.role = bearing.role
        static.gyro = bearing.gyro
        self.bearingStatic[bearing.name] = static
        for key, value in pairs(errors) do
            self.warnings[#self.warnings + 1] = key .. ": " .. value
        end
    end

    return self
end

local function clampAxis(self, axis, value)
    local target = tonumber(value) or 0
    local manual = self.config.manual
    if axis == "mainLift" then
        return self.util.clamp(target, manual.minLiftRpm, manual.maxLiftRpm)
    end
    local maxRpm = (self.config.control and self.config.control.maxRpm) or manual.maxLiftRpm
    return self.util.clamp(target, -maxRpm, maxRpm)
end

-- Command any axis RSC. Returns ok, error.
function Actuators:setAxisTarget(axis, value)
    local name = self.rsc[axis]
    if not name then return false, "unknown axis: " .. tostring(axis) end
    local target = clampAxis(self, axis, value)
    local _, commandError = self.util.call(name, "setTargetSpeed", target)
    if commandError then return false, commandError end
    self.commanded[axis] = target
    if axis == "mainLift" then self.targetLiftRpm = target end
    return true, nil
end

function Actuators:setLiftTarget(value)
    return self:setAxisTarget("mainLift", value)
end

function Actuators:updateManual()
    local manual = self.config.manual
    local km = manual.keys or {}
    local moveRpm = manual.moveRpm or 0

    local pressed, inputError = self.util.call(
        self.config.peripherals.typewriter,
        "getPressedKeyCodes"
    )
    local input = { pressed = pressed or {}, delta = 0, axes = {} }
    if inputError then input.error = inputError end

    -- build a set of currently-held key codes
    local down = {}
    if type(pressed) == "number" then pressed = { pressed } end
    if type(pressed) == "table" then
        for _, code in pairs(pressed) do down[code] = true end
    end
    local function held(code) return code ~= nil and down[code] == true end

    -- main lift: persistent stepped target (holds for hover)
    if held(km.liftUp) then input.delta = input.delta + manual.liftStep end
    if held(km.liftDown) then input.delta = input.delta - manual.liftStep end
    local ok, commandError = self:setLiftTarget(self.targetLiftRpm + input.delta)
    input.targetLiftRpm = self.targetLiftRpm
    if not ok then input.commandError = commandError end

    -- directional axes: momentary (0 when released)
    local function momentary(axis, posKey, negKey)
        local value = 0
        if held(posKey) then value = value + moveRpm end
        if held(negKey) then value = value - moveRpm end
        local axisOk, axisError = self:setAxisTarget(axis, value)
        input.axes[axis] = self.commanded[axis]
        if not axisOk and axisError then
            input.axisErrors = input.axisErrors or {}
            input.axisErrors[axis] = axisError
        end
    end
    momentary("upDown", km.ascend, km.descend)
    momentary("forwardBack", km.forward, km.back)
    momentary("leftRight", km.right, km.left)
    momentary("yaw", km.yawRight, km.yawLeft)

    return input
end

-- mode "manual": typewriter drives main lift.
-- mode "autopilot": command.axisTargets = { axis = rpm, ... } drives each RSC.
function Actuators:apply(mode, command)
    if mode == "manual" then
        return self:updateManual()
    elseif mode == "autopilot" then
        if command and type(command.axisTargets) == "table" then
            local applied, errors = {}, {}
            for axis, value in pairs(command.axisTargets) do
                local ok, err = self:setAxisTarget(axis, value)
                applied[axis] = ok
                if err then errors[axis] = err end
            end
            return { targetLiftRpm = self.targetLiftRpm, applied = applied, errors = errors }
        end
        return {
            targetLiftRpm = self.targetLiftRpm,
            commandError = "autopilot axisTargets missing",
            applied = false,
        }
    end
    return { targetLiftRpm = self.targetLiftRpm, commandError = "unsupported mode: " .. tostring(mode) }
end

function Actuators:read()
    local util = self.util
    local state = { errors = {}, rsc = {}, bearings = {} }

    for _, axis in ipairs(AXIS_ORDER) do
        local name = self.rsc[axis]
        if name then
            local data, errors = util.readMethods(name, RSC_METHODS, "rsc." .. axis)
            data.name = name
            data.commandedSpeed = self.commanded[axis]
            state.rsc[axis] = data
            util.mergeErrors(state.errors, errors)
        end
    end

    for index, bearing in ipairs(self.bearings) do
        local methods = {}
        append(methods, BEARING_DYNAMIC_METHODS)
        if bearing.gyro then append(methods, GYRO_DYNAMIC_METHODS) end
        local dynamic, errors = util.readMethods(bearing.name, methods, bearing.name)
        local result = {}
        for key, value in pairs(self.bearingStatic[bearing.name] or {}) do result[key] = value end
        for key, value in pairs(dynamic) do result[key] = value end
        result.name = bearing.name
        result.role = bearing.role
        state.bearings[index] = result
        util.mergeErrors(state.errors, errors)
    end

    return state
end

return Actuators
