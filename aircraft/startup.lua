local ROOT = "/aircraft"
local function loadModule(name)
    return dofile(fs.combine(ROOT, name .. ".lua"))
end

local config = loadModule("config")
local util = loadModule("util")
local Sensors = loadModule("sensors")
local Actuators = loadModule("actuators")
local Telemetry = loadModule("telemetry")
local Display = loadModule("display")
local Logger = loadModule("logger")

local sensors = Sensors.new(config, util)
local actuators = Actuators.new(config, util)
local telemetry = Telemetry.new(config)
local display = Display.new(config, util)
local logger = Logger.new(config)

-- The physics assembler exposes mass, inertia tensor, and center of mass
-- directly (constant while assembled), so read them once at startup.
local function readPhysics()
    local pa = config.peripherals.physicsAssembler
    return {
        mass = util.call(pa, "getMass"),
        inertiaTensor = util.call(pa, "getInertiaTensor"),
        centerOfMass = util.call(pa, "getCenterOfMass"),
    }
end
local physics = readPhysics()

local Control = loadModule("control")
local controlConfig = loadModule("control_config")
local controller = Control.new(controlConfig, util)

local function printWarnings(label, warnings)
    for _, warning in ipairs(warnings or {}) do
        print("[WARN] " .. label .. ": " .. warning)
    end
end

print("MC Aero flight computer " .. config.version)
print("Computer " .. os.getComputerID() .. " | mode: " .. config.mode)
print("Lift target preserved at " .. tostring(actuators.targetLiftRpm) .. " RPM")
printWarnings("sensor", sensors.warnings)
printWarnings("actuator", actuators.warnings)
printWarnings("display", display.warnings)
if telemetry.error then print("[WARN] telemetry: " .. telemetry.error) end
if logger.error then print("[WARN] logger: " .. logger.error) end
if logger.enabled then print("Logging to " .. logger.path) end

local controlKeys = controlConfig.keys

local function pressedSet()
    local pressed = util.call(config.peripherals.typewriter, "getPressedKeyCodes")
    local set = {}
    if type(pressed) == "number" then pressed = { pressed } end
    if type(pressed) == "table" then
        for _, code in pairs(pressed) do set[code] = true end
    end
    return set
end

local function anyManualKey(set)
    for _, code in pairs(config.manual.keys or {}) do
        if set[code] then return true end
    end
    return false
end

local function run()
    local nextDisplay = 0
    local nextTelemetry = 0
    local nextLog = 0
    local nextTick = util.nowMs()
    local telemetryError = telemetry.error
    local mode = "manual"
    local prevEngage = false

    while true do
        local now = util.nowMs()
        local keysDown = pressedSet()

        -- Mode transitions. Manual override always wins immediately: pressing the
        -- override key OR touching any manual stick key drops back to manual.
        local engageHeld = keysDown[controlKeys.engage] == true
        if mode == "manual" then
            if engageHeld and not prevEngage then
                local s = sensors:read()
                controller:engage(s, actuators.commanded.mainLift or actuators.targetLiftRpm)
                mode = "autopilot"
                print("[auto] engaged")
            end
        else
            if keysDown[controlKeys.override] == true or anyManualKey(keysDown) then
                controller:disengage()
                mode = "manual"
                -- resume manual lift where autopilot left off
                actuators.targetLiftRpm = actuators.commanded.mainLift or actuators.targetLiftRpm
                actuators.liftEnabled = true
                print("[auto] override -> manual")
            end
        end
        prevEngage = engageHeld

        -- Control path every tick. Manual = keys->RSC. Autopilot = closed loop,
        -- which needs sensor feedback each tick (kept for snapshot reuse).
        local manualInput, controlTelemetry, sensorState
        if mode == "manual" then
            manualInput = actuators:apply("manual")
        else
            sensorState = sensors:read()
            local axisTargets, tel = controller:update(sensorState, controlConfig.dt)
            actuators:apply("autopilot", { axisTargets = axisTargets })
            controlTelemetry = tel
        end

        local doTelemetry = now >= nextTelemetry
        local doDisplay = now >= nextDisplay
        local doLog = logger.enabled and now >= nextLog

        -- Slow monitoring path: heavy actuator reads + snapshot only when due.
        if doTelemetry or doDisplay or doLog then
            local readStarted = util.nowMs()
            if not sensorState then sensorState = sensors:read() end
            local actuatorState = actuators:read()

            local errors = {}
            util.mergeErrors(errors, sensorState.errors)
            util.mergeErrors(errors, actuatorState.errors)
            if manualInput then
                if manualInput.error then errors["manual.input"] = manualInput.error end
                if manualInput.commandError then errors["manual.command"] = manualInput.commandError end
                if manualInput.axisErrors then util.mergeErrors(errors, manualInput.axisErrors) end
            end

            local snapshot = {
                schema = "mc_aero.telemetry.v1",
                version = config.version,
                timestampMs = util.nowMs(),
                computerId = os.getComputerID(),
                mode = mode,
                manualInput = manualInput,
                autopilot = controlTelemetry,
                sensors = sensorState,
                actuators = actuatorState,
                errors = errors,
                errorCount = util.count(errors),
                telemetryError = telemetryError,
                physics = physics,
            }
            snapshot.loopDurationMs = util.nowMs() - readStarted

            if doTelemetry then
                local ok, sendError = telemetry:broadcast(snapshot)
                telemetryError = ok and nil or sendError
                snapshot.telemetryError = telemetryError
                nextTelemetry = now + math.floor(config.telemetry.period * 1000)
            end
            if doDisplay then
                display:update(snapshot)
                nextDisplay = now + math.floor(config.displayPeriod * 1000)
            end
            if doLog then
                local _, logError = logger:write(snapshot)
                if logError then errors["logger"] = logError end
                nextLog = now + math.floor(config.logging.period * 1000)
            end
        end

        nextTick = nextTick + math.floor(config.loopPeriod * 1000)
        local remaining = nextTick - util.nowMs()
        if remaining < 0 then
            nextTick = util.nowMs()
            remaining = 0
        end
        sleep(remaining / 1000)
    end
end

local ok, runError = pcall(run)
-- Safety on stop: zero the directional/vertical-trim RSCs so nothing runs away,
-- but leave main lift untouched (preserve philosophy).
pcall(function()
    actuators:setAxisTarget("forwardBack", 0)
    actuators:setAxisTarget("leftRight", 0)
    actuators:setAxisTarget("yaw", 0)
    actuators:setAxisTarget("upDown", 0)
end)
logger:close()
telemetry:close()

if not ok and tostring(runError) ~= "Terminated" then
    error(runError, 0)
end
print("Flight computer stopped; lift target was not changed during shutdown.")
