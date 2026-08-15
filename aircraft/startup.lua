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

local function run()
    local sequence = 0
    local nextDisplay = 0
    local nextTelemetry = 0
    local nextLog = 0
    local nextTick = util.nowMs()
    local telemetryError = telemetry.error

    while true do
        local loopStarted = util.nowMs()
        local manualInput = actuators:apply(config.mode)
        local sensorState = sensors:read()
        local actuatorState = actuators:read()

        local errors = {}
        util.mergeErrors(errors, sensorState.errors)
        util.mergeErrors(errors, actuatorState.errors)
        if manualInput.error then errors["manual.input"] = manualInput.error end
        if manualInput.commandError then errors["manual.command"] = manualInput.commandError end
        if manualInput.axisErrors then util.mergeErrors(errors, manualInput.axisErrors) end

        local snapshot = {
            schema = "mc_aero.telemetry.v1",
            version = config.version,
            timestampMs = util.nowMs(),
            computerId = os.getComputerID(),
            sequence = sequence,
            mode = config.mode,
            manualInput = manualInput,
            sensors = sensorState,
            actuators = actuatorState,
            errors = errors,
            errorCount = util.count(errors),
            telemetryError = telemetryError,
            physics = physics,
        }
        snapshot.loopDurationMs = util.nowMs() - loopStarted

        local now = util.nowMs()
        if now >= nextTelemetry then
            local ok, sendError = telemetry:broadcast(snapshot)
            telemetryError = ok and nil or sendError
            snapshot.telemetryError = telemetryError
            nextTelemetry = now + math.floor(config.telemetry.period * 1000)
        end
        if now >= nextDisplay then
            display:update(snapshot)
            nextDisplay = now + math.floor(config.displayPeriod * 1000)
        end
        if logger.enabled and now >= nextLog then
            local _, logError = logger:write(snapshot)
            if logError then errors["logger"] = logError end
            nextLog = now + math.floor(config.logging.period * 1000)
        end

        sequence = sequence + 1
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
logger:close()
telemetry:close()

if not ok and tostring(runError) ~= "Terminated" then
    error(runError, 0)
end
print("Flight computer stopped; lift target was not changed during shutdown.")
