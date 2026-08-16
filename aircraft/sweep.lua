-- MC Aero actuator sweep: characterize thrust curves (k) and spin-up dynamics
-- (tau, command->rpm gain) for each RSC/prop group.
--
-- Steps one RSC through a sequence of target speeds, holding each so rpm/thrust
-- settle, and broadcasts labeled telemetry (mc_aero.telemetry.v1 + a `sweep`
-- block) so the ground relay forwards it to S3 for offline fitting.
--
-- SAFETY: this spins props to full power. LAND or ANCHOR the craft first.
-- Press the start key to begin, the abort key (or Ctrl+T) to stop; all RSCs are
-- zeroed on exit.
--
-- Usage: aircraft/sweep [axis]
--   axis = mainLift | forwardBack | yaw | leftRight | upDown | all  (default all)

local ROOT = "/aircraft"
local function loadModule(name) return dofile(fs.combine(ROOT, name .. ".lua")) end

local config = loadModule("config")
local util = loadModule("util")
local Sensors = loadModule("sensors")
local Actuators = loadModule("actuators")
local Telemetry = loadModule("telemetry")

-- sweep parameters (override via config.sweep if present)
local SWEEP = config.sweep or {}
local STEPS = SWEEP.steps or { 0, 32, 64, 96, 128, 160, 192, 224, 256, 0 }
local HOLD_SECONDS = SWEEP.holdSeconds or 3.0
local SAMPLE_PERIOD = SWEEP.samplePeriod or 0.05 -- 20 Hz to capture spin-up
local START_KEY = SWEEP.startKey or keys.space
local ABORT_KEY = SWEEP.abortKey or keys.x
local ALL_AXES = { "mainLift", "forwardBack", "yaw", "leftRight", "upDown" }

local requested = ...
local axes
if not requested or requested == "all" then
    axes = ALL_AXES
elseif config.peripherals.rsc[requested] then
    axes = { requested }
else
    error("unknown axis: " .. tostring(requested) .. " (use " .. table.concat(ALL_AXES, "/") .. "/all)", 0)
end

local sensors = Sensors.new(config, util)
local actuators = Actuators.new(config, util)
local telemetry = Telemetry.new(config)

local function zeroAll()
    for _, axis in ipairs(ALL_AXES) do pcall(function() actuators:setAxisTarget(axis, 0) end) end
end

local function keysDown()
    local set = {}
    local p = { pcall(peripheral.call, config.peripherals.typewriter, "getPressedKeyCodes") }
    if p[1] and type(p[2]) == "table" then
        for _, code in pairs(p[2]) do set[code] = true end
    end
    return set
end

print("MC Aero actuator sweep " .. config.version)
print("Axes: " .. table.concat(axes, ", "))
print("Steps: " .. table.concat(STEPS, ", ") .. "  hold " .. HOLD_SECONDS .. "s")
print("!! LAND OR ANCHOR THE CRAFT. Props will spin to full power. !!")
print("Press START key to begin, ABORT key to stop.")
if telemetry.error then print("[WARN] telemetry: " .. telemetry.error) end

zeroAll()

-- wait for start / allow abort before spinning anything
repeat
    local down = keysDown()
    if down[ABORT_KEY] then print("aborted before start"); telemetry:close(); return end
    sleep(0.1)
until keysDown()[START_KEY]

local aborted = false

local function broadcast(axis, target, stepStartMs)
    local sensorState = sensors:read()
    local actuatorState = actuators:read()
    local errors = {}
    util.mergeErrors(errors, sensorState.errors)
    util.mergeErrors(errors, actuatorState.errors)
    telemetry:broadcast({
        schema = "mc_aero.telemetry.v1",
        version = config.version,
        timestampMs = util.nowMs(),
        computerId = os.getComputerID(),
        mode = "sweep",
        sensors = sensorState,
        actuators = actuatorState,
        errors = errors,
        errorCount = util.count(errors),
        physics = { }, -- static; omitted here to keep rows small
        sweep = {
            axis = axis,
            target = target,
            stepStartMs = stepStartMs,
            elapsedMs = util.nowMs() - stepStartMs,
        },
    })
end

local ok, err = pcall(function()
    for _, axis in ipairs(axes) do
        print("=== sweeping " .. axis .. " ===")
        for _, target in ipairs(STEPS) do
            actuators:setAxisTarget(axis, target)
            local stepStart = util.nowMs()
            local nextTick = stepStart
            print(string.format("  %s -> %d rpm", axis, target))
            while (util.nowMs() - stepStart) < HOLD_SECONDS * 1000 do
                if keysDown()[ABORT_KEY] then aborted = true; return end
                broadcast(axis, target, stepStart)
                nextTick = nextTick + math.floor(SAMPLE_PERIOD * 1000)
                local remaining = nextTick - util.nowMs()
                if remaining < 0 then nextTick = util.nowMs(); remaining = 0 end
                sleep(remaining / 1000)
            end
        end
        actuators:setAxisTarget(axis, 0)
    end
end)

zeroAll()
telemetry:close()

if aborted then
    print("sweep ABORTED; all RSCs zeroed")
elseif not ok and tostring(err) ~= "Terminated" then
    error(err, 0)
else
    print("sweep complete; all RSCs zeroed")
end
