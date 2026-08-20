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

-- Mass and inertia tensor are constant while assembled, so read them once.
local function readPhysics()
    local pa = config.peripherals.physicsAssembler
    return {
        mass = util.call(pa, "getMass"),
        inertiaTensor = util.call(pa, "getInertiaTensor"),
    }
end
local physics = readPhysics()

-- The center of mass is the true center of rotation. Its world coordinates
-- shift as the craft translates, so read it LIVE each snapshot (not once) to
-- track the CoM state separately from the offset nav/gimbal position. This is
-- p_com; the nav-reconstructed position is p_nav (sensor, offset from CoM).
local function readCenterOfMass()
    return util.call(config.peripherals.physicsAssembler, "getCenterOfMass")
end

local LQI = loadModule("lqi")
local lqiConfig = loadModule("lqi_config")
local plantK = loadModule("plant_K")
local controller = LQI.new(lqiConfig, util, plantK)

local Guidance = loadModule("guidance")
local guidance = Guidance.new(config.mission)
local commandProtocol = config.mission.commandProtocol
local mailbox = {}  -- shared with the receiver coroutine (cooperative, no locking needed)

-- Ensure the command modem (rednet) is open for receiving go-to requests.
do
    local side = config.telemetry.modemSide
    if peripheral.getType(side) == "modem" and not rednet.isOpen(side) then
        pcall(rednet.open, side)
    end
end

-- Non-blocking "process waiting for requests": blocks on rednet.receive; when a
-- request lands it is queued for the control loop to consume on its next tick.
-- Runs concurrently with the control loop via parallel.waitForAny.
local function receiver()
    while true do
        local sender, message = rednet.receive(commandProtocol)
        if type(message) == "table" and message.kind then
            mailbox[#mailbox + 1] = { sender = sender, msg = message }
        end
    end
end

local function sendAck(sender, id, accepted, reason)
    pcall(rednet.send, sender, {
        kind = "ack", id = id, accepted = accepted, reason = reason,
        computerId = os.getComputerID(),
    }, commandProtocol)
end

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

local controlKeys = lqiConfig.keys

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

    -- Snapshot current CoM-referenced state for guidance/commands.
    local function currentState(s)
        s = s or sensors:read()
        local pos, nav, alt = s.position or {}, s.navigation or {}, s.altitude or {}
        return s, {
            x = pos.comX, z = pos.comZ,
            height = tonumber(alt.height), heading = tonumber(nav.getHeading),
            dt = lqiConfig.dt,
        }
    end

    local function engageAt(s, cur)
        controller:engage(s, actuators.commanded.mainLift or actuators.targetLiftRpm)
        guidance:hold(controller.targets.x, controller.targets.z,
            controller.targets.altitude, controller.targets.heading)
        mode = "autopilot"
    end

    -- goto takes over from manual; cancel/hold parks a hover at the current CoM.
    local function processCommand(entry)
        local msg, sender = entry.msg, entry.sender
        local kind = msg.kind
        if kind == "ping" then sendAck(sender, msg.id, true, "pong"); return end
        local s, cur = currentState()
        if not (cur.x and cur.z) then sendAck(sender, msg.id, false, "no position fix"); return end
        if mode ~= "autopilot" then engageAt(s, cur) end
        if kind == "goto" then
            local ok, reason = guidance:go(
                { x = msg.x, z = msg.z, altitude = msg.altitude, heading = msg.heading }, cur)
            sendAck(sender, msg.id, ok, reason)
            if ok then print(string.format("[goto] %.1f, %.1f", msg.x, msg.z)) end
        elseif kind == "hold" or kind == "cancel" then
            guidance:hold(cur.x, cur.z, cur.height, cur.heading)
            sendAck(sender, msg.id, true, kind)
            print("[auto] " .. kind .. " -> hold")
        else
            sendAck(sender, msg.id, false, "unknown kind")
        end
    end

    while true do
        local now = util.nowMs()
        local keysDown = pressedSet()

        -- Mode transitions. Manual override always wins immediately: pressing the
        -- override key OR touching any manual stick key drops back to manual.
        local engageHeld = keysDown[controlKeys.engage] == true
        if mode == "manual" then
            if engageHeld and not prevEngage then
                local s, cur = currentState()
                engageAt(s, cur)
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

        -- Consume any queued go-to / hold / cancel requests (non-blocking).
        while #mailbox > 0 do
            processCommand(table.remove(mailbox, 1))
        end

        -- Control path every tick. Manual = keys->RSC. Autopilot = closed loop,
        -- which needs sensor feedback each tick (kept for snapshot reuse).
        local manualInput, controlTelemetry, sensorState
        if mode == "manual" then
            manualInput = actuators:apply("manual")
        else
            sensorState = sensors:read()
            local pos, nav, alt = sensorState.position or {}, sensorState.navigation or {}, sensorState.altitude or {}
            local cur = {
                x = pos.comX, z = pos.comZ,
                height = tonumber(alt.height), heading = tonumber(nav.getHeading),
                dt = lqiConfig.dt,
            }
            local sp, gstate = guidance:update(cur)
            controller:setTarget(sp)
            local axisTargets, tel = controller:update(sensorState, lqiConfig.dt)
            actuators:apply("autopilot", { axisTargets = axisTargets })
            tel.guidance = { state = gstate }
            if guidance.goal and cur.x and cur.z then
                tel.guidance.goalX, tel.guidance.goalZ = guidance.goal.x, guidance.goal.z
                tel.guidance.goalAltitude = guidance.goal.altitude
                tel.guidance.distance = math.sqrt((guidance.goal.x - cur.x) ^ 2 + (guidance.goal.z - cur.z) ^ 2)
            end
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
            local centerOfMass, comError = readCenterOfMass()

            local errors = {}
            util.mergeErrors(errors, sensorState.errors)
            util.mergeErrors(errors, actuatorState.errors)
            if comError then errors["physics.getCenterOfMass"] = comError end
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
                centerOfMass = centerOfMass,
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

local ok, runError = pcall(parallel.waitForAny, run, receiver)
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
