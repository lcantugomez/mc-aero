-- MC Aero ground station
-- Runs on a base computer with an ender/wireless modem and one or more
-- monitors. Receives aircraft telemetry over rednet, renders it to the
-- monitor(s), and (optionally) relays it to an HTTPS endpoint.
--
-- Self-contained: does not depend on the aircraft package.
--
-- Per-computer overrides go in /station_config.lua, e.g.:
--   return {
--     pages = { "signals" },          -- what this station displays
--     relay = true,                    -- only ONE station should relay
--     endpoint = "https://xxx.lambda-url.us-east-2.on.aws/",
--     apiKey = "the-secret",
--   }
-- If endpoint/apiKey are omitted, /relay_config.lua is used as a fallback.

-- ---- configuration -------------------------------------------------------
local CONFIG = {
    protocol = "mc_aero.telemetry.v1",
    pages = { "flight", "systems" }, -- one page drawn per detected monitor, in order
    textScale = 0.5,
    staleMs = 1500, -- link considered stale after this gap

    relay = false,
    endpoint = "https://REPLACE_ME.lambda-url.us-east-1.on.aws/",
    apiKey = "REPLACE_WITH_SECRET",
    flushInterval = 1.0,
    maxBatch = 40,
    maxBuffer = 400,
}

-- Load config from the first file that exists. Copy the templates in
-- ground/ to one of these paths (computer root, or next to this script).
local function loadConfig(paths)
    for _, path in ipairs(paths) do
        if fs.exists(path) then
            local ok, result = pcall(dofile, path)
            if ok and type(result) == "table" then return result, path end
            print("station: ignoring invalid " .. path)
        end
    end
    return nil
end

local overrides, overridePath = loadConfig({ "/station_config.lua", "/ground/station_config.lua" })
if overrides then
    for key, value in pairs(overrides) do CONFIG[key] = value end
    print("station: loaded config from " .. overridePath)
end
-- fall back to a shared relay config for the endpoint/secret
if CONFIG.endpoint:find("REPLACE_ME") or CONFIG.apiKey == "REPLACE_WITH_SECRET" then
    local relayCfg = loadConfig({ "/relay_config.lua", "/ground/relay_config.lua" })
    if relayCfg then
        CONFIG.endpoint = relayCfg.endpoint or CONFIG.endpoint
        CONFIG.apiKey = relayCfg.apiKey or CONFIG.apiKey
        CONFIG.protocol = relayCfg.protocol or CONFIG.protocol
    end
end
-- --------------------------------------------------------------------------

local function nowMs() return os.epoch("utc") end

-- Compact one-line rendering of any value (numbers to 3 dp, bounded tables).
local function render(value, depth, visited)
    local t = type(value)
    if t == "nil" then return "-" end
    if t == "number" then return string.format("%.3f", value) end
    if t == "boolean" then return value and "true" or "false" end
    if t == "string" then return (value:gsub("[\r\n]", " ")) end
    if t ~= "table" then return tostring(value) end
    if depth >= 2 or visited[value] then return "{...}" end
    visited[value] = true
    local keys = {}
    for k in pairs(value) do keys[#keys + 1] = k end
    table.sort(keys, function(a, b) return tostring(a) < tostring(b) end)
    local parts = {}
    for i, k in ipairs(keys) do
        if i > 6 then parts[#parts + 1] = "..."; break end
        local item = render(value[k], depth + 1, visited)
        parts[#parts + 1] = type(k) == "number" and item or (tostring(k) .. "=" .. item)
    end
    visited[value] = nil
    return "{" .. table.concat(parts, ",") .. "}"
end
local function oneLine(value) return render(value, 0, {}) end

local function shortName(name)
    return (name or "?")
        :gsub("gyroscopic_propeller_bearing_", "G")
        :gsub("propeller_bearing_", "P")
        :gsub("directional_gearshift_", "D")
end

-- ---- peripherals ---------------------------------------------------------
local function openModem()
    local fallback
    for _, name in ipairs(peripheral.getNames()) do
        if peripheral.getType(name) == "modem" then
            if peripheral.call(name, "isWireless") then
                rednet.open(name)
                return name, true
            end
            fallback = fallback or name
        end
    end
    if fallback then rednet.open(fallback); return fallback, false end
    error("no modem attached to this computer", 0)
end

local function findMonitors()
    local monitors = {}
    for _, name in ipairs(peripheral.getNames()) do
        if peripheral.getType(name) == "monitor" then
            monitors[#monitors + 1] = name
        end
    end
    return monitors
end

-- ---- page builders (reconstruct from the received snapshot) --------------
local function buildFlight(snap)
    local sensors = snap.sensors or {}
    local altitude = sensors.altitude or {}
    local gimbal = sensors.gimbal or {}
    local lines = {
        string.format("ID %s  MODE %s  SEQ %s",
            oneLine(snap.computerId), tostring(snap.mode), oneLine(snap.sequence)),
        "Height: " .. oneLine(altitude.height),
        "Vertical speed: " .. oneLine(altitude.verticalSpeed),
        "Air pressure: " .. oneLine(altitude.airPressure),
        "Angles deg: " .. oneLine(gimbal.getAngles),
        "Rates: " .. oneLine(gimbal.getAngularRatesRad),
        "Acceleration: " .. oneLine(gimbal.getLinearAcceleration),
        "Gravity: " .. oneLine(gimbal.getGravity),
    }
    for _, velocity in ipairs(sensors.velocity or {}) do
        lines[#lines + 1] = "Velocity " .. oneLine(velocity.axis) .. ": " .. oneLine(velocity.velocity)
    end
    local nav = sensors.navigation or {}
    lines[#lines + 1] = "Heading: " .. oneLine(nav.getHeading)
    lines[#lines + 1] = "Nav target: " .. oneLine(nav.hasTarget)
    if nav.hasTarget then
        lines[#lines + 1] = "Distance: " .. oneLine(nav.getDistanceToTarget)
        lines[#lines + 1] = "Bearing: " .. oneLine(nav.getBearing)
    end
    lines[#lines + 1] = string.format("Loop %s ms  Errors %s",
        oneLine(snap.loopDurationMs), oneLine(snap.errorCount))
    return "MC AERO - FLIGHT", lines
end

local function buildSystems(snap)
    local actuators = snap.actuators or {}
    local rsc = actuators.liftController or {}
    local lines = {
        "Commanded: " .. oneLine(rsc.commandedSpeed),
        "RSC target: " .. oneLine(rsc.getTargetSpeed),
        "RSC actual: " .. oneLine(rsc.getSpeed),
        "RSC source: " .. oneLine(rsc.hasSource) .. "  overstress: " .. oneLine(rsc.isOverstressed),
        "--- PROPELLERS ---",
    }
    for _, b in ipairs(actuators.bearings or {}) do
        lines[#lines + 1] = string.format("%s T=%s RPM=%s A=%s",
            shortName(b.name), oneLine(b.getThrust), oneLine(b.getRotationSpeed), oneLine(b.isActive))
    end
    lines[#lines + 1] = "--- GEARSHIFTS (read only) ---"
    for _, g in ipairs(actuators.gearshifts or {}) do
        lines[#lines + 1] = string.format("%s L=%s R=%s speed=%s",
            shortName(g.name), oneLine(g.isLeftPowered), oneLine(g.isRightPowered), oneLine(g.getSpeed))
    end
    if snap.manualInput then lines[#lines + 1] = "Input delta: " .. oneLine(snap.manualInput.delta) end
    return "MC AERO - SYSTEMS", lines
end

-- Characterization view: commanded vs actual RPM, per-bearing rpm/thrust and
-- the bearing/RSC ratio (steady-state gain), plus overstress flags.
local function buildSignals(snap)
    local actuators = snap.actuators or {}
    local rsc = actuators.liftController or {}
    local target = tonumber(rsc.getTargetSpeed)
    local lines = {
        "RSC target : " .. oneLine(rsc.getTargetSpeed),
        "RSC actual : " .. oneLine(rsc.getSpeed),
        "Overstress : " .. oneLine(rsc.isOverstressed),
        "--- BEARINGS  rpm / thrust / ratio ---",
    }
    local totalThrust = 0
    for _, b in ipairs(actuators.bearings or {}) do
        local rpm = tonumber(b.getRotationSpeed)
        local thrust = tonumber(b.getThrust)
        if thrust then totalThrust = totalThrust + thrust end
        local ratio = (rpm and target and target ~= 0) and string.format("%.3f", rpm / target) or "-"
        lines[#lines + 1] = string.format("%s rpm=%s T=%s k=%s",
            shortName(b.name), oneLine(b.getRotationSpeed), oneLine(b.getThrust), ratio)
    end
    lines[#lines + 1] = string.format("Total thrust: %.3f", totalThrust)
    return "MC AERO - SIGNALS", lines
end

local function buildRaw(snap)
    local lines = {
        "schema: " .. oneLine(snap.schema),
        "version: " .. oneLine(snap.version),
        "seq: " .. oneLine(snap.sequence) .. "  errors: " .. oneLine(snap.errorCount),
        "telemetryError: " .. oneLine(snap.telemetryError),
        "--- ERRORS ---",
    }
    local errors = snap.errors or {}
    local keys = {}
    for k in pairs(errors) do keys[#keys + 1] = k end
    table.sort(keys)
    if #keys == 0 then lines[#lines + 1] = "(none)" end
    for _, k in ipairs(keys) do lines[#lines + 1] = k .. ": " .. oneLine(errors[k]) end
    return "MC AERO - RAW", lines
end

local PAGES = {
    flight = buildFlight,
    systems = buildSystems,
    signals = buildSignals,
    raw = buildRaw,
}

-- ---- rendering -----------------------------------------------------------
local function drawPage(monitor, header, title, lines)
    local ok = pcall(function()
        monitor.setBackgroundColor(colors.black)
        monitor.setTextColor(colors.white)
        monitor.clear()
        local width = monitor.getSize()
        monitor.setCursorPos(1, 1)
        monitor.setTextColor(colors.yellow)
        monitor.write(header:sub(1, width))
        monitor.setCursorPos(1, 2)
        monitor.setTextColor(colors.cyan)
        monitor.write(title:sub(1, width))
        monitor.setTextColor(colors.white)
        local _, height = monitor.getSize()
        for i, line in ipairs(lines) do
            local row = i + 2
            if row > height then break end
            monitor.setCursorPos(1, row)
            monitor.write(tostring(line):sub(1, width))
        end
    end)
    return ok
end

-- ---- main ----------------------------------------------------------------
local modemName, wireless = openModem()
local monitorNames = findMonitors()
local monitors = {}
for i, name in ipairs(monitorNames) do
    local mon = peripheral.wrap(name)
    pcall(function() mon.setTextScale(CONFIG.textScale) end)
    monitors[i] = { name = name, handle = mon }
end

print(string.format("station: %s modem on %s", wireless and "wireless" or "wired", modemName))
print("station: protocol " .. CONFIG.protocol)
print(string.format("station: %d monitor(s), pages: %s", #monitors, table.concat(CONFIG.pages, ", ")))
print("station: relay " .. (CONFIG.relay and ("ON -> " .. CONFIG.endpoint) or "OFF"))

local latest = nil
local lastRxMs = 0
local buffer = {}
local sending = nil

local function redraw()
    if #monitors == 0 then return end
    local age = latest and (nowMs() - lastRxMs) or nil
    local status
    if not latest then
        status = "WAITING"
    elseif age and age > CONFIG.staleMs then
        status = string.format("STALE %dms", age)
    else
        status = string.format("OK %dms", age or 0)
    end
    local src = latest and oneLine(latest.computerId) or "-"
    local header = string.format("GND src=%s %s", src, status)

    for i, monitor in ipairs(monitors) do
        local pageName = CONFIG.pages[i] or CONFIG.pages[#CONFIG.pages] or "flight"
        local builder = PAGES[pageName]
        if not latest then
            drawPage(monitor.handle, header, "MC AERO - " .. pageName:upper(),
                { "Waiting for telemetry on", CONFIG.protocol .. " ..." })
        elseif builder then
            local title, lines = builder(latest)
            drawPage(monitor.handle, header, title, lines)
        else
            drawPage(monitor.handle, header, "UNKNOWN PAGE", { "no page named '" .. pageName .. "'" })
        end
    end
end

local function tryFlush()
    if not CONFIG.relay or sending or #buffer == 0 then return end
    if not http then return end
    local take = math.min(CONFIG.maxBatch, #buffer)
    local batch = {}
    for i = 1, take do batch[i] = buffer[i] end
    local remaining = {}
    for i = take + 1, #buffer do remaining[#remaining + 1] = buffer[i] end
    buffer = remaining
    local ok, body = pcall(textutils.serializeJSON, batch)
    if not ok then print("station: JSON encode failed, dropped " .. take); return end
    sending = batch
    http.request({
        url = CONFIG.endpoint,
        method = "POST",
        headers = { ["Content-Type"] = "application/json", ["x-api-key"] = CONFIG.apiKey },
        body = body,
    })
end

redraw()
local flushTimer = os.startTimer(CONFIG.flushInterval)
local uiTimer = os.startTimer(0.5)

while true do
    local event = { os.pullEvent() }
    local kind = event[1]

    if kind == "rednet_message" then
        local _, message, protocol = event[2], event[3], event[4]
        if protocol == CONFIG.protocol and type(message) == "table" then
            latest = message
            lastRxMs = nowMs()
            if CONFIG.relay then
                buffer[#buffer + 1] = message
                while #buffer > CONFIG.maxBuffer do table.remove(buffer, 1) end
                if #buffer >= CONFIG.maxBatch then tryFlush() end
            end
            redraw()
        end
    elseif kind == "timer" and event[2] == flushTimer then
        tryFlush()
        flushTimer = os.startTimer(CONFIG.flushInterval)
    elseif kind == "timer" and event[2] == uiTimer then
        redraw() -- refresh staleness indicator even with no new data
        uiTimer = os.startTimer(0.5)
    elseif kind == "http_success" then
        local url, handle = event[2], event[3]
        if url == CONFIG.endpoint then
            if handle then handle.readAll(); handle.close() end
            sending = nil
        end
    elseif kind == "http_failure" then
        local url, reason, handle = event[2], event[3], event[4]
        if url == CONFIG.endpoint then
            if handle then handle.close() end
            print("station: POST failed: " .. tostring(reason))
            if sending then
                for i = #sending, 1, -1 do table.insert(buffer, 1, sending[i]) end
                while #buffer > CONFIG.maxBuffer do table.remove(buffer, #buffer) end
                sending = nil
            end
        end
    end
end
