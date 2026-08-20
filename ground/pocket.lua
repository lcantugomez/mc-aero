-- MC Aero pocket ground station + command console.
-- Runs on a POCKET computer with an ender modem. Two jobs run concurrently:
--   1) relay: receive telemetry over rednet, mirror it to the S3 endpoint.
--   2) console: send go-to / hold / cancel requests to the aircraft, manage
--      typed waypoints, and show live link + mission status.
-- Needs HTTP enabled. Copy ground/relay_config.example.lua to /pocket_config.lua
-- (or /relay_config.lua) with your endpoint + apiKey.

local CONFIG = {
    protocol = "mc_aero.telemetry.v1",
    commandProtocol = "mc_aero.command.v1",
    endpoint = "https://REPLACE_ME.lambda-url.us-east-2.on.aws/",
    apiKey = "REPLACE_WITH_SECRET",
    flushInterval = 1.0,
    maxBatch = 40,
    maxBuffer = 200,
    staleMs = 1500,
    waypointFile = "/waypoints.lua",
}

local function loadConfig(paths)
    for _, path in ipairs(paths) do
        if fs.exists(path) then
            local ok, result = pcall(dofile, path)
            if ok and type(result) == "table" then return result, path end
        end
    end
    return nil
end
local override = loadConfig({ "/pocket_config.lua", "/relay_config.lua" })
if override then
    CONFIG.endpoint = override.endpoint or CONFIG.endpoint
    CONFIG.apiKey = override.apiKey or CONFIG.apiKey
    CONFIG.protocol = override.protocol or CONFIG.protocol
    CONFIG.commandProtocol = override.commandProtocol or CONFIG.commandProtocol
end

-- Only push to S3 when the endpoint + key are actually configured; otherwise the
-- pocket is command-only and we skip HTTP entirely (no mounting http_failure).
local relayEnabled = type(CONFIG.endpoint) == "string"
    and not CONFIG.endpoint:find("REPLACE")
    and type(CONFIG.apiKey) == "string"
    and not CONFIG.apiKey:find("REPLACE")

if relayEnabled and not http then error("HTTP is disabled in the CC:Tweaked config", 0) end

local function openModem()
    for _, name in ipairs(peripheral.getNames()) do
        if peripheral.getType(name) == "modem" then
            rednet.open(name)
            return name
        end
    end
    error("no modem on this pocket computer (needs an ender/wireless modem)", 0)
end
openModem()

-- ---- helpers ------------------------------------------------------------
local function nowMs() return os.epoch("utc") end
local function n(v) return type(v) == "number" and v or nil end
local function fmt(v) local x = n(v); return x and string.format("%.1f", x) or "-" end
local function get(tbl, ...)
    local cur = tbl
    for _, key in ipairs({ ... }) do
        if type(cur) ~= "table" then return nil end
        cur = cur[key]
    end
    return cur
end

-- ---- shared state -------------------------------------------------------
local latest, lastRxMs = nil, 0
local buffer, sending = {}, nil
local posted, failed = 0, 0
local acks = {}
local nextCmdId = 0
local lastAck = nil

local function loadWaypoints()
    local ok, wp = pcall(dofile, CONFIG.waypointFile)
    if ok and type(wp) == "table" then return wp end
    return {}
end
local function saveWaypoints(wp)
    local f = fs.open(CONFIG.waypointFile, "w")
    if not f then return false end
    f.write("return " .. textutils.serialize(wp) .. "\n")
    f.close()
    return true
end
local waypoints = loadWaypoints()

-- ---- S3 relay (no rendering; console owns the screen) -------------------
local function tryFlush()
    if not relayEnabled then return end
    if sending or #buffer == 0 then return end
    local take = math.min(CONFIG.maxBatch, #buffer)
    local batch = {}
    for i = 1, take do batch[i] = buffer[i] end
    local remaining = {}
    for i = take + 1, #buffer do remaining[#remaining + 1] = buffer[i] end
    buffer = remaining
    local ok, body = pcall(textutils.serializeJSON, batch)
    if not ok then return end
    sending = batch
    http.request({
        url = CONFIG.endpoint,
        method = "POST",
        headers = { ["Content-Type"] = "application/json", ["x-api-key"] = CONFIG.apiKey },
        body = body,
    })
end

local function relay()
    local flushTimer = os.startTimer(CONFIG.flushInterval)
    while true do
        local e = { os.pullEvent() }
        local kind = e[1]
        if kind == "rednet_message" then
            local _, message, protocol = e[2], e[3], e[4]
            if protocol == CONFIG.protocol and type(message) == "table" then
                latest = message
                lastRxMs = nowMs()
                if relayEnabled then
                    buffer[#buffer + 1] = message
                    while #buffer > CONFIG.maxBuffer do table.remove(buffer, 1) end
                    if #buffer >= CONFIG.maxBatch then tryFlush() end
                end
            elseif protocol == CONFIG.commandProtocol and type(message) == "table" and message.kind == "ack" then
                acks[message.id] = message
            end
        elseif kind == "timer" and e[2] == flushTimer then
            tryFlush()
            flushTimer = os.startTimer(CONFIG.flushInterval)
        elseif kind == "http_success" then
            local url, handle = e[2], e[3]
            if url == CONFIG.endpoint then
                if handle then handle.readAll(); handle.close() end
                posted = posted + 1
                sending = nil
            end
        elseif kind == "http_failure" then
            local url, _, handle = e[2], e[3], e[4]
            if url == CONFIG.endpoint then
                if handle then handle.close() end
                failed = failed + 1
                if sending then
                    for i = #sending, 1, -1 do table.insert(buffer, 1, sending[i]) end
                    while #buffer > CONFIG.maxBuffer do table.remove(buffer, #buffer) end
                    sending = nil
                end
            end
        end
    end
end

-- ---- command sending ----------------------------------------------------
local function sendCommand(msg)
    nextCmdId = nextCmdId + 1
    msg.id = nextCmdId
    rednet.broadcast(msg, CONFIG.commandProtocol)
    return msg.id
end

local function awaitAck(id, timeoutMs)
    local deadline = nowMs() + (timeoutMs or 2500)
    while nowMs() < deadline do
        if acks[id] then return acks[id] end
        sleep(0.1)   -- yields so the relay coroutine can capture the ack
    end
    return nil
end

-- ---- console UI ---------------------------------------------------------
local function statusLines()
    local age = latest and (nowMs() - lastRxMs) or nil
    local link = (not latest and "WAIT")
        or (age and age > CONFIG.staleMs and ("STALE " .. age .. "ms"))
        or ("OK " .. (age or 0) .. "ms")
    local lines = { "MC AERO POCKET", "link: " .. link }
    if latest then
        lines[#lines + 1] = "mode: " .. tostring(latest.mode)
        local gd = get(latest, "autopilot", "guidance")
        if gd then
            lines[#lines + 1] = "gd:   " .. tostring(gd.state) .. "  d=" .. fmt(gd.distance)
        end
        local pos = get(latest, "sensors", "position")
        if pos and pos.comX then
            lines[#lines + 1] = string.format("com:  %s,%s", fmt(pos.comX), fmt(pos.comZ))
        end
        lines[#lines + 1] = "alt:  " .. fmt(get(latest, "sensors", "altitude", "getHeight"))
            .. " hdg:" .. fmt(get(latest, "sensors", "navigation", "getHeading"))
    end
    if lastAck then
        lines[#lines + 1] = "ack: " .. (lastAck.accepted and "OK " or "NO ")
            .. tostring(lastAck.reason or "")
    end
    lines[#lines + 1] = relayEnabled
        and string.format("S3 ok:%d err:%d", posted, failed)
        or "S3 relay: off"
    return lines
end

local function render()
    local w, h = term.getSize()
    term.setBackgroundColor(colors.black)
    term.setTextColor(colors.white)
    term.clear()
    local lines = statusLines()
    for i, s in ipairs(lines) do
        term.setCursorPos(1, i)
        term.write(tostring(s):sub(1, w))
    end
    term.setCursorPos(1, h)
    term.write("[g]oto [w]p [s]ave [h]old [c]ancel")
end

local function prompt(label, allowBlank)
    local _, h = term.getSize()
    term.setCursorPos(1, h)
    term.clearLine()
    term.write(label)
    local s = read()
    if allowBlank and (s == nil or s == "") then return nil, true end
    return tonumber(s), (s ~= nil and s ~= "")
end

local function showAck(a)
    lastAck = a or { accepted = false, reason = "no reply (timeout)" }
    render()
    sleep(1.2)
end

local function doGoto(preset)
    term.clear()
    local x, z, alt, hdg
    if preset then
        x, z, alt, hdg = preset.x, preset.z, preset.altitude, preset.heading
    else
        local ok
        x, ok = prompt("X: "); if not ok then return end
        z, ok = prompt("Z: "); if not ok then return end
        alt = prompt("Alt (blank=req cfg): ", true)
        hdg = prompt("Final hdg (blank=none): ", true)
    end
    if not (x and z) then showAck({ accepted = false, reason = "need X and Z" }); return end
    local id = sendCommand({ kind = "goto", x = x, z = z, altitude = alt, heading = hdg })
    showAck(awaitAck(id))
end

local function doWaypointGoto()
    local names = {}
    for name in pairs(waypoints) do names[#names + 1] = name end
    table.sort(names)
    term.clear()
    term.setCursorPos(1, 1)
    if #names == 0 then term.write("no waypoints saved"); sleep(1.0); return end
    for i, name in ipairs(names) do
        term.setCursorPos(1, i + 1)
        local wp = waypoints[name]
        term.write(string.format("%d %s (%s,%s)", i, name, fmt(wp.x), fmt(wp.z)):sub(1, select(1, term.getSize())))
    end
    local sel = prompt("pick #: ")
    local name = sel and names[math.floor(sel)]
    if name then doGoto(waypoints[name]) end
end

local function doSaveWaypoint()
    local _, h = term.getSize()
    term.clear()
    term.setCursorPos(1, h)
    term.write("name: ")
    local name = read()
    if not name or name == "" then return end
    local x = prompt("X: "); local z = prompt("Z: ")
    local alt = prompt("Alt (blank=none): ", true)
    local hdg = prompt("Hdg (blank=none): ", true)
    if not (x and z) then return end
    waypoints[name] = { x = x, z = z, altitude = alt, heading = hdg }
    saveWaypoints(waypoints)
    lastAck = { accepted = true, reason = "saved " .. name }
end

local function console()
    render()
    local ticker = os.startTimer(0.5)
    while true do
        local e = { os.pullEvent() }
        if e[1] == "timer" and e[2] == ticker then
            render()
            ticker = os.startTimer(0.5)
        elseif e[1] == "char" then
            local k = e[2]
            if k == "g" then doGoto()
            elseif k == "w" then doWaypointGoto()
            elseif k == "s" then doSaveWaypoint()
            elseif k == "h" then lastAck = awaitAck(sendCommand({ kind = "hold" }))
            elseif k == "c" then lastAck = awaitAck(sendCommand({ kind = "cancel" }))
            end
            render()
            ticker = os.startTimer(0.5)
        end
    end
end

parallel.waitForAny(relay, console)
