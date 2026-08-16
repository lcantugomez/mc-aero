-- MC Aero pocket ground station.
-- Runs on a POCKET computer with a wireless/ender modem, carried on or near the
-- aircraft. Receives telemetry over rednet and relays it to the S3 endpoint,
-- with a compact live display on the pocket screen. Needs HTTP enabled.
--
-- Config: copy ground/relay_config.example.lua to /pocket_config.lua (or
-- /relay_config.lua) with your endpoint + apiKey.

local CONFIG = {
    protocol = "mc_aero.telemetry.v1",
    endpoint = "https://REPLACE_ME.lambda-url.us-east-2.on.aws/",
    apiKey = "REPLACE_WITH_SECRET",
    flushInterval = 1.0,
    maxBatch = 40,
    maxBuffer = 200,
    staleMs = 1500,
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
end

if not http then error("HTTP is disabled in the CC:Tweaked config", 0) end

local function openModem()
    for _, name in ipairs(peripheral.getNames()) do
        if peripheral.getType(name) == "modem" then
            rednet.open(name)
            return name
        end
    end
    error("no modem on this pocket computer (needs a wireless/ender modem)", 0)
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

-- ---- state --------------------------------------------------------------
local latest, lastRxMs = nil, 0
local buffer, sending = {}, nil
local posted, failed = 0, 0

local function render()
    local w = select(1, term.getSize())
    term.setBackgroundColor(colors.black)
    term.setTextColor(colors.white)
    term.clear()
    local function line(row, s)
        term.setCursorPos(1, row)
        term.write(tostring(s):sub(1, w))
    end

    local age = latest and (nowMs() - lastRxMs) or nil
    local status = (not latest and "WAIT")
        or (age and age > CONFIG.staleMs and ("STALE " .. age .. "ms"))
        or ("OK " .. (age or 0) .. "ms")

    line(1, "MC AERO POCKET")
    line(2, "link: " .. status)
    if latest then
        line(3, "mode: " .. tostring(latest.mode))
        line(4, "lift: " .. fmt(get(latest, "actuators", "rsc", "mainLift", "getTargetSpeed"))
            .. " / " .. fmt(get(latest, "actuators", "rsc", "mainLift", "getSpeed")))
        local pos = get(latest, "sensors", "position")
        if pos and pos.valid then
            line(5, string.format("pos: %s,%s,%s", fmt(pos.x), fmt(pos.y), fmt(pos.z)))
        else
            line(5, "pos: -")
        end
        line(6, "vs: " .. fmt(get(latest, "sensors", "altitude", "verticalSpeed"))
            .. "  hdg: " .. fmt(get(latest, "sensors", "navigation", "getHeading")))
        line(7, "err: " .. tostring(latest.errorCount or 0))
        if latest.sweep then
            line(8, "SWEEP " .. tostring(latest.sweep.axis) .. " -> " .. fmt(latest.sweep.target))
        end
    end
    line(select(2, term.getSize()), string.format("S3 ok:%d err:%d", posted, failed))
end

local function tryFlush()
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

render()
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
            buffer[#buffer + 1] = message
            while #buffer > CONFIG.maxBuffer do table.remove(buffer, 1) end
            if #buffer >= CONFIG.maxBatch then tryFlush() end
            render()
        end
    elseif kind == "timer" and event[2] == flushTimer then
        tryFlush()
        flushTimer = os.startTimer(CONFIG.flushInterval)
    elseif kind == "timer" and event[2] == uiTimer then
        render()
        uiTimer = os.startTimer(0.5)
    elseif kind == "http_success" then
        local url, handle = event[2], event[3]
        if url == CONFIG.endpoint then
            if handle then handle.readAll(); handle.close() end
            posted = posted + 1
            sending = nil
        end
    elseif kind == "http_failure" then
        local url, _, handle = event[2], event[3], event[4]
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
