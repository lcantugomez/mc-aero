-- MC Aero telemetry relay
-- Runs on a ground CC:Tweaked computer with a wireless/ender modem.
-- Receives aircraft telemetry over rednet and POSTs it to an HTTPS endpoint
-- (e.g. an AWS Lambda Function URL) in small JSON batches.
--
-- The flight computer is untouched; all network latency lives here.
-- Keep this computer chunk-loaded so it keeps running.

-- ---- configuration -------------------------------------------------------
local ENDPOINT = "https://REPLACE_ME.lambda-url.us-east-1.on.aws/"
local API_KEY = "REPLACE_WITH_A_LONG_RANDOM_SECRET"
local PROTOCOL = "mc_aero.telemetry.v1"

local FLUSH_INTERVAL = 1.0 -- seconds between POSTs
local MAX_BATCH = 40 -- records per POST
local MAX_BUFFER = 400 -- cap buffered records (bounds memory)

-- Keep secrets out of the committed script: optionally override from a
-- local file that you do NOT commit.
--   /relay_config.lua ->  return { endpoint = "...", apiKey = "...", protocol = "..." }
if fs.exists("/relay_config.lua") then
    local ok, override = pcall(dofile, "/relay_config.lua")
    if ok and type(override) == "table" then
        ENDPOINT = override.endpoint or ENDPOINT
        API_KEY = override.apiKey or API_KEY
        PROTOCOL = override.protocol or PROTOCOL
    else
        print("relay: ignoring invalid /relay_config.lua")
    end
end
-- --------------------------------------------------------------------------

if not http then error("HTTP is disabled in the CC:Tweaked config", 0) end

local function openModem()
    local fallback
    for _, name in ipairs(peripheral.getNames()) do
        if peripheral.getType(name) == "modem" then
            local wireless = peripheral.call(name, "isWireless")
            if wireless then
                rednet.open(name)
                return name, true
            end
            fallback = fallback or name
        end
    end
    if fallback then
        rednet.open(fallback)
        return fallback, false
    end
    error("no modem attached to this computer", 0)
end

local modemName, wireless = openModem()
print(string.format("relay: %s modem on %s", wireless and "wireless" or "wired", modemName))
print("relay: protocol " .. PROTOCOL)
print("relay: endpoint " .. ENDPOINT)

local buffer = {}
local sending = nil -- batch currently in flight (nil = idle)

local function drop(fromFront)
    while #buffer > MAX_BUFFER do
        table.remove(buffer, fromFront and 1 or #buffer)
    end
end

local function tryFlush()
    if sending or #buffer == 0 then return end

    local batch = {}
    local take = math.min(MAX_BATCH, #buffer)
    for i = 1, take do batch[i] = buffer[i] end

    local remaining = {}
    for i = take + 1, #buffer do remaining[#remaining + 1] = buffer[i] end
    buffer = remaining

    local ok, body = pcall(textutils.serializeJSON, batch)
    if not ok then
        print("relay: JSON encode failed, dropping " .. take .. " records")
        return
    end

    sending = batch
    http.request({
        url = ENDPOINT,
        method = "POST",
        headers = {
            ["Content-Type"] = "application/json",
            ["x-api-key"] = API_KEY,
        },
        body = body,
    })
end

local flushTimer = os.startTimer(FLUSH_INTERVAL)

while true do
    local event = { os.pullEvent() }
    local kind = event[1]

    if kind == "rednet_message" then
        local _, message, protocol = event[2], event[3], event[4]
        if protocol == PROTOCOL and type(message) == "table" then
            buffer[#buffer + 1] = message
            drop(true)
            if #buffer >= MAX_BATCH then tryFlush() end
        end
    elseif kind == "timer" and event[2] == flushTimer then
        tryFlush()
        flushTimer = os.startTimer(FLUSH_INTERVAL)
    elseif kind == "http_success" then
        local url, handle = event[2], event[3]
        if url == ENDPOINT then
            if handle then
                handle.readAll()
                handle.close()
            end
            sending = nil
        end
    elseif kind == "http_failure" then
        local url, reason, handle = event[2], event[3], event[4]
        if url == ENDPOINT then
            if handle then handle.close() end
            print("relay: POST failed: " .. tostring(reason))
            -- requeue the failed batch at the front, then re-cap
            if sending then
                for i = #sending, 1, -1 do
                    table.insert(buffer, 1, sending[i])
                end
                drop(false)
                sending = nil
            end
        end
    end
end
