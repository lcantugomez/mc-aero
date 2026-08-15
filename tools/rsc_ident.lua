-- rsc_ident.lua : identify which control axis a Rotation Speed Controller drives.
-- Pulses ONE named RSC briefly, reports which bearings spin up and which motion
-- axis responds, then restores the RSC's original target.
--
-- USE IN A STABLE HOVER, ready to Ctrl+T. It will move the craft a little.
--
-- Usage: rsc_ident <RSC_name> [speed] [seconds]
--   e.g. rsc_ident Create_RotationSpeedController_1 16 2

local name, speedArg, durArg = ...
if not name then
    print("Usage: rsc_ident <RSC_name> [speed=16] [seconds=2]")
    print("Attached RSCs:")
    for _, n in ipairs(peripheral.getNames()) do
        if peripheral.getType(n) == "Create_RotationSpeedController" then print("  " .. n) end
    end
    return
end
if peripheral.getType(name) ~= "Create_RotationSpeedController" then
    error(name .. " is not a Rotation Speed Controller", 0)
end

local speed = tonumber(speedArg) or 16
local duration = tonumber(durArg) or 2

-- discover sensors/bearings
local bearings, velocitySensors, gimbal = {}, {}, nil
for _, n in ipairs(peripheral.getNames()) do
    local t = peripheral.getType(n)
    if t == "gyroscopic_propeller_bearing" or t == "propeller_bearing" then
        bearings[#bearings + 1] = n
    elseif t == "velocity_sensor" then
        velocitySensors[#velocitySensors + 1] = n
    elseif t == "gimbal_sensor" then
        gimbal = n
    end
end

local function num(v) return type(v) == "number" and v or 0 end
local function callNum(nm, method) return num(select(2, pcall(peripheral.call, nm, method))) end

local function sample()
    local rpm = {}
    for _, b in ipairs(bearings) do rpm[b] = callNum(b, "getRotationSpeed") end
    local vel = { x = 0, y = 0, z = 0 }
    for _, s in ipairs(velocitySensors) do
        local axis = tostring(select(2, pcall(peripheral.call, s, "getAxis"))):lower()
        if vel[axis] ~= nil then vel[axis] = callNum(s, "getVelocity") end
    end
    local rates = { 0, 0, 0 }
    if gimbal then
        local ok, r = pcall(peripheral.call, gimbal, "getAngularRates")
        if ok and type(r) == "table" then rates = { num(r[1]), num(r[2]), num(r[3]) } end
    end
    return { rpm = rpm, vel = vel, rates = rates }
end

local original = callNum(name, "getTargetSpeed")
print(string.format("Identifying %s  (pulse %.0f for %.1fs, then restore to %.0f)", name, speed, duration, original))

local before = sample()
local peak = { vel = { x = 0, y = 0, z = 0 }, rates = { 0, 0, 0 }, rpm = {} }

local ok = pcall(function()
    pcall(peripheral.call, name, "setTargetSpeed", speed)
    local deadline = os.clock() + duration
    while os.clock() < deadline do
        local s = sample()
        for a in pairs(peak.vel) do
            if math.abs(s.vel[a] - before.vel[a]) > math.abs(peak.vel[a]) then peak.vel[a] = s.vel[a] - before.vel[a] end
        end
        for i = 1, 3 do
            if math.abs(s.rates[i] - before.rates[i]) > math.abs(peak.rates[i]) then peak.rates[i] = s.rates[i] - before.rates[i] end
        end
        for _, b in ipairs(bearings) do
            local d = s.rpm[b] - before.rpm[b]
            if math.abs(d) > math.abs(peak.rpm[b] or 0) then peak.rpm[b] = d end
        end
        sleep(0.1)
    end
end)

-- always restore
pcall(peripheral.call, name, "setTargetSpeed", original)

print("--- bearings that responded (rpm delta) ---")
for _, b in ipairs(bearings) do
    local d = peak.rpm[b] or 0
    if math.abs(d) > 0.05 then print(string.format("  %-32s %+0.2f", b, d)) end
end
print(string.format("peak velocity delta: x=%+.2f y=%+.2f z=%+.2f", peak.vel.x, peak.vel.y, peak.vel.z))
print(string.format("peak ang-rate delta: p=%+.3f q=%+.3f r=%+.3f", peak.rates[1], peak.rates[2], peak.rates[3]))
print("restored target to " .. original .. (ok and "" or "  (interrupted)"))
