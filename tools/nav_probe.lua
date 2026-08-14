-- nav_probe.lua : inspect the Create Aeronautics navigation table live.
-- Optional arg: peripheral name (defaults to navigation_table_0, else autodetect).
-- Run on the aircraft computer. Ctrl+T to stop.

local name = ...
if not name or not peripheral.isPresent(name) then
    name = nil
    for _, n in ipairs(peripheral.getNames()) do
        if peripheral.getType(n):find("navigation") then name = n break end
    end
end
if not name or not peripheral.isPresent(name) then
    error("no navigation table found (pass its name as an argument)", 0)
end

local function show(v)
    if type(v) == "table" then
        local ok, s = pcall(textutils.serialize, v, { compact = true })
        return ok and s or "{table}"
    end
    return tostring(v)
end

local methods = peripheral.getMethods(name)
print("peripheral : " .. name .. "  (" .. peripheral.getType(name) .. ")")
print("methods    : " .. table.concat(methods, ", "))
print("----- full getter dump -----")

local present = {}
for _, m in ipairs(methods) do present[m] = true end

-- Only call read-style methods (getters), so we never trigger a setter.
for _, m in ipairs(methods) do
    if m:match("^get") or m:match("^is") or m:match("^has") then
        local r = { pcall(peripheral.call, name, m) }
        local ok = table.remove(r, 1)
        if ok then
            local parts = {}
            for i = 1, #r do parts[i] = show(r[i]) end
            print(string.format("  %-28s -> %s", m, #parts > 0 and table.concat(parts, ", ") or "(nil)"))
        else
            print(string.format("  %-28s !! %s", m, tostring(r[1])))
        end
    end
end

-- Live watch: altitude vs vertical offset, so you can climb and compare.
print("----- live watch (Ctrl+T to stop) -----")
local altName
for _, n in ipairs(peripheral.getNames()) do
    if peripheral.getType(n):find("altitude") then altName = n break end
end

local function callOr(nm, method)
    if not nm then return "n/a" end
    local r = { pcall(peripheral.call, nm, method) }
    if r[1] then return show(r[2]) end
    return "err"
end

while true do
    local gx, gy, gz = gps.locate(0.4)
    local line = {
        "alt_height=" .. (altName and callOr(altName, "getHeight") or "n/a"),
        "gpsY=" .. tostring(gy),
    }
    if present.hasTarget then line[#line + 1] = "hasTarget=" .. callOr(name, "hasTarget") end
    if present.getVerticalOffsetToTarget then
        line[#line + 1] = "vOffset=" .. callOr(name, "getVerticalOffsetToTarget")
    end
    if present.getDistanceToTarget then line[#line + 1] = "dist=" .. callOr(name, "getDistanceToTarget") end
    if present.getBearing then line[#line + 1] = "bearing=" .. callOr(name, "getBearing") end
    print(table.concat(line, "  "))
    sleep(1.0)
end
