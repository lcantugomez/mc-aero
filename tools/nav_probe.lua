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

-- Fill in your lodestone's world coordinates (F3 on the lodestone block).
local LODESTONE = { x = 0, y = 0, z = 0 }

local atan2 = math.atan2 or function(y, x) return math.atan(y, x) end

local function numOf(nm, method)
    if not nm then return nil end
    local r = { pcall(peripheral.call, nm, method) }
    if r[1] and type(r[2]) == "number" then return r[2] end
    return nil
end

while true do
    local gx, gy, gz = gps.locate(0.4)
    local dist = present.getDistanceToTarget and numOf(name, "getDistanceToTarget") or nil
    local voff = present.getVerticalOffsetToTarget and numOf(name, "getVerticalOffsetToTarget") or nil
    local bearing = present.getBearing and numOf(name, "getBearing") or nil
    local height = altName and numOf(altName, "getHeight") or nil

    local line = {
        string.format("gps=(%s,%s,%s)", tostring(gx), tostring(gy), tostring(gz)),
        "height=" .. tostring(height),
        "dist=" .. tostring(dist),
        "vOff=" .. tostring(voff),
        "bearing=" .. tostring(bearing),
    }
    -- horizontal range from the slant/vertical triangle
    if dist and voff then
        local h2 = dist * dist - voff * voff
        line[#line + 1] = string.format("horizNav=%.2f", h2 > 0 and math.sqrt(h2) or 0)
    end
    -- truth from GPS: horizontal range + geometric bearing to the lodestone,
    -- so we can compare against the nav table's bearing and lock the convention.
    if gx and gz then
        local dxu, dzu = LODESTONE.x - gx, LODESTONE.z - gz
        line[#line + 1] = string.format("horizGPS=%.2f", math.sqrt(dxu * dxu + dzu * dzu))
        line[#line + 1] = string.format("trueBrgXZ=%.1f", math.deg(atan2(dxu, dzu)))
    end
    print(table.concat(line, "  "))
    sleep(1.0)
end
