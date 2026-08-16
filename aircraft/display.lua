local Display = {}
Display.__index = Display

local function shortName(name)
    return (name or "?")
        :gsub("gyroscopic_propeller_bearing_", "G")
        :gsub("propeller_bearing_", "P")
        :gsub("directional_gearshift_", "D")
end

-- Format a number to fixed decimals, or "--" when missing/non-numeric.
local function fmt(v, dec)
    v = tonumber(v)
    if not v then return "--" end
    return string.format("%." .. (dec or 1) .. "f", v)
end

-- Extract x,y,z from a vector table (keyed {x,y,z} or array {1,2,3}).
local function xyz(v)
    if type(v) ~= "table" then return nil, nil, nil end
    local x = v.x; if x == nil then x = v[1] end
    local y = v.y; if y == nil then y = v[2] end
    local z = v.z; if z == nil then z = v[3] end
    return tonumber(x), tonumber(y), tonumber(z)
end

function Display.new(config, util)
    local self = setmetatable({ config = config, util = util, warnings = {} }, Display)
    self.flightName = config.peripherals.flightMonitor
    self.systemName = config.peripherals.systemMonitor

    for _, item in ipairs({
        { self.flightName, "flight monitor" },
        { self.systemName, "system monitor" },
    }) do
        local ok, reason = util.checkPeripheral(item[1], "monitor")
        if not ok then self.warnings[#self.warnings + 1] = item[2] .. ": " .. reason end
    end

    for _, name in ipairs({ self.flightName, self.systemName }) do
        util.call(name, "setTextScale", config.display.textScale)
        util.call(name, "setBackgroundColor", colors.black)
        util.call(name, "setTextColor", colors.white)
        util.call(name, "clear")
    end
    return self
end

function Display:draw(name, title, lines)
    if not peripheral.isPresent(name) then return end
    local size, sizeError = self.util.call(name, "getSize")
    if sizeError or type(size) ~= "table" then return end
    local width, height = tonumber(size[1]) or 0, tonumber(size[2]) or 0

    self.util.call(name, "setBackgroundColor", colors.black)
    self.util.call(name, "setTextColor", colors.white)
    self.util.call(name, "clear")
    self.util.call(name, "setCursorPos", 1, 1)
    self.util.call(name, "setTextColor", colors.cyan)
    self.util.call(name, "write", title:sub(1, width))
    self.util.call(name, "setTextColor", colors.white)

    for index, line in ipairs(lines) do
        local row = index + 1
        if row > height then break end
        self.util.call(name, "setCursorPos", 1, row)
        self.util.call(name, "write", tostring(line):sub(1, width))
    end
end

function Display:update(snapshot)
    local sensors, actuators = snapshot.sensors, snapshot.actuators
    local altitude = sensors.altitude or {}
    local gimbal = sensors.gimbal or {}
    local navigation = sensors.navigation or {}
    local position = sensors.position or {}
    local ap = snapshot.autopilot
    local u = self.util
    local cx, cy, cz = xyz(snapshot.centerOfMass)
    local hdg = tonumber(navigation.getHeading)

    local flight = {
        string.format("ID %d  MODE %s", snapshot.computerId, snapshot.mode),
    }

    -- Controller dashboard: current > target (error). Falls back to raw state
    -- in manual mode where there are no setpoints.
    if ap and ap.targets then
        local t = ap.targets
        local hErr = ap.heading and math.deg(ap.heading.headingError or 0) or nil
        local aErr = ap.altitude and ap.altitude.error or nil
        flight[#flight + 1] = string.format("HDG %s > %s e %s", fmt(hdg), fmt(t.heading), fmt(hErr))
        flight[#flight + 1] = string.format("ALT %s > %s e %s", fmt(altitude.height), fmt(t.altitude), fmt(aErr))
        if ap.pressure then
            flight[#flight + 1] = string.format(
                "P %s  gain x%s", fmt(ap.pressure.raw, 3), fmt(ap.pressure.gainScale, 2)
            )
        end
    else
        flight[#flight + 1] = "HDG " .. fmt(hdg)
        flight[#flight + 1] = "ALT " .. fmt(altitude.height) .. " VS " .. fmt(altitude.verticalSpeed)
    end

    -- Position: nav sensor (p_nav) vs center of mass (p_com), side by side.
    flight[#flight + 1] = "POS   nav       com"
    flight[#flight + 1] = string.format("X %s  %s", fmt(position.valid and position.x), fmt(cx))
    flight[#flight + 1] = string.format("Y %s  %s", fmt(position.y), fmt(cy))
    flight[#flight + 1] = string.format("Z %s  %s", fmt(position.valid and position.z), fmt(cz))
    if ap and ap.targets and (ap.targets.x or ap.targets.z) then
        flight[#flight + 1] = string.format("TGT X %s Z %s", fmt(ap.targets.x), fmt(ap.targets.z))
    end
    if ap and ap.horizontal then
        flight[#flight + 1] = string.format(
            "ERR X %s Z %s", fmt(ap.horizontal.xError), fmt(ap.horizontal.zError)
        )
    elseif position.valid ~= true and position.reason then
        flight[#flight + 1] = "POS " .. position.reason
    end

    -- Velocity (world x/z + vertical) and attitude detail.
    local vx, vz
    for _, velocity in ipairs(sensors.velocity or {}) do
        local a = tostring(velocity.axis):lower()
        if a == "x" then vx = velocity.velocity elseif a == "z" then vz = velocity.velocity end
    end
    flight[#flight + 1] = string.format("VEL x %s z %s vy %s", fmt(vx), fmt(vz), fmt(altitude.verticalSpeed))
    flight[#flight + 1] = "Angles: " .. u.oneLine(gimbal.getAngles)
    flight[#flight + 1] = "Rates: " .. u.oneLine(gimbal.getAngularRatesRad)
    flight[#flight + 1] = "Accel: " .. u.oneLine(gimbal.getLinearAcceleration)
    if navigation.hasTarget then
        flight[#flight + 1] = string.format(
            "Nav d %s brg %s",
            u.oneLine(navigation.getDistanceToTarget),
            u.oneLine(navigation.getBearing)
        )
    end
    flight[#flight + 1] = string.format(
        "Loop %.1f ms  Err %d",
        snapshot.loopDurationMs or 0,
        snapshot.errorCount or 0
    )

    local rscState = actuators.rsc or {}
    local system = {
        "MAIN LIFT +/- " .. tostring(self.config.manual.liftStep) .. " RPM (Up/Down)",
        "--- RSC  target / actual ---",
    }
    for _, axis in ipairs({ "mainLift", "upDown", "forwardBack", "leftRight", "yaw" }) do
        local r = rscState[axis] or {}
        system[#system + 1] = string.format(
            "%-11s %s / %s%s",
            axis,
            self.util.oneLine(r.getTargetSpeed),
            self.util.oneLine(r.getSpeed),
            r.isOverstressed == true and " OVS" or ""
        )
    end
    system[#system + 1] = "--- PROPELLERS  thrust / rpm ---"
    for _, bearing in ipairs(actuators.bearings or {}) do
        system[#system + 1] = string.format(
            "%s %s / %s",
            bearing.role or shortName(bearing.name),
            self.util.oneLine(bearing.getThrust),
            self.util.oneLine(bearing.getRotationSpeed)
        )
    end
    if snapshot.manualInput then
        system[#system + 1] = "Input delta: " .. self.util.oneLine(snapshot.manualInput.delta)
    end
    if snapshot.autopilot then
        local ap = snapshot.autopilot
        local t = ap.targets or {}
        system[#system + 1] = "AUTO alt=" .. self.util.oneLine(t.altitude)
            .. " hdg=" .. self.util.oneLine(t.heading)
        system[#system + 1] = "AUTO tgt x/z: " .. self.util.oneLine(t.x)
            .. " / " .. self.util.oneLine(t.z)
        if ap.horizontal then
            system[#system + 1] = "AUTO err x/z: " .. self.util.oneLine(ap.horizontal.xError)
                .. " / " .. self.util.oneLine(ap.horizontal.zError)
        end
    end
    if snapshot.telemetryError then system[#system + 1] = "REDNET: " .. snapshot.telemetryError end

    self:draw(self.flightName, "MC AERO - FLIGHT", flight)
    self:draw(self.systemName, "MC AERO - SYSTEMS", system)
end

return Display
