local Display = {}
Display.__index = Display

local function shortName(name)
    return (name or "?")
        :gsub("gyroscopic_propeller_bearing_", "G")
        :gsub("propeller_bearing_", "P")
        :gsub("directional_gearshift_", "D")
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
    local flight = {
        string.format("ID %d  MODE %s", snapshot.computerId, snapshot.mode),
        "Height: " .. self.util.oneLine(altitude.height),
        "Vertical speed: " .. self.util.oneLine(altitude.verticalSpeed),
        "Air pressure: " .. self.util.oneLine(altitude.airPressure),
        "Angles deg: " .. self.util.oneLine(gimbal.getAngles),
        "Angles rad: " .. self.util.oneLine(gimbal.getAnglesRad),
        "Rates: " .. self.util.oneLine(gimbal.getAngularRatesRad),
        "Acceleration: " .. self.util.oneLine(gimbal.getLinearAcceleration),
        "Gravity: " .. self.util.oneLine(gimbal.getGravity),
    }
    for _, velocity in ipairs(sensors.velocity or {}) do
        flight[#flight + 1] = "Velocity " .. self.util.oneLine(velocity.axis) .. ": "
            .. self.util.oneLine(velocity.velocity)
    end
    local navigation = sensors.navigation or {}
    flight[#flight + 1] = "Heading: " .. self.util.oneLine(navigation.getHeading)
    flight[#flight + 1] = "Nav target: " .. self.util.oneLine(navigation.hasTarget)
    if navigation.hasTarget then
        flight[#flight + 1] = "Distance: " .. self.util.oneLine(navigation.getDistanceToTarget)
        flight[#flight + 1] = "Bearing: " .. self.util.oneLine(navigation.getBearing)
        flight[#flight + 1] = "Vertical offset: " .. self.util.oneLine(navigation.getVerticalOffsetToTarget)
    end
    local position = sensors.position
    if position and position.valid then
        flight[#flight + 1] = string.format(
            "Pos: %.1f, %.1f, %.1f",
            position.x or 0, position.y or 0, position.z or 0
        )
    elseif position then
        flight[#flight + 1] = "Pos: " .. (position.reason or "unavailable")
    end
    flight[#flight + 1] = string.format(
        "Loop %.1f ms  Errors %d",
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
        system[#system + 1] = "AUTO alt=" .. self.util.oneLine(ap.targets and ap.targets.altitude)
            .. " hdg=" .. self.util.oneLine(ap.targets and ap.targets.heading)
    end
    if snapshot.telemetryError then system[#system + 1] = "REDNET: " .. snapshot.telemetryError end

    self:draw(self.flightName, "MC AERO - FLIGHT", flight)
    self:draw(self.systemName, "MC AERO - SYSTEMS", system)
end

return Display
