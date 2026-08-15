local Logger = {}
Logger.__index = Logger

local HEADERS = {
    "timestamp_ms",
    "sequence",
    "mode",
    "height",
    "vertical_speed",
    "air_pressure",
    "gimbal_angles",
    "gimbal_rates_rad",
    "linear_acceleration",
    "gravity",
    "velocities",
    "rsc",
    "bearings",
    "navigation",
    "position",
    "errors",
}

local function serialize(value)
    if value == nil then return "" end
    if type(value) == "number" or type(value) == "boolean" then return tostring(value) end
    if type(value) == "string" then return value end
    local ok, result = pcall(textutils.serialize, value, { compact = true })
    return ok and result or tostring(value)
end

local function csv(value)
    local text = serialize(value):gsub('"', '""'):gsub("[\r\n]", " ")
    return '"' .. text .. '"'
end

function Logger.new(config)
    local self = setmetatable({
        enabled = config.logging.enabled,
        file = nil,
        path = nil,
        error = nil,
    }, Logger)
    if not self.enabled then return self end

    local directory = config.logging.directory
    if not fs.exists(directory) then fs.makeDir(directory) end
    self.path = fs.combine(
        directory,
        string.format("flight-%d-%d.csv", os.getComputerID(), os.epoch("utc"))
    )
    self.file = fs.open(self.path, "w")
    if not self.file then
        self.error = "could not open " .. self.path
        self.enabled = false
        return self
    end
    self.file.writeLine(table.concat(HEADERS, ","))
    self.file.flush()
    return self
end

function Logger:write(snapshot)
    if not self.enabled or not self.file then return false, self.error end
    local sensors, actuators = snapshot.sensors, snapshot.actuators
    local altitude, gimbal = sensors.altitude or {}, sensors.gimbal or {}
    local row = {
        snapshot.timestampMs,
        snapshot.sequence,
        snapshot.mode,
        altitude.height,
        altitude.verticalSpeed,
        altitude.airPressure,
        gimbal.getAngles,
        gimbal.getAngularRatesRad,
        gimbal.getLinearAcceleration,
        gimbal.getGravity,
        sensors.velocity,
        actuators.rsc,
        actuators.bearings,
        sensors.navigation,
        sensors.position,
        snapshot.errors,
    }
    for index = 1, #HEADERS do row[index] = csv(row[index]) end
    self.file.writeLine(table.concat(row, ","))
    self.file.flush()
    return true, nil
end

function Logger:close()
    if self.file then
        self.file.close()
        self.file = nil
    end
end

return Logger
