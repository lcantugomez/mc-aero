local Telemetry = {}
Telemetry.__index = Telemetry

function Telemetry.new(config)
    local self = setmetatable({
        config = config,
        enabled = config.telemetry.enabled,
        opened = false,
        error = nil,
    }, Telemetry)

    if not self.enabled then return self end

    local side = config.telemetry.modemSide
    if peripheral.getType(side) ~= "modem" then
        self.error = side .. " is not a directly attached modem"
        return self
    end

    local ok, openError = pcall(rednet.open, side)
    if not ok then
        self.error = tostring(openError)
        return self
    end
    self.opened = true
    return self
end

function Telemetry:broadcast(snapshot)
    if not self.enabled then return true, nil end
    if not self.opened then return false, self.error or "modem is not open" end

    local ok, broadcastError = pcall(
        rednet.broadcast,
        snapshot,
        self.config.telemetry.protocol
    )
    if not ok then return false, tostring(broadcastError) end
    return true, nil
end

function Telemetry:close()
    if self.opened then
        pcall(rednet.close, self.config.telemetry.modemSide)
        self.opened = false
    end
end

return Telemetry
