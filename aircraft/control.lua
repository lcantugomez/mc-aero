-- MC Aero autopilot controller (gyros ON): cascaded altitude / horizontal /
-- heading hold, pressure-scheduled gains, feedforward hover, anti-windup.
-- Produces axisTargets for actuators:apply("autopilot", { axisTargets = ... }).

local Control = {}
Control.__index = Control

local function clamp(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

local function wrapPi(a)
    while a > math.pi do a = a - 2 * math.pi end
    while a < -math.pi do a = a + 2 * math.pi end
    return a
end

local function velocityMap(sensorState)
    local v = { x = 0, y = 0, z = 0 }
    for _, s in ipairs(sensorState.velocity or {}) do
        local axis = tostring(s.axis):lower()
        if v[axis] ~= nil and type(s.velocity) == "number" then v[axis] = s.velocity end
    end
    return v
end

function Control.new(cc, util)
    return setmetatable({
        cc = cc,
        util = util,
        mode = "manual",
        targets = { altitude = nil, x = nil, z = nil, heading = nil },
        ih = 0, -- altitude integral
    }, Control)
end

local function pressureScale(self, altitude)
    local p = tonumber(altitude.airPressure) or self.cc.referencePressure
    return clamp(p, self.cc.pressure.minScale, self.cc.pressure.maxScale)
end

local function hoverFeedforward(self, p)
    return (self.cc.mass * self.cc.gravity) / (self.cc.mainLiftForcePerCmd100 * p)
end

-- Capture setpoints and seed the integral for a bumpless manual->auto transfer.
function Control:engage(sensorState, currentMainCmd)
    local alt = sensorState.altitude or {}
    local pos = sensorState.position or {}
    local nav = sensorState.navigation or {}
    self.targets.altitude = tonumber(alt.height)
    self.targets.x = (pos.valid and pos.x) or nil
    self.targets.z = (pos.valid and pos.z) or nil
    self.targets.heading = tonumber(nav.getHeading)

    local a = self.cc.altitude
    local p = pressureScale(self, alt)
    local uHover = hoverFeedforward(self, p)
    if a.Kih and a.Kih > 0 and type(currentMainCmd) == "number" then
        self.ih = clamp((currentMainCmd - uHover) / a.Kih, a.integralMin, a.integralMax)
    else
        self.ih = 0
    end
    self.mode = "autopilot"
end

function Control:disengage()
    self.mode = "manual"
end

function Control:update(sensorState, dt)
    local cc = self.cc
    local alt = sensorState.altitude or {}
    local nav = sensorState.navigation or {}
    local pos = sensorState.position or {}
    local rates = (sensorState.gimbal or {}).getAngularRatesRad
    local vel = velocityMap(sensorState)

    local p = pressureScale(self, alt)
    local gainScale = 1.0 / p

    local axis = {}
    local telemetry = {
        mode = self.mode,
        pressure = { raw = alt.airPressure, scale = p, gainScale = gainScale },
        targets = {
            altitude = self.targets.altitude,
            x = self.targets.x,
            z = self.targets.z,
            heading = self.targets.heading,
        },
    }

    -- ---- altitude hold (main lift) --------------------------------------
    local a = cc.altitude
    local h = tonumber(alt.height)
    local vy = tonumber(alt.verticalSpeed) or 0
    local uHover = hoverFeedforward(self, p)
    local uMain
    if cc.enable.altitude and h and self.targets.altitude then
        local eh = self.targets.altitude - h
        local vyCmd = clamp(a.Kh * eh, -a.vyCmdMax, a.vyCmdMax)
        local ev = vyCmd - vy
        local Kvy = a.Kvy100 * gainScale
        local raw = uHover + Kvy * ev + a.Kih * self.ih
        uMain = clamp(raw, a.mainLiftMin, a.mainLiftMax)
        local saturated = math.abs(uMain - raw) > 1e-9
        local pushingOut = (uMain >= a.mainLiftMax and eh > 0) or (uMain <= a.mainLiftMin and eh < 0)
        if (not saturated) or (not pushingOut) then
            self.ih = clamp(self.ih + eh * dt, a.integralMin, a.integralMax)
        end
        telemetry.altitude = {
            error = eh, vyCommand = vyCmd, velocityError = ev,
            hoverFeedforward = uHover, Kvy = Kvy, integral = self.ih,
            rawCommand = raw, command = uMain, saturated = saturated,
        }
    else
        uMain = clamp(uHover + a.Kih * self.ih, a.mainLiftMin, a.mainLiftMax)
        telemetry.altitude = { hoverFeedforward = uHover, command = uMain, disabled = true }
    end
    axis.mainLift = uMain
    axis.upDown = 0 -- reserved (spec section 11.6)

    -- ---- horizontal position hold --------------------------------------
    local hz = cc.horizontal
    local headingDeg = tonumber(nav.getHeading)
    local uFB, uLR = 0, 0
    if cc.enable.horizontal and headingDeg and pos.valid and self.targets.x and self.targets.z then
        local eX = self.targets.x - pos.x
        local eZ = self.targets.z - pos.z
        local VxCmd = clamp(hz.Kpos * eX, -hz.velocityCmdMax, hz.velocityCmdMax)
        local VzCmd = clamp(hz.Kpos * eZ, -hz.velocityCmdMax, hz.velocityCmdMax)
        local eVX = VxCmd - vel.x
        local eVZ = VzCmd - vel.z
        local psi = math.rad(headingDeg)
        local eVf = eVX * math.sin(psi) + eVZ * math.cos(psi)
        local eVl = eVX * math.cos(psi) - eVZ * math.sin(psi)
        local Kvf = hz.Kvf100 * gainScale
        local Kvl = hz.Kvl100 * gainScale
        uFB = clamp(Kvf * eVf, -hz.forwardBackMax, hz.forwardBackMax)
        uLR = clamp(Kvl * eVl, -hz.leftRightMax, hz.leftRightMax)
        telemetry.horizontal = {
            xError = eX, zError = eZ, vxCommand = VxCmd, vzCommand = VzCmd,
            bodyForwardVelocityError = eVf, bodyLateralVelocityError = eVl,
            Kvf = Kvf, Kvl = Kvl, forwardBackCommand = uFB, leftRightCommand = uLR,
        }
    end
    axis.forwardBack = uFB
    axis.leftRight = uLR

    -- ---- heading hold ---------------------------------------------------
    local hd = cc.heading
    local uYaw = 0
    if cc.enable.heading and headingDeg and self.targets.heading then
        local epsi = wrapPi(math.rad(self.targets.heading) - math.rad(headingDeg))
        local rCmd = clamp(hd.Kpsi * epsi, -hd.yawRateCmdMax, hd.yawRateCmdMax)
        local r = 0
        if type(rates) == "table" then r = tonumber(rates[hd.yawRateIndex or 2]) or 0 end
        r = r * (hd.yawRateSign or 1)
        local erate = rCmd - r
        uYaw = clamp((hd.KrYaw or 0) * erate, -hd.yawMax, hd.yawMax)
        telemetry.heading = {
            headingError = epsi, yawRateCommand = rCmd, yawRateError = erate, yawCommand = uYaw,
        }
    end
    axis.yaw = uYaw

    return axis, telemetry
end

return Control
