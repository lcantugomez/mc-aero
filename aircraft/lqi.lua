-- MC Aero — discrete MIMO LQI regulator (gyros ON, roll/pitch left to gyros).
-- Control law: u = u_nominal - K * x_I, evaluated once per tick. K and the plant
-- metadata (state/input ordering, dt, hover nominal) are generated offline by
-- model/build_plant.py and shipped in aircraft/plant_K.lua.
--
-- State x_I (deviation from hover, ordered by meta.stateOrder):
--   [s_x s_y s_z psi  v_x v_y v_z r  w_1..w_5  xi_sx xi_sy xi_sz xi_psi]
-- Body/sensor frame: x aft (forward=-x), y up, z left; yaw psi about +y.
-- Positions s are (current - target); kinematics d s/dt = v. Velocities and yaw
-- rate come straight from the (body-frame) sensors. Actuator states w are
-- command-equivalent (steady = command), propagated with the first-order lag.

local LQI = {}
LQI.__index = LQI

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

-- Raw velocity-sensor axes ARE the body frame (x aft, y up, z left).
local function velocityMap(sensorState)
    local v = { x = 0, y = 0, z = 0 }
    for _, s in ipairs(sensorState.velocity or {}) do
        local axis = tostring(s.axis):lower()
        if v[axis] ~= nil and type(s.velocity) == "number" then v[axis] = s.velocity end
    end
    return v
end

function LQI.new(lc, util, plant)
    local meta = plant.meta or {}
    local self = setmetatable({
        lc = lc,
        util = util,
        K = plant.K,
        meta = meta,
        channels = meta.inputOrder,      -- { mainLift, forwardBack, leftRight, yaw, upDown }
        stateOrder = meta.stateOrder,    -- 17 names
        dt = meta.dt or lc.dt,
        tau = lc.tau,
        limit = lc.limit,
        uNom = {},
        omegaHat = {},
        xi = { s_x = 0, s_y = 0, s_z = 0, psi = 0 },
        targets = { altitude = nil, x = nil, z = nil, heading = nil },
        mode = "manual",
    }, LQI)
    for _, c in ipairs(self.channels) do
        self.uNom[c] = (lc.uNominal or {})[c] or 0
        self.omegaHat[c] = self.uNom[c]
    end
    return self
end

-- Capture setpoints; seed actuator states + hover feedforward for bumpless entry.
function LQI:engage(sensorState, currentMainCmd)
    local alt = sensorState.altitude or {}
    local pos = sensorState.position or {}
    local nav = sensorState.navigation or {}
    self.targets.altitude = tonumber(alt.height)
    self.targets.x = pos.comX
    self.targets.z = pos.comZ
    self.targets.heading = tonumber(nav.getHeading)
    self.xi = { s_x = 0, s_y = 0, s_z = 0, psi = 0 }
    for _, c in ipairs(self.channels) do self.omegaHat[c] = self.uNom[c] or 0 end
    if type(currentMainCmd) == "number" then
        self.uNom.mainLift = currentMainCmd   -- hover feedforward = what held it
        self.omegaHat.mainLift = currentMainCmd
    end
    self.mode = "autopilot"
end

function LQI:disengage()
    self.mode = "manual"
end

-- Clear the integral accumulators (fresh start for a new mission/command, so a
-- stale integral from a previous target can't bias the new one).
function LQI:resetIntegral()
    self.xi = { s_x = 0, s_y = 0, s_z = 0, psi = 0 }
end

-- Update setpoints without touching integrals/actuator states (for the guidance
-- layer to steer the hold point smoothly). Only fields present are changed.
function LQI:setTarget(t)
    if t.altitude ~= nil then self.targets.altitude = t.altitude end
    if t.x ~= nil then self.targets.x = t.x end
    if t.z ~= nil then self.targets.z = t.z end
    if t.heading ~= nil then self.targets.heading = t.heading end
end

function LQI:update(sensorState, dt)
    local lc = self.lc
    dt = dt or self.dt
    local alt = sensorState.altitude or {}
    local nav = sensorState.navigation or {}
    local pos = sensorState.position or {}
    local gim = sensorState.gimbal or {}
    local vel = velocityMap(sensorState)

    local height = tonumber(alt.height)
    local heading = tonumber(nav.getHeading)
    local rates = gim.getAngularRatesRad

    -- ---- tracked-output errors (current - target) ----------------------
    local s_x, s_z = 0, 0
    if lc.enable.position and pos.comX and pos.comZ
        and self.targets.x and self.targets.z and heading then
        local dX = pos.comX - self.targets.x
        local dZ = pos.comZ - self.targets.z
        -- World->body rotation fitted from flight data: bodyVel = Rot(heading)*worldVel
        -- (theta = heading + small offset). This makes d(s)/dt = v (same frame as the
        -- velocity sensors), which is what lets position hold settle instead of spiral.
        local th = math.rad(heading + (lc.headingOffsetDeg or 0))
        local ct, st = math.cos(th), math.sin(th)
        local sgn = lc.positionSign or {}
        s_x = (sgn.forward or 1) * (ct * dX - st * dZ)
        s_z = (sgn.lateral or 1) * (st * dX + ct * dZ)
    end
    local s_y = 0
    if lc.enable.altitude and height and self.targets.altitude then
        s_y = height - self.targets.altitude
    end
    local psi_err = 0
    if lc.enable.heading and heading and self.targets.heading then
        psi_err = wrapPi(math.rad(heading) - math.rad(self.targets.heading))
    end

    -- ---- inner states (always fed) -------------------------------------
    local v_x, v_y, v_z = vel.x or 0, vel.y or 0, vel.z or 0
    local r = 0
    if type(rates) == "table" then
        r = (tonumber(rates[lc.yawRateIndex or 2]) or 0) * (lc.yawRateSign or 1)
    end

    local vals = {
        s_x = s_x, s_y = s_y, s_z = s_z, psi = psi_err,
        v_x = v_x, v_y = v_y, v_z = v_z, r = r,
        xi_sx = self.xi.s_x, xi_sy = self.xi.s_y, xi_sz = self.xi.s_z, xi_psi = self.xi.psi,
    }
    for _, c in ipairs(self.channels) do
        vals["w_" .. c] = (self.omegaHat[c] or 0) - (self.uNom[c] or 0)
    end

    -- assemble x by the model's state ordering
    local x = {}
    for j, name in ipairs(self.stateOrder) do x[j] = vals[name] or 0 end

    -- ---- control law: u = u_nom - K x ----------------------------------
    local u = {}
    for i, c in ipairs(self.channels) do
        local Krow = self.K[i]
        local acc = 0
        for j = 1, #x do acc = acc + Krow[j] * x[j] end
        local raw = (self.uNom[c] or 0) - acc
        local lim = self.limit[c]
        local lo, hi
        if type(lim) == "table" then lo, hi = lim[1], lim[2] else lo, hi = -lim, lim end
        u[c] = clamp(raw, lo, hi)
    end

    -- ---- integrate tracked outputs (regulator: d xi/dt = -s) -----------
    -- Conditional integration (anti-windup): only integrate an axis while its
    -- primary actuator is NOT saturated. A floored climb/cruise then cannot wind
    -- up, but a normal (unsaturated) approach still integrates out steady error.
    local function satC(c)
        local lim = self.limit[c]
        local lo, hi
        if type(lim) == "table" then lo, hi = lim[1], lim[2] else lo, hi = -lim, lim end
        local v = u[c] or 0
        return v >= (hi - 1e-6) or v <= (lo + 1e-6)
    end
    local xm = lc.xiMax or {}
    if dt > 0 then
        if not satC("mainLift") then self.xi.s_y = clamp(self.xi.s_y - s_y * dt, -(xm.s_y or 1e9), (xm.s_y or 1e9)) end
        if not satC("forwardBack") then self.xi.s_x = clamp(self.xi.s_x - s_x * dt, -(xm.s_x or 1e9), (xm.s_x or 1e9)) end
        if not satC("leftRight") then self.xi.s_z = clamp(self.xi.s_z - s_z * dt, -(xm.s_z or 1e9), (xm.s_z or 1e9)) end
        if not satC("yaw") then self.xi.psi = clamp(self.xi.psi - psi_err * dt, -(xm.psi or 1e9), (xm.psi or 1e9)) end
    end

    -- ---- propagate command-equivalent actuator states -----------------
    for _, c in ipairs(self.channels) do
        local tau = self.tau[c] or 0.3
        local a = math.exp(-dt / tau)
        self.omegaHat[c] = a * (self.omegaHat[c] or 0) + (1 - a) * u[c]
    end

    local axisTargets = {
        mainLift = u.mainLift, forwardBack = u.forwardBack,
        leftRight = u.leftRight, yaw = u.yaw, upDown = u.upDown,
    }
    local telemetry = {
        mode = self.mode,
        controller = "lqi",
        provisionalSigns = self.meta.provisionalSigns,
        targets = {
            altitude = self.targets.altitude, x = self.targets.x,
            z = self.targets.z, heading = self.targets.heading,
        },
        errors = { s_x = s_x, s_y = s_y, s_z = s_z, psi = psi_err,
                   v_x = v_x, v_y = v_y, v_z = v_z, r = r },
        integral = { s_x = self.xi.s_x, s_y = self.xi.s_y, s_z = self.xi.s_z, psi = self.xi.psi },
        command = axisTargets,
        enable = lc.enable,
    }
    return axisTargets, telemetry
end

return LQI
