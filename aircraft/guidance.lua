-- MC Aero — go-to-coordinate guidance (reference generator for the LQI).
-- Turns a goal {x, z, altitude, heading} into an instantaneous setpoint that the
-- LQI tracks, sequencing: CLIMB to a safe altitude, ORIENT nose-on to the goal,
-- CRUISE straight in with a decel-limited moving setpoint, then DESCEND onto the
-- target and HOLD. One controller/gain throughout; behaviour comes from the
-- setpoint + commanded heading, not from swapping matrices.

local Guidance = {}
Guidance.__index = Guidance

local function clamp(x, lo, hi)
    if x < lo then return lo elseif x > hi then return hi else return x end
end

local function wrapDeg(a)
    while a > 180 do a = a - 360 end
    while a < -180 do a = a + 360 end
    return a
end

function Guidance.new(cfg)
    return setmetatable({
        cfg = cfg,
        state = "hold",
        goal = nil,
        launch = { x = 0, z = 0 },
        sp = { x = 0, z = 0, altitude = 0, heading = 0 },
    }, Guidance)
end

-- World heading whose FORWARD travel axis points at (tx,tz) from (cx,cz).
-- From the fitted rotation (bodyVel = Rot(heading)*worldVel) and forward = -body_x,
-- the forward-world direction is (-cos th, sin th) with th = rad(heading+offset),
-- so heading = deg(atan2(gz, -gx)) - offset.
function Guidance:bearingTo(cx, cz, tx, tz)
    local gx, gz = tx - cx, tz - cz
    local off = self.cfg.headingOffsetDeg or -1.5
    local hdg = math.deg(math.atan2(gz, -gx)) - off
    if self.cfg.bearingFlip then hdg = hdg + 180 end
    return wrapDeg(hdg)
end

-- Park the setpoint at a fixed point (hover-and-hold). Used on cancel/idle.
function Guidance:hold(x, z, altitude, heading)
    self.state = "hold"
    self.goal = nil
    self.sp = { x = x, z = z, altitude = altitude, heading = heading }
    return self.sp
end

-- Begin a mission to a goal. Returns accepted, reason.
function Guidance:go(goal, cur)
    local c = self.cfg
    if not (goal and type(goal.x) == "number" and type(goal.z) == "number") then
        return false, "missing x/z"
    end
    local dist = math.sqrt((goal.x - cur.x) ^ 2 + (goal.z - cur.z) ^ 2)
    if dist > (c.maxGotoDistance or 200) then
        return false, string.format("too far: %.0f > %.0f", dist, c.maxGotoDistance or 200)
    end
    local alt = goal.altitude or cur.height
    if alt and c.altitudeFloor and alt < c.altitudeFloor then return false, "below altitude floor" end
    if alt and c.altitudeCeiling and alt > c.altitudeCeiling then return false, "above altitude ceiling" end

    self.goal = { x = goal.x, z = goal.z, altitude = alt, heading = goal.heading }
    self.launch = { x = cur.x, z = cur.z }
    self.sp = { x = cur.x, z = cur.z, altitude = c.cruiseAltitude, heading = cur.heading }
    self.state = "climb"
    return true, nil
end

-- cur = { x, z, height, heading, dt }. Returns setpoint table + state string.
function Guidance:update(cur)
    local c = self.cfg
    local g = self.goal
    local st = self.state
    local dt = cur.dt or 0.05

    -- If the nav fix is missing, don't run position math (would produce nil
    -- arithmetic / NaN). Hold the last setpoint until position returns.
    if st ~= "hold" and not (type(cur.x) == "number" and type(cur.z) == "number") then
        return self.sp, self.state
    end

    if st == "climb" then
        self.sp.x, self.sp.z = self.launch.x, self.launch.z
        self.sp.altitude = c.cruiseAltitude
        -- heading held at whatever it was when the mission started
        local near = cur.height and cur.height >= c.cruiseAltitude - (c.altTolerance or 1.5)
        -- "good enough": within a band and climb has plateaued -> gun it (the
        -- altitude loop keeps pushing toward cruiseAltitude during cruise).
        local goodEnough = cur.height
            and cur.height >= c.cruiseAltitude - (c.climbBand or 25)
            and (cur.vy == nil or math.abs(cur.vy) <= (c.climbVyLow or 0.4))
        if near or goodEnough then
            self.state = "orient"
        end

    elseif st == "orient" then
        self.sp.x, self.sp.z = self.launch.x, self.launch.z
        self.sp.altitude = c.cruiseAltitude
        local brg = self:bearingTo(cur.x, cur.z, g.x, g.z)
        self.sp.heading = brg
        local herr = math.abs(wrapDeg(brg - (cur.heading or brg)))
        local dist = math.sqrt((g.x - cur.x) ^ 2 + (g.z - cur.z) ^ 2)
        if herr <= (c.orientToleranceDeg or 8) or dist < (c.arriveRadius or 3) then
            self.sp.x, self.sp.z = cur.x, cur.z   -- seed the moving setpoint at the craft
            self.state = "cruise"
        end

    elseif st == "cruise" then
        self.sp.altitude = c.cruiseAltitude
        self.sp.heading = self:bearingTo(cur.x, cur.z, g.x, g.z)
        local distCur = math.sqrt((g.x - cur.x) ^ 2 + (g.z - cur.z) ^ 2)
        if distCur <= (c.brakeRadius or 30) then
            self.sp.x, self.sp.z = g.x, g.z
            -- Freeze heading now (final if requested, else current) so we stop
            -- chasing the bearing-to-goal, which spins wildly right over the target.
            self.finalHeading = g.heading or cur.heading or self.sp.heading
            self.state = "arrive"
        elseif c.directCruise then
            -- unbounded: hand the goal straight to the LQI. It saturates the
            -- command (floors it) then decelerates via its own velocity damping.
            self.sp.x, self.sp.z = g.x, g.z
        else
            -- advance the setpoint along the line to the goal at a decel-limited
            -- speed, but never let it lead the craft by more than maxLead (keeps
            -- the LQI position error bounded => smooth, no saturation).
            local lead = math.sqrt((self.sp.x - cur.x) ^ 2 + (self.sp.z - cur.z) ^ 2)
            if lead < (c.maxLead or 5) then
                local gx, gz = g.x - self.sp.x, g.z - self.sp.z
                local distSp = math.sqrt(gx * gx + gz * gz)
                if distSp > 1e-3 then
                    local vCmd = math.min(c.vCruise or 8, math.sqrt(2 * (c.aDecel or 1) * math.max(0, distCur)))
                    local step = math.min(vCmd * dt, distSp)
                    self.sp.x = self.sp.x + (gx / distSp) * step
                    self.sp.z = self.sp.z + (gz / distSp) * step
                end
            end
        end

    elseif st == "arrive" then
        -- Brake and settle over the target and re-orient to the final heading
        -- BEFORE descending. Hold the goal (LQI decelerates) and wait until it is
        -- both close and slow, so a fast long-range approach can bleed off speed.
        self.sp.x, self.sp.z = g.x, g.z
        self.sp.altitude = c.cruiseAltitude
        self.sp.heading = self.finalHeading or self.sp.heading
        local distCur = math.sqrt((g.x - cur.x) ^ 2 + (g.z - cur.z) ^ 2)
        if distCur <= (c.arriveRadius or 10) and (cur.speed or 0) <= (c.arriveSpeed or 2) then
            self.state = "descend"
        end

    elseif st == "descend" then
        self.sp.x, self.sp.z = g.x, g.z
        self.sp.heading = self.finalHeading or self.sp.heading
        self.sp.altitude = g.altitude or c.cruiseAltitude
        if cur.height and math.abs(cur.height - self.sp.altitude) <= (c.altTolerance or 1.5) then
            self.state = "hold"
        end
    end
    -- "hold": setpoint unchanged (park in place).

    return self.sp, self.state
end

return Guidance
