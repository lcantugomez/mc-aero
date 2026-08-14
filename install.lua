local REPOSITORY = "https://raw.githubusercontent.com/lcantugomez/mc-aero/main/"

-- File manifests per install target. "all" installs everything.
local TARGETS = {
    aircraft = {
        "aircraft/config.lua",
        "aircraft/util.lua",
        "aircraft/sensors.lua",
        "aircraft/actuators.lua",
        "aircraft/telemetry.lua",
        "aircraft/display.lua",
        "aircraft/logger.lua",
        "aircraft/startup.lua",
    },
    ground = {
        "ground/station.lua",
        "ground/relay.lua",
    },
}

local RUN_HINT = {
    aircraft = "aircraft/startup",
    ground = "ground/station",
}

local arguments = { ... }
local target = arguments[1] or "aircraft"
if target == "help" or target == "--help" then
    print("Usage: install [aircraft|ground|all]")
    print("  aircraft  flight computer package (default)")
    print("  ground    base telemetry station + relay")
    print("  all       both packages")
    return
end

-- Resolve the target into an ordered, de-duplicated file list.
local files = {}
local seen = {}
local function addFiles(list)
    for _, path in ipairs(list) do
        if not seen[path] then
            seen[path] = true
            files[#files + 1] = path
        end
    end
end

if target == "all" then
    addFiles(TARGETS.aircraft)
    addFiles(TARGETS.ground)
elseif TARGETS[target] then
    addFiles(TARGETS[target])
else
    error("Unknown install target: " .. tostring(target) .. " (try: aircraft, ground, all)", 0)
end

if not http then
    error("CC:Tweaked HTTP is disabled", 0)
end

local staging = "/.mc-aero-install-" .. tostring(os.getComputerID())
if fs.exists(staging) then fs.delete(staging) end
fs.makeDir(staging)

local function download(relativePath)
    local url = REPOSITORY .. relativePath
    print("GET " .. relativePath)
    local response, requestError = http.get(url, nil, true)
    if not response then return false, tostring(requestError or "request failed") end

    local code = nil
    if response.getResponseCode then
        local ok, result = pcall(response.getResponseCode)
        if ok then code = result end
    end
    local content = response.readAll()
    response.close()
    if code and code >= 400 then return false, "HTTP " .. tostring(code) end
    if not content or #content == 0 then return false, "empty response" end

    local stagedPath = fs.combine(staging, relativePath)
    local directory = fs.getDir(stagedPath)
    if not fs.exists(directory) then fs.makeDir(directory) end
    local file = fs.open(stagedPath, "wb")
    if not file then return false, "cannot write " .. stagedPath end
    file.write(content)
    file.close()
    return true, nil
end

for _, relativePath in ipairs(files) do
    local ok, downloadError = download(relativePath)
    if not ok then
        fs.delete(staging)
        error("Install failed for " .. relativePath .. ": " .. downloadError, 0)
    end
end

for _, relativePath in ipairs(files) do
    local source = fs.combine(staging, relativePath)
    local destination = "/" .. relativePath
    local directory = fs.getDir(destination)
    if not fs.exists(directory) then fs.makeDir(directory) end
    if fs.exists(destination) then fs.delete(destination) end
    fs.move(source, destination)
end
fs.delete(staging)

print("MC Aero '" .. target .. "' package installed.")
if target == "all" then
    print("Run aircraft: " .. RUN_HINT.aircraft)
    print("Run ground:   " .. RUN_HINT.ground)
else
    print("Run: " .. RUN_HINT[target])
end
if target == "aircraft" or target == "all" then
    print("Optional overrides: /aircraft/user_config.lua")
end
if target == "ground" or target == "all" then
    print("Ground config: /station_config.lua (+ /relay_config.lua on the relaying station)")
end
