-- peripheral_dump.lua : list every attached peripheral, its type, methods, and
-- the output of every getter (get*/is*/has* only -- never calls a setter).
--
-- Usage:
--   peripheral_dump              dump everything
--   peripheral_dump <filter>     only peripherals whose name or type contains <filter>
--   peripheral_dump /out.txt     dump everything and also write it to /out.txt
--
-- Run on any computer to discover peripheral names/methods (e.g. the new RSCs).

local arg1 = ...
local outPath, filter
if type(arg1) == "string" and arg1:sub(1, 1) == "/" then
    outPath = arg1
elseif type(arg1) == "string" and #arg1 > 0 then
    filter = arg1
end

local lines = {}
local function emit(s)
    print(s)
    lines[#lines + 1] = s
end

local function show(v)
    if type(v) == "table" then
        local ok, s = pcall(textutils.serialize, v, { compact = true })
        return ok and s or "{table}"
    end
    return tostring(v)
end

local names = peripheral.getNames()

-- summary by type
local counts = {}
for _, n in ipairs(names) do
    local t = peripheral.getType(n)
    counts[t] = (counts[t] or 0) + 1
end
emit("=== peripheral summary (" .. #names .. " total) ===")
local types = {}
for t in pairs(counts) do types[#types + 1] = t end
table.sort(types)
for _, t in ipairs(types) do emit(string.format("  %-34s x%d", t, counts[t])) end

emit("=== detail ===")
table.sort(names)
for _, name in ipairs(names) do
    local t = peripheral.getType(name)
    if not filter or name:find(filter, 1, true) or t:find(filter, 1, true) then
        emit("")
        emit(name .. "  (" .. t .. ")")
        local methods = peripheral.getMethods(name)
        table.sort(methods)
        emit("  methods: " .. table.concat(methods, ", "))
        for _, m in ipairs(methods) do
            if m:match("^get") or m:match("^is") or m:match("^has") then
                local r = { pcall(peripheral.call, name, m) }
                local ok = table.remove(r, 1)
                if ok then
                    local parts = {}
                    for i = 1, #r do parts[i] = show(r[i]) end
                    emit(string.format("    %-26s -> %s", m,
                        #parts > 0 and table.concat(parts, ", ") or "(nil)"))
                else
                    emit(string.format("    %-26s !! %s", m, tostring(r[1])))
                end
            end
        end
    end
end

if outPath then
    local file = fs.open(outPath, "w")
    if file then
        file.write(table.concat(lines, "\n"))
        file.close()
        print("written to " .. outPath)
    else
        print("could not write " .. outPath)
    end
end
