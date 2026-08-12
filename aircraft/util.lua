local util = {}

local function pack(...)
    return { n = select("#", ...), ... }
end

function util.nowMs()
    return os.epoch("utc")
end

function util.clamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

function util.round(value, places)
    local scale = 10 ^ (places or 0)
    return math.floor(value * scale + 0.5) / scale
end

function util.call(name, method, ...)
    if not name then
        return nil, "peripheral name is not configured"
    end
    if not peripheral.isPresent(name) then
        return nil, name .. " is not present"
    end

    local result = pack(pcall(peripheral.call, name, method, ...))
    if not result[1] then
        return nil, tostring(result[2])
    end

    local count = result.n - 1
    if count == 0 then
        return true, nil
    elseif count == 1 then
        return result[2], nil
    end

    local values = {}
    for index = 2, result.n do
        values[index - 1] = result[index]
    end
    return values, nil
end

function util.checkPeripheral(name, expectedType)
    if not name then
        return false, "not configured"
    end
    if not peripheral.isPresent(name) then
        return false, "not present"
    end

    local actualType = peripheral.getType(name)
    if expectedType and actualType ~= expectedType then
        return false, "expected " .. expectedType .. ", got " .. tostring(actualType)
    end
    return true, nil
end

function util.readMethods(name, methods, errorPrefix)
    local values, errors = {}, {}
    for _, method in ipairs(methods) do
        local value, callError = util.call(name, method)
        if callError then
            errors[(errorPrefix or name) .. "." .. method] = callError
        else
            values[method] = value
        end
    end
    return values, errors
end

function util.mergeErrors(destination, source)
    for key, value in pairs(source or {}) do
        destination[key] = value
    end
end

function util.count(tableValue)
    local total = 0
    for _ in pairs(tableValue or {}) do
        total = total + 1
    end
    return total
end

local function render(value, depth, visited)
    local valueType = type(value)
    if valueType == "nil" then
        return "-"
    elseif valueType == "number" then
        return string.format("%.3f", value)
    elseif valueType == "boolean" then
        return value and "true" or "false"
    elseif valueType == "string" then
        return value:gsub("[\r\n]", " ")
    elseif valueType ~= "table" then
        return tostring(value)
    end

    if depth >= 2 or visited[value] then
        return "{...}"
    end
    visited[value] = true

    local parts, keys = {}, {}
    for key in pairs(value) do
        keys[#keys + 1] = key
    end
    table.sort(keys, function(left, right)
        return tostring(left) < tostring(right)
    end)
    for index, key in ipairs(keys) do
        if index > 6 then
            parts[#parts + 1] = "..."
            break
        end
        local item = render(value[key], depth + 1, visited)
        if type(key) == "number" then
            parts[#parts + 1] = item
        else
            parts[#parts + 1] = tostring(key) .. "=" .. item
        end
    end
    visited[value] = nil
    return "{" .. table.concat(parts, ",") .. "}"
end

function util.oneLine(value)
    return render(value, 0, {})
end

return util
