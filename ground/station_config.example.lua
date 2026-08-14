-- MC Aero ground station config TEMPLATE.
--
-- This is a Lua file that returns a table (NOT JSON).
-- Copy it to /station_config.lua on each ground station and edit per role:
--   cp /ground/station_config.example.lua /station_config.lua
--
-- One page is drawn per detected monitor, in order. Assign your four
-- stations different pages, e.g. { "flight" }, { "systems" }, { "signals" },
-- { "raw" }. A station with two monitors can list two pages.

return {
    -- flight | systems | signals | raw
    pages = { "flight" },

    -- Set relay = true on EXACTLY ONE station (otherwise S3 gets duplicates).
    relay = false,

    -- Endpoint/secret for the relaying station. Preferred: leave these out and
    -- provide /relay_config.lua instead (so the secret lives in one place).
    -- endpoint = "https://REPLACE_ME.lambda-url.us-east-2.on.aws/",
    -- apiKey = "REPLACE_WITH_YOUR_SECRET",

    -- protocol = "mc_aero.telemetry.v1",
    -- textScale = 0.5,
}
