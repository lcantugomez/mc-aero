-- MC Aero relay/endpoint config TEMPLATE.
--
-- This is a Lua file that returns a table (NOT JSON).
-- Copy it to /relay_config.lua on the relaying computer and fill in your
-- values:   cp /ground/relay_config.example.lua /relay_config.lua
--
-- Keep the real /relay_config.lua out of version control: it holds a secret.

return {
    -- Lambda Function URL (or any HTTPS ingest endpoint), include trailing slash.
    endpoint = "https://REPLACE_ME.lambda-url.us-east-2.on.aws/",

    -- Shared secret sent as the x-api-key header; must match the endpoint's.
    apiKey = "REPLACE_WITH_YOUR_SECRET",

    -- Rednet protocol to listen on (leave as-is unless you changed it).
    protocol = "mc_aero.telemetry.v1",
}
