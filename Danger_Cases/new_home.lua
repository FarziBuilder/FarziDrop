--------------------------------------------------------------------------------
-- Frequency of updates in milliseconds
--------------------------------------------------------------------------------
local FREQUENCY = 1000

--------------------------------------------------------------------------------
-- Desired new home coordinates
--------------------------------------------------------------------------------
local NEW_HOME_LAT = 17.5448290
local NEW_HOME_LON = 78.0420822
local NEW_HOME_ALT = 0

--------------------------------------------------------------------------------
-- Target mode to set (here 11 is often RTL or a custom mode; verify in your firmware)
--------------------------------------------------------------------------------
local TARGET_MODE = 11

--------------------------------------------------------------------------------
-- Main update function
--------------------------------------------------------------------------------
function update()
    -- 1) Make sure AHRS is ready
    if not ahrs:healthy() then
        gcs:send_text(5, "Waiting for AHRS initialization...")
        return update, FREQUENCY
    end

    -- 2) Construct a Location object for the new home
    local loc = Location()
    loc:lat(math.floor(NEW_HOME_LAT * 1e7 + 0.5))
    loc:lng(math.floor(NEW_HOME_LON * 1e7 + 0.5))
    loc:alt(NEW_HOME_ALT)

    -- 3) Attempt to set this as the new home
    local result = ahrs:set_home(loc)
    if result then
        gcs:send_text(6, string.format("Home set: (%.6f, %.6f), alt=%.2f",
                                       NEW_HOME_LAT, NEW_HOME_LON, NEW_HOME_ALT))
    else
        gcs:send_text(3, "Failed to set custom home!")
    end

    -- 4) Set vehicle mode to 11
    vehicle:set_mode(TARGET_MODE)
    gcs:send_text(6, "Vehicle mode set to 11")

    -- 5) Return for scheduling again
    return update, FREQUENCY
end

--------------------------------------------------------------------------------
-- Schedule the script
--------------------------------------------------------------------------------
return update()
