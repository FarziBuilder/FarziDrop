-- Frequency of updates in milliseconds
local FREQUENCY = 1000

local start_time = 0
local triggered = false

function update()
    -- Capture start time on first run
    if start_time == 0 then
        start_time = millis()
    end

    -- Check if AHRS (Attitude and Heading Reference System) is healthy
    if not ahrs:healthy() then
        gcs:send_text(5, "Waiting for AHRS initialization...")
        return update, FREQUENCY
    end

    -- Get the relative position in NED (North-East-Down) frame
    local dist = ahrs:get_relative_position_NED_home()
    if dist then
        local altitude = -dist:z()  -- Convert 'down' to 'up'
        gcs:send_text(5, string.format("Altitude relative to home: %.2f m", altitude))
    else
        gcs:send_text(5, "Unable to retrieve relative position.")
    end

    -- After 10 seconds, trigger RTL if not already triggered
    local now = millis()
    if not triggered and (now - start_time) >= 10000 then
        triggered = true
        -- Mode 6 is typically RTL for Copter
        local mode_changed = vehicle:set_mode(11)
        if mode_changed then
            gcs:send_text(5, "Triggered RTL")
        else
            gcs:send_text(5, "Failed to set RTL mode")
        end
    end

    return update, FREQUENCY
end

return update()
