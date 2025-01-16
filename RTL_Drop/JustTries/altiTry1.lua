-- Frequency of updates in milliseconds
local FREQUENCY = 1000 

function update()
    -- Check if AHRS (Attitude and Heading Reference System) is healthy
    if not ahrs:healthy() then
        gcs:send_text(5, "Waiting for AHRS initialization...")
        return update, FREQUENCY
    end

    -- Get the relative position in NED (North-East-Down) frame
    local dist = ahrs:get_relative_position_NED_home()
    
    if dist then
        -- Extract altitude (convert from 'down' to 'up' by multiplying by -1)
        local altitude = -1 * dist:z()
        gcs:send_text(5, string.format("Altitude relative to home: %.2f meters", altitude))
    else
        gcs:send_text(5, "Unable to retrieve relative position.")
    end

    return update, FREQUENCY -- Schedule this function again after the specified frequency
end

return update()
