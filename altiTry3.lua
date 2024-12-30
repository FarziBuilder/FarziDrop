-- Frequency of updates in milliseconds
local FREQUENCY = 1000
local flag = 0
local past_altitude = 0
local altitude = 0
local check_altitude = 0

function update()
    -- Check if AHRS (Attitude and Heading Reference System) is healthy
    if not ahrs:healthy() then
        gcs:send_text(5, "AHRS ka intezaar hai2...")
        return update, FREQUENCY
    end

    -- Get the relative position in NED (North-East-Down) frame
    local dist = ahrs:get_relative_position_NED_home()
    
    if dist then
        -- Extract altitude (convert from 'down' to 'up' by multiplying by -1)
        past_altitude = altitude
        altitude = -1 * dist:z()
        gcs:send_text(5, string.format("Altitude relative to home: %.2f meters", altitude))

        -- Check if the altitude is decreasing
        if past_altitude > altitude then
            if flag == 0 then
                check_altitude = past_altitude
                flag = 1
            end

            if (check_altitude - altitude) < 20 then
                gcs:send_text(5, "I am falling bruh!")
                local mode_changed = vehicle:set_mode(5)  -- Example: 5 might be FBW
                if mode_changed then
                    gcs:send_text(5, "Triggered FBW")
                else
                    gcs:send_text(5, "Failed to set FBW mode")
                end
            else
                gcs:send_text(5, "Time to RTL!")
                local mode_changed = vehicle:set_mode(11) -- Example: 11 might be RTL
                if mode_changed then
                    gcs:send_text(5, "Triggered RTL")
                else
                    gcs:send_text(5, "Failed to set RTL mode")
                end
            end
        end
    else
        gcs:send_text(5, "Unable to retrieve relative position.")
    end

    return update, FREQUENCY -- Schedule this function again after the specified frequency
end

-- Run the update function immediately once the script is loaded
return update()
