--[[ 
  Simple ArduPilot Lua script for checking altitude decrease. 
  If altitude is decreasing for 1 second, switch to FBW mode. 
  5 seconds later, switch to RTL mode.
--]]

local prev_alt = nil         -- store previous altitude
local decrease_start = 0     -- time at which altitude started decreasing
local fbw_initiated = false  -- flag to indicate FBW mode has been triggered
local fbw_time = 0           -- time at which FBW was triggered
local rtl_done = false       -- flag to indicate RTL has been triggered
local FREQUENCY = 500

function update()
    -- get current altitude in meters
    if not ahrs:healthy() then
        gcs:send_text(5, "AHRS ka intezaar hai4...")
        return update, FREQUENCY
    end
    
    local dist = ahrs:get_relative_position_NED_home()
    local current_alt = -1 * dist:z()
    local now = millis()

    -- if prev_alt is nil (first run), set it to current altitude
    if not prev_alt then 
        prev_alt = current_alt
        return update, FREQUENCY
    end
    gcs:send_text(5, string.format("Altitude relative to home: %.2f meters", current_alt))
    -- check if altitude is decreasing
    if current_alt < prev_alt then
        -- if not already counting, start counting now
        if decrease_start == 0 then 
            decrease_start = now
        else
            -- if it's been decreasing for >= 1000ms (1 second) and FBW not yet triggered
            if not fbw_initiated and (now - decrease_start >= 1000) then
                gcs:send_text(6, "Altitude decreasing for 1s -> Switching to FBW")
                -- FBWA is mode #5 for ArduPilot Plane
                vehicle:set_mode(5)  
                fbw_initiated = true
                fbw_time = now
            end
        end
    else
        -- altitude not decreasing, reset timer
        decrease_start = 0
    end

    -- if FBW has been initiated, and 5 seconds have passed, switch to RTL (mode #6)
    if fbw_initiated and not rtl_done and (now - fbw_time >= 4000) then
        gcs:send_text(6, "Switching to RTL")
        vehicle:set_mode(11)  -- RTL is mode #6 for Plane
        rtl_done = true
    end

    -- store current altitude for next iteration
    prev_alt = current_alt
end

return update()
