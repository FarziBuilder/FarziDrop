local waypoints = {
    { lat = 26.8760, lon = 80.0951, alt = 50, cmd = 10 }, -- Waypoint 1 (16 = normal waypoint)
    { lat = 26.9760, lon = 80.9951, alt = 60, cmd = 10 }, -- Waypoint 2
    { lat = 26.8060, lon = 80.9951, alt = 70, cmd = 10 }, -- Waypoint 3
}
-- Frequency of updates in milliseconds (ms)
local FREQUENCY = 1000
local altitude = 0
local past_altitude = 0
local flag = 0

function update_mission()
    print("Lets see bro")
    if mission:get_current_nav_index() == 1 then  -- Index 1 = second waypoint
        local m = mission:get_item(2)  -- Get third waypoint (index 2)
        m:command(10)  -- Normal WAYPOINT
        m:x(26.7760)
        m:y(80.9951)
        m:z(100)
        mission:set_item(2, m)  -- Update mission item
        gcs:send_text(6, "Mission updated at waypoint 2!")
    end
end

function update()
    -- 1) Check if AHRS (Attitude and Heading Reference System) is healthy
    if not ahrs:healthy() then
        gcs:send_text(5, "Waiting for AHRS initialization...")
        return update, FREQUENCY
    end

    if mission:get_current_nav_index() == 1 then  -- Index 1 = second waypoint
        local m = mission:get_item(2)  -- Get third waypoint (index 2)
        -- print(m)
        m:command(10)  -- Normal WAYPOINT
        m:x(2677600)
        m:y(8099510)
        m:z(100)
        mission:set_item(2, m)  -- Update mission item
        mission:write()
        gcs:send_text(6, "Mission updated at waypoint 2!")
    end

    -- 6) Schedule the function again
    return update, FREQUENCY
end

return update()


