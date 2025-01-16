-- Loads one of three mission files to autopilot on each arm, depending on position of the Mission Reset AUX FUNC switch
-- Must have Mission Reset switch assigned, it will function normally when armed or disarmed
-- but also on the disarm to arm transition, it will load (if file exists) a file in the root named
-- missionH.txt, missionM.txt, or missionL.txt corresponding to the Mission Reset switch position of High/Mid/Low

local mission_loaded = false

local function read_mission()
    -- Clear any existing mission
    assert(mission:clear(), 'Could not clear current mission')

    local item = mavlink_mission_item_int_t()

    --------------------------------------
    -- Line 0 from QGC mission (Index=0)
    -- 0  1  0  16  0  0  0  0  27.0649942  80.2686842  139.61  1
    --------------------------------------
    item:seq(0)
    -- Frame=0 (MAV_FRAME_GLOBAL)
    item:frame(0)
    -- Command=16 (NAV_WAYPOINT)
    item:command(16)
    -- Param1..Param4 = 0
    item:param1(0)
    item:param2(0)
    item:param3(0)
    item:param4(0)
    -- Lat=27.0649942, Lon=80.2686842, Alt=139.61
    item:x(math.floor(27.0649942 * 1e7))
    item:y(math.floor(80.2686842 * 1e7))
    item:z(139.61)
    assert(mission:set_item(0,item), 'Failed to set mission item 0')

    --------------------------------------
    -- Line 1 from QGC mission (Index=1)
    -- 1  0  3  22  0  0  0  0  0.0        0.0        100.0    1
    -- (Takeoff command, relative alt frame)
    --------------------------------------
    print("Head to waypoint 2")
    item:seq(1)
    -- Frame=3 (MAV_FRAME_GLOBAL_RELATIVE_ALT)
    item:frame(0)
    -- Command=22 (NAV_TAKEOFF)
    item:command(22)
    -- Param1..Param4 = 0
    item:param1(0)
    item:param2(0)
    item:param3(0)
    item:param4(0)
    -- Lat=0, Lon=0, Alt=100  (takeoff to 100m)
    item:x(math.floor(27.0689942 * 1e7))
    item:y(math.floor(80.2706842 * 1e7))
    item:z(200)
    assert(mission:set_item(1,item), 'Failed to set mission item 1')

    --------------------------------------
    -- Line 2 from QGC mission (Index=2)
    -- 2  0  3  16  0  0  0  0  27.0676480  80.2668107  100.0   1
    --------------------------------------
    item:seq(2)
    item:frame(0)
    item:command(16) -- NAV_WAYPOINT
    item:param1(0)
    item:param2(0)
    item:param3(0)
    item:param4(0)
    -- Lat=27.067648, Lon=80.2668107, Alt=100
    item:x(math.floor(27.0690994  * 1e7))
    item:y(math.floor(80.2768107 * 1e7))
    item:z(250)
    assert(mission:set_item(2,item), 'Failed to set mission item 2')

    gcs:send_text(6, "3 Mission Items Loaded")
end

function update()
    if not arming:is_armed() then
        -- If disarmed, reset our flag so we can load again on next arm
        mission_loaded = false
        return update, 1000
    end

    if not mission_loaded then
        -- If first time after arm, load the mission
        mission_loaded = true
        read_mission()
    end

    return update, 1000
end

gcs:send_text(5,"Loaded MissionSelector.lua")

return update, 5000
