    -- MissionSelector.lua
    -- Example script that loads specific waypoints from QGC WPL 110 list
    -- (Adapts the same logic from your original script)

    local mission_loaded = false

    -- This function sets up the mission waypoints as specified.
    local function read_mission()
        -- Clear any existing mission
        assert(mission:clear(), 'Could not clear current mission')

        local item = mavlink_mission_item_int_t()
        ------------------------------------------------------------------------
        item:seq(0)
        -- Frame=3 (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        item:frame(3)
        -- Command=22 (NAV_TAKEOFF)
        item:command(22)
        -- Param1..Param4 = 0
        item:param1(0)
        item:param2(0)
        item:param3(0)
        item:param4(0)
        -- Lat=0, Lon=0, Alt=20
        item:x(math.floor(-35.3632620 * 1e7))
        item:y(math.floor(149.1652370 * 1e7))
        item:z(20)
        assert(mission:set_item(0, item), 'Failed to set mission item 1')

        ------------------------------------------------------------------------
        -- Waypoint #2
        -- From QGC line:
        -- 2 0 3 16 0 0 0 0 -35.35790620 149.16446690 50.000000 1
        ------------------------------------------------------------------------
        item:seq(1)
        item:frame(3)
        item:command(16) -- NAV_WAYPOINT
        item:param1(0)
        item:param2(0)
        item:param3(0)
        item:param4(0)
        -- Lat=-35.35790620, Lon=149.16446690, Alt=50
        item:x(math.floor(-35.35790620 * 1e7))
        item:y(math.floor(149.16446690 * 1e7))
        item:z(50)
        assert(mission:set_item(1, item), 'Failed to set mission item 2')

        ------------------------------------------------------------------------
        -- Waypoint #3
        -- From QGC line:
        -- 3 0 3 189 0 0 0 0 0.0 0.0 0.0 1
        -- (Command #189, typically DO_CHANGE_SPEED or user-defined)
        ------------------------------------------------------------------------
        local item2 = mavlink_mission_item_int_t()

        item2 = mission:get_item(1)
        item2:seq(2)
        item2:frame(3)
        item2:command(16) -- NAV_WAYPOINT
        item2:param1(0)
        item2:param2(0)
        item2:param3(0)
        item2:param4(0)
        -- Lat=-35.35790620, Lon=149.16446690, Alt=50
        item2:x(math.floor(-35.35790720 * 1e7))
        item2:y(math.floor(149.16446790 * 1e7))
        item2:z(90)

        assert(mission:set_item(2,item2), 'Failed to set mission item 2')
        print("Item 2 set " .. tostring(item2:z()))
        
        -- item:seq(2)
        -- item:frame(3)
        -- item:command(189)
        -- item:param1(0)
        -- item:param2(0)
        -- item:param3(0)
        -- item:param4(0)
        -- -- Often these DO_ commands ignore lat/lon/alt, but QGC still includes them
        -- item:x(math.floor(0.0 * 1e7))
        -- item:y(math.floor(0.0 * 1e7))
        -- item:z(0)
        -- assert(mission:set_item(2, item), 'Failed to set mission item 3')

        ------------------------------------------------------------------------
        -- Waypoint #4
        -- From QGC line:
        -- 4 0 3 16 0 0 0 0 -35.36509830 149.16391970 40.000000 1
        ------------------------------------------------------------------------
        item:seq(3)
        item:frame(3)
        item:command(16) -- NAV_WAYPOINT
        item:param1(0)
        item:param2(0)
        item:param3(0)
        item:param4(0)
        -- Lat=-35.36509830, Lon=149.16391970, Alt=40
        item:x(math.floor(-35.36509830 * 1e7))
        item:y(math.floor(149.16391970 * 1e7))
        item:z(40)
        assert(mission:set_item(3, item), 'Failed to set mission item 4')

        ------------------------------------------------------------------------
        -- Waypoint #5
        -- From QGC line:
        -- 5 0 3 16 0 0 0 0 -35.36509830 149.16558260 30.000000 1
        ------------------------------------------------------------------------
        item:seq(4)
        item:frame(3)
        item:command(16) -- NAV_WAYPOINT
        item:param1(0)
        item:param2(0)
        item:param3(0)
        item:param4(0)
        -- Lat=-35.36509830, Lon=149.16558260, Alt=30
        item:x(math.floor(-35.36509830 * 1e7))
        item:y(math.floor(149.16558260 * 1e7))
        item:z(30)
        assert(mission:set_item(4, item), 'Failed to set mission item 5')

        ------------------------------------------------------------------------
        -- Waypoint #6
        -- From QGC line:
        -- 6 0 3 21 0 0 0 1.0 -35.36325640 149.16526480 0.000000 1
        -- Command=21 (NAV_LAND), param4=1.0
        ------------------------------------------------------------------------
        item:seq(5)
        item:frame(3)
        item:command(21) -- NAV_LAND
        item:param1(0)
        item:param2(0)
        item:param3(0)
        item:param4(1.0)
        -- Lat=-35.36325640, Lon=149.16526480, Alt=0
        item:x(math.floor(-35.36325640 * 1e7))
        item:y(math.floor(149.16526480 * 1e7))
        item:z(0)
        assert(mission:set_item(5, item), 'Failed to set mission item 6')

        -- Send a quick message to confirm
        gcs:send_text(6, "7 Mission Items Loaded")
    end

    --------------------------------------------------------------------------------
    -- The main update function:
    --------------------------------------------------------------------------------
    function update()
        -- If disarmed, reset our loaded flag
        if not arming:is_armed() then
            mission_loaded = false
            return update, 1000
        end

        -- If we just transitioned to armed, load the mission once
        if not mission_loaded then
            mission_loaded = true
            read_mission()
        end

        -- Continue calling this script periodically
        return update, 1000
    end

    -- Print a message at load
    gcs:send_text(5, "Loaded MissionSelector.lua")

    -- Return the update function and first delay
    return update, 5000
