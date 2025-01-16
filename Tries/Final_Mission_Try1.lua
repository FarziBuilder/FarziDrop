--------------------------------------------------------------------------------
-- Frequency of updates in milliseconds
--------------------------------------------------------------------------------
local FREQUENCY    = 1000
local altitude     = 0
local past_altitude = 0
local flag         = 0
local flag_rtl     = 0
local next_wpt_alt = 0
local flag_land     = 0
------------------------
-- Define states
local STATE_IDLE = 0
local STATE_DROP_DETECTED = 1
local STATE_RTL = 2
local STATE_LANDING = 3

-- Initialize state
local current_state = STATE_IDLE
--------------------------


--------------------------------------------------------------------------------
-- If math.atan2 is not available, define our own:
--------------------------------------------------------------------------------
local function my_atan2(y, x)
    if x > 0 then
        return math.atan(y/x)
    elseif x < 0 then
        if y >= 0 then
            return math.atan(y/x) + math.pi
        else
            return math.atan(y/x) - math.pi
        end
    else
        -- x == 0
        if y > 0 then
            return math.pi / 2
        elseif y < 0 then
            return -math.pi / 2
        else
            return 0  -- undefined, but 0 as fallback
        end
    end
end

--------------------------------------------------------------------------------
-- Earth radius (for the calc_intermediate_waypoint logic)
--------------------------------------------------------------------------------
local R = 6371000  -- meters

--------------------------------------------------------------------------------
-- Helper: convert degrees <-> radians
--------------------------------------------------------------------------------
local function deg2rad(d) return d * math.pi / 180 end
local function rad2deg(r) return r * 180 / math.pi end

--------------------------------------------------------------------------------
-- Compute bearing in radians from (lat1, lon1) to (lat2, lon2).
--------------------------------------------------------------------------------
local function get_bearing_radians(lat1_deg, lon1_deg, lat2_deg, lon2_deg)
    local lat1 = deg2rad(lat1_deg)
    local lat2 = deg2rad(lat2_deg)
    local dLon = deg2rad(lon2_deg - lon1_deg)

    local y = math.sin(dLon) * math.cos(lat2)
    local x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
    return my_atan2(y, x)
end

--------------------------------------------------------------------------------
-- Great-circle "destination" formula:
--  Given a start lat/lon (deg), distance (m), and bearing (rad),
--  compute the new lat/lon (deg).
--------------------------------------------------------------------------------
local function calc_destination(lat1_deg, lon1_deg, distance_m, bearing_rad)
    local lat1 = deg2rad(lat1_deg)
    local lon1 = deg2rad(lon1_deg)
    local d_R  = distance_m / R  -- angular distance on Earth’s surface

    local lat2 = math.asin(
        math.sin(lat1) * math.cos(d_R) +
        math.cos(lat1) * math.sin(d_R) * math.cos(bearing_rad)
    )
    local lon2 = lon1 + my_atan2(
        math.sin(bearing_rad) * math.sin(d_R) * math.cos(lat1),
        math.cos(d_R) - math.sin(lat1) * math.sin(lat2)
    )

    return rad2deg(lat2), rad2deg(lon2)
end

--------------------------------------------------------------------------------
-- Compute an intermediate waypoint:
--   1) Bearing from Drop -> Home
--   2) Move distance_to_move from Drop along that bearing
--   3) Altitude = new_altitude
--------------------------------------------------------------------------------
local function calc_intermediate_waypoint(home_lat, home_lon, drop_lat, drop_lon, distance_to_move, new_altitude)
    local bearing_drop_to_home =
        get_bearing_radians(drop_lat, drop_lon, home_lat, home_lon)

    local wpt_lat, wpt_lon =
        calc_destination(drop_lat, drop_lon, distance_to_move, bearing_drop_to_home)

    return wpt_lat, wpt_lon, new_altitude
end

--------------------------------------------------------------------------------
-- Example home coordinates (replace with actual or fetch from ahrs:get_home())
--------------------------------------------------------------------------------
local HOME_LAT = -35.3632484
local HOME_LON = 149.1652702
local HOME_ALT = 0

local LAND_LAT = -35.31106300
local LAND_LON = 149.17268510
local LAND_ALT = 150.000000
--------------------------------------------------------------------------------
-- Array of possible "home" coordinates
--------------------------------------------------------------------------------
local possible_homes = {
    {
        home = {lat = -35.3450953, lon = 149.1736507},
        landing = {lat = -35.3452000, lon = 149.1737000}
    },
    {
        home = {lat = -35.3734621, lon = 149.1792083},
        landing = {lat = -35.3735000, lon = 149.1792500}
    },
    {
        home = {lat = -35.3151252, lon = 149.1931343},
        landing = {lat = -35.3152000, lon = 149.1932000}
    },
    -- Add more entries as needed
}

--------------------------------------------------------------------------------
-- We'll store the drop lat/lon once we detect the altitude drop.
--------------------------------------------------------------------------------
local drop_lat = 0
local drop_lon = 0
local seq = 0

local wpt_lat_temp, wpt_lon_temp, wpt_alt_temp

--------------------------------------------------------------------------------
-- Create and upload a mission item
--------------------------------------------------------------------------------
local function create_and_upload_mission(seq, wpt_lat, wpt_lon, wpt_alt, cmd)
    local item_waypoint = mavlink_mission_item_int_t()
    item_waypoint:seq(seq)
    item_waypoint:frame(3)          -- MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    item_waypoint:command(cmd)       -- MAV_CMD_NAV_WAYPOINT
    item_waypoint:param1(0)
    item_waypoint:param2(0)
    item_waypoint:param3(0)
    item_waypoint:param4(0)
    item_waypoint:x(math.floor(wpt_lat * 1e7 + 0.5))
    item_waypoint:y(math.floor(wpt_lon * 1e7 + 0.5))
    item_waypoint:z(wpt_alt)        -- altitude in meters
    assert(mission:set_item(seq, item_waypoint), 'Failed to set mission item # '..tostring(seq))
end

--------------------------------------------------------------------------------
-- Modify (or create) an existing mission item at 'seq'
--------------------------------------------------------------------------------
local function get_and_upload_mission(seq, wpt_lat, wpt_lon, wpt_alt)
    local item2 = mavlink_mission_item_int_t()
    item2 = mission:get_item(seq)
    item2:seq(seq)
    item2:frame(3)    -- MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    item2:command(16) -- NAV_WAYPOINT
    item2:param1(0)
    item2:param2(0)
    item2:param3(0)
    item2:param4(0)
    item2:x(math.floor(wpt_lat * 1e7 + 0.5))
    item2:y(math.floor(wpt_lon * 1e7 + 0.5))
    item2:z(wpt_alt)

    assert(mission:set_item(seq,item2), 'Failed to set mission item # '..tostring(seq))
    -- print("Item " .. tostring(seq) .. " set alt=" .. tostring(item2:z()))
end

--------------------------------------------------------------------------------
-- Compute horizontal distance (meters) between two GPS coordinates
-- using the Haversine formula.
--------------------------------------------------------------------------------
local function compute_distance(lat1_deg, lon1_deg, lat2_deg, lon2_deg)
    local dlat = deg2rad(lat2_deg - lat1_deg)
    local dlon = deg2rad(lon2_deg - lon1_deg)
    local lat1 = deg2rad(lat1_deg)
    local lat2 = deg2rad(lat2_deg)

    local a = math.sin(dlat/2) * math.sin(dlat/2) +
              math.cos(lat1) * math.cos(lat2) *
              math.sin(dlon/2) * math.sin(dlon/2)
    local c = 2 * my_atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c
end
--------------------------------------------------------------------------------
-- Find the closest home and its corresponding landing position from a reference point
--------------------------------------------------------------------------------
local function find_closest_coordinate(ref_lat, ref_lon, coords)
    local min_dist = math.huge
    local closest = nil

    for _, c in ipairs(coords) do
        local dist = compute_distance(ref_lat, ref_lon, c.home.lat, c.home.lon)
        if dist < min_dist then
            min_dist = dist
            closest = c
        end
    end

    if closest then
        return closest.home, closest.landing
    else
        return nil, nil
    end
end


--------------------------------------------------------------------------------
-- Function: reset_mission
-- Description: Resets the mission by setting the vehicle mode, clearing existing
--              missions, and uploading new waypoints based on the current location
--              and altitude.
-- Parameters:
--   curr_lat  - Current latitude in degrees
--   curr_lon  - Current longitude in degrees
--   altitude  - Current altitude in meters
-- Returns:
--   None
--------------------------------------------------------------------------------
local function reset_mission(curr_lat, curr_lon, altitude)
    -- Set vehicle mode to custom RTL (Return to Launch) or any predefined mode (e.g., mode 6)
    vehicle:set_mode(6)   -- Replace '6' with the appropriate mode number if different

    -- Clear existing mission items
    assert(mission:clear(), 'Could not clear current mission')

    -- Initialize sequence number
    -- seq = 0

    -- Upload Home Waypoint
    create_and_upload_mission(0, HOME_LAT, HOME_LON, HOME_ALT, 16)
    --  seq = seq + 1

    -- Compute and upload first intermediate waypoint 500 meters from home towards current location
    local wpt_lat1, wpt_lon1, wpt_alt1 =
        calc_intermediate_waypoint(HOME_LAT, HOME_LON, curr_lat, curr_lon, 50, altitude - 5)
    create_and_upload_mission(1, wpt_lat1, wpt_lon1, wpt_alt1, 16)
    
    --create_and_upload_mission(1, -35.3061950, 149.1933918, wpt_alt1)
    -- Compute and upload second intermediate waypoint 500 meters from the first waypoint
    local wpt_lat2, wpt_lon2, wpt_alt2 =
        calc_intermediate_waypoint(HOME_LAT, HOME_LON, wpt_lat1, wpt_lon1, 50, altitude - 10)
    
    wpt_lat_temp, wpt_lon_temp, wpt_alt_temp = wpt_lat2, wpt_lon2, wpt_alt2

    create_and_upload_mission(2, wpt_lat2, wpt_lon2, wpt_alt2, 16)
    return wpt_alt1
end


--------------------------------------------------------------------------------
-- Main update function
--------------------------------------------------------------------------------
function update()
    -- 1) Check if AHRS is healthy
    if not ahrs:healthy() then
        gcs:send_text(5, "Waiting for AHRS initialization...")
        return update, FREQUENCY
    end

    -- 2) Get current location and NED-home distance
    local location = ahrs:get_location()
    local dist     = ahrs:get_relative_position_NED_home()

    if (not location) or (not dist) then
        gcs:send_text(5, "Location or NED data not available.")
        return update, FREQUENCY
    end

    -- Convert 'down' to 'up' for altitude
    altitude = -1 * dist:z()

    -- Current lat/lon in degrees
    local curr_lat = location:lat() * 1.0e-7
    local curr_lon = location:lng() * 1.0e-7

    ---------------------------------------------------------
    -- PART A: Detect altitude drop => "drop" event
    ---------------------------------------------------------
    if current_state == STATE_IDLE then
        -- If altitude + 20 < past_altitude => significant drop
        if altitude + 10 < past_altitude then
            if flag == 0 then
                current_state = STATE_DROP_DETECTED
                vehicle:set_mode(6)
                gcs:send_text(6, "Drop detected. Switching to RTL...")

                -- A) Save the drop position
                drop_lat = curr_lat
                drop_lon = curr_lon

                home = ahrs:get_home()
                HOME_ALT = home:alt()
                -- B) Find the closest "home" from possible_homes
                local chosen_home, landing_pos = find_closest_coordinate(drop_lat, drop_lon, possible_homes)

                if chosen_home and landing_pos then
                    HOME_LAT = chosen_home.lat
                    HOME_LON = chosen_home.lon
                    LAND_LAT = landing_pos.lat
                    LAND_LON = landing_pos.lon
                    -- Proceed with setting home and landing positions
                else
                    gcs:send_text(4, "Error: No valid home and landing positions found! Defaulting to standard home pos")
                end

                
                gcs:send_text(6, "Early home alt is this " .. tostring(HOME_ALT))
                -- Attempt to set this new location as "home"
                local loc = Location()
                loc:alt(HOME_ALT)
                loc:lat(math.floor(HOME_LAT * 1e7 + 0.5))
                loc:lng(math.floor(HOME_LON * 1e7 + 0.5))
                -- Optionally set altitude if you want loc:alt(...) 
                local result = ahrs:set_home(loc)
                print("Home alt is " .. tostring(HOME_ALT))
                -- Let the GCS know if it worked
                if result then
                    gcs:send_text(6, string.format("Home set to Lat=%.6f, Lng=%.6f", HOME_LAT, HOME_LON))
                else
                    gcs:send_text(6, "Failed to set custom home!")
                end

                -- Debug message
                gcs:send_text(6, string.format("Closest home is lat=%.6f, lon=%.6f", HOME_LAT, HOME_LON))

                -- C) Compute the new waypoint
                local wpt_lat, wpt_lon, wpt_alt =
                    calc_intermediate_waypoint(HOME_LAT, HOME_LON, drop_lat, drop_lon, 50, altitude - 10)
                --next_wpt_alt = wpt_alt
                gcs:send_text(6, 
                    string.format("DropLat=%.6f, DropLon=%.6f", drop_lat, drop_lon))
                gcs:send_text(6,
                    string.format("New WP: lat=%.6f, lon=%.6f, alt=%.1f",
                        wpt_lat, wpt_lon, wpt_alt))

                -- D) Clear and upload the new mission
                assert(mission:clear(), 'Could not clear current mission')

                create_and_upload_mission(seq, HOME_LAT, HOME_LON, home:alt(), 16)
                seq = seq+1
                create_and_upload_mission(seq, -35.3579062, 149.1646814, 150, 16)
                -- next_wpt_alt = 150
                wpt_lat_temp, wpt_lon_temp, wpt_alt_temp = calc_intermediate_waypoint(HOME_LAT, HOME_LON, -35.3579062, 149.1646814, 1000, altitude + 15)
                create_and_upload_mission(seq+1, wpt_lat_temp, wpt_lon_temp, wpt_alt_temp, 16)

                vehicle:set_mode(10)
                -- seq = seq + 1
                
                print("Wpt_alt is " .. tostring(wpt_alt))
                local item = mission:get_item(seq)
                gcs:send_text(6,string.format("Current WP: lat=%.6f, lon=%.6f, alt=%.1f, seq_num=%d",item:x()* 1.0e-7, item:y()* 1.0e-7, item:z(), mission:get_current_nav_index()))
                gcs:send_text(6,string.format("Current location: lat=%.6f, lon=%.6f, alt=%.1f, seq_num=%d",curr_lat, curr_lon, altitude , mission:get_current_nav_index()))
                gcs:send_text(6, "Item Z value is this " .. tostring(item:z()))
                check_home = ahrs:get_home()
                gcs:send_text(6, "home alt is this " .. tostring(check_home:alt()))
                print(mission:get_current_nav_index())
            end
        end

        -- If altitude has increased beyond past_altitude, update it
        if altitude > past_altitude then
            past_altitude = altitude
           -- gcs:send_text(5, string.format("New altitude: %.2f meters", altitude))
        end
    end

    ---------------------------------------------------------
    -- PART B: If drop is detected (flag=1) but no RTL yet
    ---------------------------------------------------------
    if current_state >= STATE_DROP_DETECTED then
        local item = mission:get_item(seq)
        -- print("This is the seq when drop has happened " .. tostring(seq))
        -- print("Item z value of the waypoint " .. tostring(item:z()))
        if altitude < 55 and current_state == STATE_DROP_DETECTED then
            current_state = STATE_RTL
            vehicle:set_mode(11)   -- e.g. 11 = SPORT or custom RTL
            flag_rtl = 1
            gcs:send_text(6, "Bro its RTL time...")
            return update, FREQUENCY
        
        elseif compute_distance(curr_lat, curr_lon, HOME_LAT, HOME_LON) < 100 and current_state == STATE_RTL then
            vehicle:set_mode(10)
            assert(mission:clear(), 'Could not clear current mission')
            create_and_upload_mission(0, HOME_LAT, HOME_LON, HOME_ALT, 16)
            create_and_upload_mission(1, LAND_LAT, LAND_LON, LAND_ALT, 16)
            create_and_upload_mission(2, LAND_LAT, LAND_LON, LAND_ALT, 189)
            create_and_upload_mission(3, LAND_LAT, LAND_LON, LAND_ALT, 21)
            vehicle:set_mode(10)
            gcs:send_text(6, "Bro its landing time...")
            flag_land = 1 
            return update, FREQUENCY

        elseif mission:get_current_nav_index() == seq + 1 and current_state == STATE_DROP_DETECTED then
            -- Next waypoint: 1km away, altitude minus 10
            gcs:send_text(6, string.format("nav_index: %d", mission:get_current_nav_index()))
            wpt_lat_temp, wpt_lon_temp, wpt_alt_temp =
                calc_intermediate_waypoint(HOME_LAT, HOME_LON, wpt_lat_temp, wpt_lon_temp, 100, altitude - 20)
            seq = seq + 1
            create_and_upload_mission(seq+1, wpt_lat_temp, wpt_lon_temp, wpt_alt_temp, 16)
            -- next_wpt_alt = wpt_alt_temp
            gcs:send_text(6, string.format("Boom lat and lon are %.6f and %.6f", curr_lat, curr_lon))
            gcs:send_text(6, string.format("going to %.6f, %.6f, %.6f", wpt_lat_temp, wpt_lon_temp, wpt_alt_temp))

        elseif altitude < item:z() and current_state == STATE_DROP_DETECTED then
            seq = 0
            local wpt_alt = reset_mission(curr_lat, curr_lon, altitude)
            seq = seq + 1
            vehicle:set_mode(10)
            -- next_wpt_alt = wpt_alt
            gcs:send_text(6,string.format("alti boi")) 
            -- gcs:send_text(6,
            -- string.format("Alti dropped fast. Updated WP: lat=%.6f, lon=%.6f, alt=%.1f, seq_num=%d",
            --     wpt_lat, wpt_lon, wpt_alt, mission:get_current_nav_index()))
            --     gcs:send_text(6,
            --     string.format("Alti dropped fast. These are: altitude=%.6f, next_wpt_alt=%.6f", altitude, next_wpt_alt))
        
        elseif compute_distance(HOME_LAT, HOME_LON,
                                item:x()*1.0e-7, item:y()*1.0e-7)
               > compute_distance(HOME_LAT, HOME_LON, curr_lat, curr_lon) and current_state == STATE_DROP_DETECTED then
            -- We are further from home than the waypoint, so update
            gcs:send_text(6,string.format("close close pt pt"))    
            gcs:send_text(6, string.format("Destination %.6f, %.6f", item:x()*1.0e-7, item:y()*1.0e-7))
            gcs:send_text(6, string.format("Current %.6f, %.6f, seq num %d", curr_lat, curr_lon, seq))
            gcs:send_text(4,
                    string.format("2 compute distances are %f and %f", compute_distance(HOME_LAT, HOME_LON,
                                item:x()*1.0e-7, item:y()*1.0e-7),compute_distance(HOME_LAT, HOME_LON, curr_lat, curr_lon)))
     
            seq = 0
            local wpt_alt = reset_mission(curr_lat, curr_lon, altitude)
            seq = seq + 1
            vehicle:set_mode(10)
            --next_wpt_alt = wpt_alt        
            -- gcs:send_text(4,
            --         string.format("Closer to home. Updated WP: lat=%.6f, lon=%.6f, alt=%.1f, seq_num=%d",
            --             wpt_lat1, wpt_lon1, wpt_alt1, mission:get_current_nav_index()))
        end
    end

    -- Schedule again
    return update, FREQUENCY
end

return update()
