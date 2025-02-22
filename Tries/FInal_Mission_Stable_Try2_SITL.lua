--------------------------------------------------------------------------------
-- Frequency of updates in milliseconds
--------------------------------------------------------------------------------
local FREQUENCY    = 1000
local altitude     = 0
local past_altitude = 0
local flag         = 0
local next_wpt_alt = 0
local flag_servo   = 0
------------------------
-- Define states
local STATE_IDLE = 0
local STATE_DROP_DETECTED = 1
local STATE_RTL = 2
local STATE_LANDING = 3
local STATE_RTL_ERROR = 4

local SUB_STATE_NOT_CLOSE = 0
local SUB_STATE_CLOSE = 1

-- Initialize state
local current_state = STATE_IDLE
local current_sub_state = SUB_STATE_NOT_CLOSE
--------------------------
------------------------------------------------------------------------------
-- Custom paras
local MIN_DROP_ALTI_TRIGGER = 30 --20

local HORI_DIST_DROP_WP1 = 70 --3000
local ALTI_DROP_WP1 = 10 --3000

local HORI_DIST_DROP_WP2 = 70 --1000
local ALTI_DROP_WP2 = 10 --1000


local HORI_DIST_MIS = 70 --500
local ALTI_MIS = 10

local HORI_DIST_CLOSE = 400
local ALTI_CLOSE = 100

local CMD_DO_LAND = 189
local CMD_LAND = 21 
local CMD_FBW = 6
local CMD_RTL = 20
local CMD_WP = 16

local MODE_RTL = 11
local MODE_MIS = 10
local MODE_FBW = 6

local MIN_LAND_DIST = 5000
local DROP_ALT = 28000


local DROP_TIME = 0
local DROP_TIMEOUT_MS =  9000 * 1000  -- 150 minutes
--local DROP_TIMEOUT_MS2 = 60 * 1000

local TRIM_THROT_LOW = 0
local TRIM_THROT_MED = 0
local TRIM_THROT_HIGH = 35

local MIN_ALTI_RTL = 600


--------------------------------------------------------------------------------
-- Example home coordinates (replace with actual or fetch from ahrs:get_home())
--------------------------------------------------------------------------------
local HOME_LAT = 17.5447206
local HOME_LON = 78.0420480
local HOME_ALT = 0

local LAND_LAT = 17.548117
local LAND_LON = 78.043168
local LAND_ALT = 40.000000

local LAND_LAT10 = 17.546699
local LAND_LON10 = 78.044407
local LAND_ALT10 = 500.000000

local LAND_LAT9 = 17.542846
local LAND_LON9 = 78.043745
local LAND_ALT9 = 350.000000

local LAND_LAT8 = 17.542491
local LAND_LON8 = 78.040573
local LAND_ALT8 = 250.000000

local LAND_LAT7 = 17.545391
local LAND_LON7 = 78.042197
local LAND_ALT7 = 150.000000

local LAND_LAT6 = 17.545842
local LAND_LON6 = 78.044212
local LAND_ALT6 = 90.000000

local LAND_LAT5 = 17.548286
local LAND_LON5 = 78.044999
local LAND_ALT5 = 60.000000

local LAND_LAT4 = 17.550662
local LAND_LON4 = 78.043901
local LAND_ALT4 = 50.000000


local LAND_LAT3 = 17.548117
local LAND_LON3 = 78.043168
local LAND_ALT3 = 40.000000

local LAND_LAT2 = 17.547867
local LAND_LON2 = 78.043089
local LAND_ALT2 = 32.000000

local LAND_LAT1 = 17.547423
local LAND_LON1 = 78.042957
local LAND_ALT1 = 25.000000
--------------------------------------------------------------------------------
-- Array of possible "home" coordinates 26.762544, 80.984242  26.760941, 80.985321
-- 26.761646, 80.986101
--26.762839, 80.983783
--------------------------------------------------------------------------------
local possible_homes = {
    {
        home = {lat = 17.5447206, lon = 78.0420480},
        landing = {lat = 17.548117, lon = 78.043168}
    },-- Add more entries as needed
    {
        home = {lat = 17.5447206, lon = 78.0420480},
        landing = {lat = 17.548117, lon = 78.0428869}
    }
}

--------------------------------------------------------------------------------
-- We'll store the drop lat/lon once we detect the altitude drop.

--------------------------------------------------------------------------------
local drop_lat = 0
local drop_lon = 0
local seq = 0

local wpt_lat_temp, wpt_lon_temp, wpt_alt_temp

--------------------------------------------------------------------------------
-- Change parameters


local RELAY_NUM = 54 -- Replace with your desired relay number (e.g., 0 for RELAY_PIN)

local TRIM_THROT = Parameter()      --creates a parameter object
TRIM_THROT:init('TRIM_THROTTLE')       --get the physical location in parameters memory area so no future name search is done
TRIM_THROT:set_and_save(TRIM_THROT_HIGH)  --retrieve that parameters value and assign to "parameter"

----------------------------
-- local F_SERVO = 5
-- local servo_channel = assert(SRV_Channels:find_channel(F_SERVO))
-- SRV_Channels:set_output_pwm_chan_timeout(servo_channel, 1900,10000)
--------------------------------------------------------------------------------
-- Earth radius (for the calc_intermediate_waypoint logic)
--------------------------------------------------------------------------------
local R = 6371000  -- meters

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
    vehicle:set_mode(MODE_FBW)   -- Replace '6' with the appropriate mode number if different

    -- Clear existing mission items
    assert(mission:clear(), 'Could not clear current mission')

    -- Initialize sequence number
    -- seq = 0

    -- Upload Home Waypoint
    create_and_upload_mission(0, HOME_LAT, HOME_LON, HOME_ALT, CMD_WP)
    --  seq = seq + 1

    -- Compute and upload first intermediate waypoint 500 meters from home towards current location
    local wpt_lat1, wpt_lon1, wpt_alt1 =
        calc_intermediate_waypoint(HOME_LAT, HOME_LON, curr_lat, curr_lon, HORI_DIST_MIS, altitude - ALTI_MIS)
    create_and_upload_mission(1, wpt_lat1, wpt_lon1, wpt_alt1, CMD_WP)
    
    --create_and_upload_mission(1, -35.3061950, 149.1933918, wpt_alt1)
    -- Compute and upload second intermediate waypoint 500 meters from the first waypoint
    local wpt_lat2, wpt_lon2, wpt_alt2 =
        calc_intermediate_waypoint(HOME_LAT, HOME_LON, wpt_lat1, wpt_lon1, HORI_DIST_MIS, altitude - ALTI_MIS)
    
    wpt_lat_temp, wpt_lon_temp, wpt_alt_temp = wpt_lat2, wpt_lon2, wpt_alt2

    create_and_upload_mission(2, wpt_lat2, wpt_lon2, wpt_alt2, CMD_WP)
    return wpt_alt1
end


--------------------------------------------------------------------------------
-- Main update function with error handling
--------------------------------------------------------------------------------
function update()
    -- Wrap the entire update logic in a protected call to catch any errors
    local status, err = pcall(function()
        -- 1) Check if AHRS is healthy
        if not ahrs:healthy() then
            gcs:send_text(5, "Waiting for AHRS initialization...")
            return
        end

        -- 2) Get current location and NED-home distance
        local location = ahrs:get_location()
        local dist     = ahrs:get_relative_position_NED_home()

        if (not location) or (not dist) then
            gcs:send_text(5, "Location or NED data not available.")
            -- return
        end

        -- Convert 'down' to 'up' for altitude
        altitude = -1 * dist:z()

        -- Current lat/lon in degrees
        local curr_lat = location:lat() * 1.0e-7
        local curr_lon = location:lng() * 1.0e-7

        if arming:is_armed() then
            if DROP_TIME == 0 then
                DROP_TIME = millis()
                gcs:send_text(5, "Vehicle armed. Starting 10-minute timer.")
            end
        else
            -- Disarmed => reset
            if DROP_TIME ~= 0 then
                DROP_TIME = 0
                gcs:send_text(5, "Vehicle disarmed. Timer reset.")
            end
        end

        ---------------------------------------------------------
        -- PART A: Detect altitude drop => "drop" event
        ---------------------------------------------------------
        
        if current_state == STATE_IDLE then
            -- If altitude + MIN_DROP_ALTI_TRIGGER < past_altitude => significant drop
            -- if altitude > DROP_ALT then
            --     SRV_Channels:set_output_pwm_chan_timeout(servo_channel, 1100,10000)
            --     gcs:send_text(6, "Servo Moved")
            --     flag_servo = 1
            -- end

            -- if DROP_TIME ~= 0 and (millis() - DROP_TIME) >= DROP_TIMEOUT_MS then
            --     gcs:send_text(4, "10 minutes since arming have passed. Taking action.")
            --     SRV_Channels:set_output_pwm_chan_timeout(servo_channel, 1100,10000)
            --     gcs:send_text(6, "Servo Moved")
            --     flag_servo = 1
            -- end

            if DROP_TIME ~= 0 and (millis() - DROP_TIME) >= DROP_TIMEOUT_MS2 then
                
                if relay:enabled(RELAY_NUM) then
                    relay:on(RELAY_NUM) -- Turn on the relay
                    gcs:send_text(6, "Relay turned ON")
                else
                    gcs:send_text(4, "Relay is not enabled!")
                end

            end


            if altitude + MIN_DROP_ALTI_TRIGGER < past_altitude then
                
                --gcs:send_text(6, string.format("Time is %.6f", (millis() - DROP_TIME)/1000))
                -- if flag_servo == 0 then
                --     SRV_Channels:set_output_pwm_chan_timeout(servo_channel, 1100,10000)
                --     gcs:send_text(6, "Servo Move Earlier---")
                --     flag_servo = 1
                -- end

                current_state = STATE_DROP_DETECTED
                vehicle:set_mode(MODE_FBW)
                gcs:send_text(6, "Drop detected. Switching to RTL...")
                TRIM_THROT:set_and_save(TRIM_THROT_LOW)
                -- A) Save the drop position
                drop_lat = curr_lat
                drop_lon = curr_lon


                local home = ahrs:get_home()
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

                -- C) Compute the new waypoint
                local wpt_lat, wpt_lon, wpt_alt =
                    calc_intermediate_waypoint(HOME_LAT, HOME_LON, drop_lat, drop_lon, HORI_DIST_DROP_WP1, altitude - ALTI_DROP_WP1)

                -- D) Clear and upload the new mission
                assert(mission:clear(), 'Could not clear current mission')

                create_and_upload_mission(seq, HOME_LAT, HOME_LON, home:alt(), CMD_WP)
                seq = seq+1
                create_and_upload_mission(seq, wpt_lat, wpt_lon, wpt_alt, CMD_WP)
                -- next_wpt_alt = 150
                wpt_lat_temp, wpt_lon_temp, wpt_alt_temp = calc_intermediate_waypoint(HOME_LAT, HOME_LON, wpt_lat, wpt_lon, HORI_DIST_DROP_WP2, altitude - ALTI_DROP_WP2)
                create_and_upload_mission(seq+1, wpt_lat_temp, wpt_lon_temp, wpt_alt_temp, CMD_WP)

                vehicle:set_mode(MODE_MIS)
                -- seq = seq + 1
                
                local item = mission:get_item(seq)
                local check_home = ahrs:get_home()
                
            end

            -- If altitude has increased beyond past_altitude, update it
            if altitude > past_altitude then
                past_altitude = altitude
               gcs:send_text(5, string.format("New altitude: %.2f meters", altitude))
            end
        end

        ---------------------------------------------------------
        -- PART B: If drop is detected (flag=1) but no RTL yet
        ---------------------------------------------------------
        local item = mission:get_item(seq)
        if current_state == STATE_DROP_DETECTED then
            if altitude < MIN_ALTI_RTL then
                current_state = STATE_RTL
                -- vehicle:set_mode(MODE_RTL)   -- e.g. 11 = SPORT or custom RTL
                TRIM_THROT:set_and_save(TRIM_THROT_HIGH)
                vehicle:set_mode(MODE_FBW)
                assert(mission:clear(), 'Could not clear current mission')
                create_and_upload_mission(0, HOME_LAT, HOME_LON, HOME_ALT, CMD_WP)
                create_and_upload_mission(1, HOME_LAT, HOME_LON, 0, CMD_RTL)
                create_and_upload_mission(2, HOME_LAT, HOME_LON, 0, CMD_DO_LAND)
                
                create_and_upload_mission(3, LAND_LAT10, LAND_LON10, LAND_ALT10, CMD_WP)
                create_and_upload_mission(4, LAND_LAT9, LAND_LON9, LAND_ALT9, CMD_WP)
                create_and_upload_mission(5, LAND_LAT8, LAND_LON8, LAND_ALT8, CMD_WP)
                create_and_upload_mission(3, LAND_LAT7, LAND_LON7, LAND_ALT7, CMD_WP)

                create_and_upload_mission(4, LAND_LAT6, LAND_LON6, LAND_ALT6, CMD_WP)
                create_and_upload_mission(5, LAND_LAT5, LAND_LON5, LAND_ALT5, CMD_WP)
                create_and_upload_mission(6, LAND_LAT4, LAND_LON4, LAND_ALT4, CMD_WP)

                create_and_upload_mission(7, LAND_LAT3, LAND_LON3, LAND_ALT3, CMD_WP)
                create_and_upload_mission(8, LAND_LAT2, LAND_LON2, LAND_ALT2, CMD_WP)
                create_and_upload_mission(9, LAND_LAT1, LAND_LON1, LAND_ALT1, CMD_WP)
                create_and_upload_mission(10, HOME_LAT, HOME_LON, 0, CMD_LAND)
                vehicle:set_mode(MODE_MIS)
                gcs:send_text(6, string.format("Going to land at Lat=%.6f, Lng=%.6f", LAND_LAT, LAND_LON))
                -- gcs:send_text(6, "RTL enabled")
                return
            elseif mission:get_current_nav_index() == seq + 1 and current_sub_state == SUB_STATE_NOT_CLOSE then
                 -- Next waypoint: 1km away, altitude minus 10
                 wpt_lat_temp, wpt_lon_temp, wpt_alt_temp =
                 calc_intermediate_waypoint(HOME_LAT, HOME_LON, wpt_lat_temp, wpt_lon_temp, HORI_DIST_MIS, altitude - ALTI_MIS)
                 seq = seq + 1
                 create_and_upload_mission(seq+1, wpt_lat_temp, wpt_lon_temp, wpt_alt_temp, CMD_WP)
                 gcs:send_text(6, string.format("Next wp at Lat=%.6f, Lng=%.6f", wpt_lat_temp, wpt_lon_temp))
            elseif mission:get_current_nav_index() == seq + 1 and current_sub_state == SUB_STATE_CLOSE then
                -- Next waypoint: 1km away, altitude minus 10
                wpt_lat_temp, wpt_lon_temp, wpt_alt_temp =
                calc_intermediate_waypoint(HOME_LAT, HOME_LON, drop_lat, drop_lon, HORI_DIST_CLOSE * state, altitude - ALTI_CLOSE)
                seq = seq + 1
                create_and_upload_mission(seq+1, wpt_lat_temp, wpt_lon_temp, wpt_alt_temp, CMD_WP)
                state = -1 * state
                gcs:send_text(6, string.format("Close to home, wp is at Lat=%.6f, Lng=%.6f", wpt_lat_temp, wpt_lon_temp))

            elseif altitude < item:z() then
                seq = 0
                gcs:send_text(6, string.format("Current alti=%.6f, wp_alti=%.6f", altitude, item:z()))
                local wpt_alt = reset_mission(curr_lat, curr_lon, altitude)
                seq = seq + 1
                vehicle:set_mode(MODE_MIS)
                
            elseif compute_distance(HOME_LAT, HOME_LON,item:x()*1.0e-7, item:y()*1.0e-7) > compute_distance(HOME_LAT, HOME_LON, curr_lat, curr_lon) and current_state == STATE_DROP_DETECTED then
                -- We are further from home than the waypoint, so update
                seq = 0
                local wpt_alt = reset_mission(curr_lat, curr_lon, altitude)
                seq = seq + 1
                vehicle:set_mode(MODE_MIS)
                gcs:send_text(6, "Closer to home than WP is close.")
            end
        end

        local check_dist = compute_distance(curr_lat, curr_lon, HOME_LAT, HOME_LON)
        gcs:send_text(6, string.format("THis is dist %.6f",check_dist))
        if check_dist < MIN_LAND_DIST and altitude > CLOSE_ALT and current_sub_state == SUB_STATE_NOT_CLOSE then
            TRIM_THROT:set_and_save(TRIM_THROT_MED)
            current_sub_state = SUB_STATE_CLOSE
            
            gcs:send_text(6, "Sub State now close to home!")
        end
        
        -- Schedule again
    end)

    if not status and current_state ~= STATE_RTL_ERROR then
        -- An error occurred; set vehicle mode to RTL
        -- TRIM_THROT:set_and_save(TRIM_THROT_HIGH)
        -- vehicle:set_mode(MODE_RTL)
        -- current_state = STATE_RTL_ERROR
        gcs:send_text(3, "Error occurred: " .. tostring(err) .. ". Switching to RTL.")
    end

    -- Schedule the next update
    return update, FREQUENCY
end

return update()
