from pymavlink import mavutil
import os
import sys
import math
import time
import json
import serial

# Variables
FORWARD_SPEED = 5.0   # m/s, cruise speed while the drone is flying
MAX_YAW = 0.6         # rad/s at full stick to turn left or right

# Create an absolute path for cmd.txt so Python can always find it
CMD_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cmd.txt")

# Test cases
home1 = [37.622465, -122.373455, 4.64]
home2 = [37.6217268, -122.37128, 3.64]

waypoints_1 = [ # Form a triangle pattern
    ("takeoff", 0.0, 0.0, 5.0),   # lat/lon 0 = take off from current position
    ("waypoint", 37.6241063, -122.3758292, 5.0),
    ("waypoint", 37.6224577, -122.3761082, 5.0),
    ("rtl", 0.0, 0.0, 0.0),
]

waypoints_2 = [ # Form a square pattern
    ("takeoff", 0.0, 0.0, 5.0),   # lat/lon 0 = take off from current position
    ("waypoint", 37.6237324, -122.3723531, 5.0),
    ("waypoint", 37.6231205, -122.3707438, 5.0),
    ("waypoint", 37.6218459, -122.3715162, 10.0),
    ("rtl", 0.0, 0.0, 0.0),
]

waypoints_3 = [ # Form a 5-pt diamond-ish pattern
    ("takeoff",  0.0, 0.0, 10.0),   # lat/lon 0 = take off from current position
    ("waypoint", 37.6250495, -122.3715699, 15.0),
    ("waypoint", 37.6253639, -122.3733294, 20.0),
    ("waypoint", 37.6249135, -122.3748529, 20.0),
    ("waypoint", 37.6236304, -122.374649, 10.0),
    ("rtl", 0.0, 0.0, 0.0),
]

# Struct
# Telemetry from drone - Quad Multirotor
drone = {
    "Latitude": 0,
    "Longitude": 0,
    "Altitude": 0,
    "Ground Speed": 0,
    "Voltage": 0,
}

##########################################################################################
# DEFINE SUPPORT FUNCTIONS HERE!!!
##########################################################################################
# Function to read joysticks
def read_joystick(ser):
    line = None
    while (ser.in_waiting):
        raw_data = ser.readline()
        line = raw_data.decode("utf-8", errors="ignore").strip() # Read bytes buffer sent from serial
        print(f"Arduino line: {line}") # Should be 500,0 constantly
    if (not line):
        return None

    # Split Voltage reading and button state out
    line_split = line.split(",")
    x = int(line_split[0])
    button = int(line_split[1])
    if (x is None and button is None):
        raise ValueError
    
    # Error check
    x = (x - 515) / 515 # Prevent negative value
    if (abs(x) < 0.1):
        x = 0.0
    return x, button

# Function to send a velocity + yaw rate command (body frame, so "forward" is where the nose points)
def send_velocity(target, forward, right, up, yaw_rate):
    # send local position, velocity, or acceleration targets to a drone
    target.mav.set_position_target_local_ned_send(
        0, # Timeboot in ms
        target.target_system,
        target.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        # Set to ignore
        # Don't need location 0-2
        # Need velocity 3-5
        # Don't need acceleration 6-8
        # Normal set point 9
        # Yaw angle is ignored 10
        # Need yaw rate for turning 11
        0b0_000_010_111_000_111, # Use velocity + yaw rate
        0, 0, 0, # position (ignored)
        forward, 0, 0,#right, -up, # Velocity (NED, so up is negative)
        0, 0, 0, # acceleration (ignored)
        0, yaw_rate # yaw (ignored), yaw rate
    )

# Function to execute change in path flight
def go_here(target, lat, long, alt):
    # Set mode to guided while flying
    target.set_mode("GUIDED")

    # Send command to the drone 
    target.mav.command_int_send(
        target.target_system,
        target.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0,
        0,
        -1,
        1, # Guide 
        0,
        float("nan"),
        int(lat * 1e7),
        int(long * 1e7),
        float(alt)
    )

# Function to get distance in meters between two points
def distance_m(lat1, lon1, lat2, lon2):
    dy = (lat2 - lat1) * 111320
    dx = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
    return math.hypot(dx, dy)

# Function to read the next new line from cmd.txt (returns command list and new file position)
def read_command(pos):
    # Open cmd.txt file
    with open(CMD_FILE, "r") as file:
        file.seek(pos)
        line = file.readline()
        if (line.endswith("\n")):
            return line.strip().split(), file.tell() # return a list of each command 
    return None, pos

# Function to parse the .json file 
def json_parser(cmd_path):
    with open(cmd_path) as file:
        config_file = json.load(file)

    # Grabbing home coordinates to set new home as a list
    home = [config_file["home"]["lat"], config_file["home"]["long"], config_file["home"]["alt"]]

    # Loop through every item in mission and save them to mission list
    mission = []
    for item in config_file["mission"]:
        cmd = item["type"] 
        if (cmd == "takeoff"):
            mission.append(("takeoff", item["lat"], item["long"], item["alt"]))
        elif (cmd == "waypoint"):
            if ("speed" in item):
                mission.append(("speed", item["speed"]))
            mission.append(("waypoint", item["lat"], item["long"], item["alt"]))
        elif (cmd == "rtl"):
            mission.append(("rtl", item["lat"], item["long"], item["alt"]))
        else:
            raise ValueError("Unknown mission item")

    # Return connection, home coordiate, and mission to plan
    return config_file["connection"], home, mission

# Function to request telemetry info at x rate
def request_message(target, message_id: int, rate):
    interval_us = int(1000000 / rate) # Rate at which the message arrives

    # Request
    target.mav.command_long_send(
        target.target_system,
        target.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id, # message ID
        interval_us, 0, 0, 0, 0, 0
    )

# Function to arm and disarm
def arm_disarm(target, mode: int):
    # Request
    target.mav.command_long_send(
        target.target_system,
        target.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        mode, # 0 for disarm | 1 for arm
        0, 0, 0, 0, 0, 0
    )

# Function to start the mission
def start_mission(target):
    target.mav.command_long_send(
        target.target_system,
        target.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# Function to set home
def set_home(target, coordinates: list):
    lat = coordinates[0] * 1e7
    long = coordinates[1] * 1e7
    alt = coordinates[2]

    # Sent command to FC
    target.mav.command_int_send(
        target.target_system,
        target.target_component, 
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, # current
        0, # autocontinue
        0, # Use the coordinates below as home
        0, 0, 0,
        int(lat),
        int(long),
        float(alt) 
    )

    # Wait from response from FC
    home_ack = target.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)

    # Error check
    if (home_ack is None):
        print("Set home failed...")
        return False
    
    print("New Home set!")
    return True

# Function to add or set WP
def mission_upload(target, waypoints):
    # Adding a placeholder seq 0
    waypoints = [("home", 0.0, 0.0, 0.0)] + list(waypoints)

    # Let the flight controller know how mant mission items we're uploading
    target.mav.mission_count_send(
        target.target_system,
        target.target_component,
        len(waypoints), # a list
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION # enum type
    )

    # Looping through the given WP list
    for i in range(len(waypoints)):
        # Wait for response
        mission_msg = target.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
            blocking=True,
            timeout=3
        )

        # Error check
        if mission_msg is None:
            print("No mission message arrived...")
            return False

        seq_number = mission_msg.seq # Grab the item sequence number
        waypoint = waypoints[seq_number] # Grab the WP of seq_num

        if waypoint[0] in ("home", "waypoint"):
            cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        elif waypoint[0] == "takeoff":
            cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        elif waypoint[0] == "rtl":
            cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        elif waypoint[0] == "speed":
            # Send change of speed to the drone
            target.mav.mission_item_int_send(
                target.target_system,
                target.target_component,
                seq_number,
                mavutil.mavlink.MAV_FRAME_MISSION,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0, # current
                1, # autocontinue
                1, # ground speed
                float(waypoint[1]), # param2: speed in m/s
                -1, # throttle, no change
                0, 0, 0, 0.0
            )
            continue
        else:
            print("Unknown mission item")
            return False

        lat = int(waypoint[1] * 1e7)
        lon = int(waypoint[2] * 1e7)
        alt = float(waypoint[3])  

        print("-----------------------------------------------")
        print(f"Sending seq={mission_msg.seq}, "
            f"cmd={cmd}, "
            f"lat={lat}, "
            f"lon={lon}, "
            f"alt={alt}"
            )
        print("-----------------------------------------------")

        # Upload command back to the drone
        target.mav.mission_item_int_send(
            target.target_system,
            target.target_component,
            mission_msg.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            cmd,
            0,    # current
            1,    # autocontinue
            0.0,  # param1
            0.0,  # param2
            0.0,  # param3
            0.0,  # param4
            int(lat),
            int(lon),
            float(alt)
        )

    # Wait for the drone to confirm the upload ack
    upload_ack = target.recv_match(type="MISSION_ACK", blocking=True, timeout=3)

    if (upload_ack):   
        return upload_ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
    else:
        print("Upload Ack failed")
        return False
##########################################################################################


##########################################################################################
# RUN MAIN
##########################################################################################
def main():
    # Establish serial connection to the joystick
    com_port = sys.argv[2]
    if not com_port:
        print("Usage: python mvlinkScript.py mission.json COM# or mvlinkScript.py tcp:LOCAL_HOST_IP:ArduPilot_PORT COM#")
        sys.exit(1)

    serialConnection = serial.Serial(com_port, 115200, timeout=0)   # check Device Manager for your port

    last_joy = time.time()
    prev_button = 0
    cruising = False # This is toggled by the stick button

    # Slow down the print statements 
    last_print = 0.0
    PRINT_INTERVAL = 5.0 # 5s per print

    # Commandline arguments to parse device connection
    device = None
    if (len(sys.argv) > 3):
        print("Usage: python mvlinkScript.py mission.json COM# or mvlinkScript.py tcp:LOCAL_HOST_IP:ArduPilot_PORT COM#")
        sys.exit(1)
    elif (len(sys.argv) == 3):
        if (sys.argv[1] == "mission.json"):
            # Grab stuff from .json
            json_file = sys.argv[1] 
            device, home, mission = json_parser(json_file)
        else: 
            device = sys.argv[1]

    # Establish connection to the flight controller
    mavConnect = mavutil.mavlink_connection(device)

    # Wait for response from the system
    heartbeat = mavConnect.wait_heartbeat(timeout=5)
    if (heartbeat):
        print("Connection: Connected")
    else:
        print("Not connecting...")

    # Request heartbeat
    request_message(mavConnect, 0, 5)

    # Only set home and upload mission plan if mission.json filed is given
    if (len(sys.argv) == 2 and (sys.argv[1] == "mission.json")):
        # Set Home
        set_home(mavConnect, home)

        ##################################################################################
        # Uploading missions Section
        ##################################################################################
        is_upload = mission_upload(mavConnect, mission)
        if not is_upload:
            print("Mission upload failed")
            return
        print("Success")

    ##################################################################################
    # Get number of mission items
    ##################################################################################
    mavConnect.mav.mission_request_list_send(mavConnect.target_system, mavConnect.target_component)

    # Wait to receive the mission count response
    mission_msg = mavConnect.recv_match(type="MISSION_COUNT", blocking=True, timeout=3)
    if (mission_msg):
        print(f"Number of missions: {mission_msg.count}")
    else:
        print("No mission count received")

    # ##################################################################################
    # # Download mission items for verification
    # ##################################################################################
    for seq in range(3):
        mavConnect.mav.mission_request_int_send(
            mavConnect.target_system,
            mavConnect.target_component,
            seq,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        msg = mavConnect.recv_match(
            type=["MISSION_ITEM_INT", "MISSION_ITEM"],
            blocking=True,
            timeout=5
        )

        if msg:
            print(f"Stored seq: {msg.seq}")
            print(f"Command: {msg.command}")
            print(f"Altitude: {msg.z}")
            print("-------------------")
        else:
            print(f"Could not retrieve mission item {seq}")

    ##################################################################################
    # Arm/disarm Section
    ##################################################################################
    arm_disarm(mavConnect, 1) # command to arm

    # Receive COMMAND_ACK comfirmation
    ack_msg = mavConnect.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if (ack_msg):
        if (ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
            print("Ack Result: ", ack_msg.result)
            print("Arm command accepted")
        else:
            print(f"Arm command rejected. Command result: {ack_msg.result}")

    # Get heartbeat to check current status of the drone
    heartbeat_msg = mavConnect.recv_match(type="HEARTBEAT", blocking=True, timeout=3)

    # Check if the drone is armed
    if (heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("Drone is armed!")
        start_mission(mavConnect)

        # Read messages from the flight controller
        msg = mavConnect.recv_match(type="STATUSTEXT", blocking=True, timeout=3)
        if msg:
            print("FC MESSAGE:", msg.text)
        else:
            print("No STATUSTEXT message received")
    else:
        print("Drone is still disarmed")

        # Check for mode
        current_mode = mavutil.mode_string_v10(heartbeat_msg)
        print(f"Current mode: {current_mode}")
        
        # Switch to guided mode if it's auto then it will auto itself
        if (current_mode != "GUIDED"):
            mavConnect.set_mode("GUIDED")
            print("  ")
            print("Re-arming the drone")
            arm_disarm(mavConnect, 1) # re-arm
            start_mission(mavConnect) # Start mission

    # Create and clear command line file
    open(CMD_FILE, "w").close()
    cmd_pos = 0

    # Variables to keep track of the drone mode for flight path/destination change
    state = "MISSION"   # MISSION or DETOUR
    target_point = None

    # Request telemetry messages
    request_message(mavConnect, 33, 5) # GLOBAL_POSITION_INT
    request_message(mavConnect, 74, 1) # VFR_HUD
    request_message(mavConnect, 147, 1) # BATTERY_STATUS

    ##################################################################################
    # Loop it!!! Get drone's real-time data info
    ##################################################################################
    mission_completed = False 
    was_armed = False
    while not mission_completed:
        # Check for a new command from cmd.txt
        cmd, cmd_pos = read_command(cmd_pos)
        if (cmd):
            if (cmd[0] == "goto" and len(cmd) == 4):
                target_point = (float(cmd[1]), float(cmd[2]), float(cmd[3]))
                go_here(mavConnect, target_point[0], target_point[1], target_point[2])
                state = "DETOUR"
            elif (cmd[0] == "resume"):
                mavConnect.set_mode("AUTO")
                state = "MISSION"
            elif (cmd[0] == "rtl"):
                mavConnect.set_mode("RTL")
                state = "RTL"
            elif (cmd[0] == "joy"):
                mavConnect.set_mode("GUIDED")
                cruising = False
                state = "MANUAL"
                print("Manual control is now on.")
                print("Press the button to start/stop moving")
                print("Move the button up/down to turn right/left")
            elif (cmd[0] == "joyoff"):
                send_velocity(mavConnect, 0, 0, 0, 0)
                mavConnect.set_mode("AUTO")      # resume the mission (use "LOITER" to just hold)
                state = "MISSION"
            else:
                print("Usage: goto lat long alt | resume | rtl")

        # Print real-time telemetry every 5 seconds
        now = time.time()
        if now - last_print >= PRINT_INTERVAL:
            last_print = now
            print("-------------------------------------------------------------------------")
            print( f"| Lat {drone['Latitude']:.6f} | Long {drone['Longitude']:.6f} | "
                f"Alt {drone['Altitude']:.1f}m | "
                f"GS {drone['Ground Speed']:.1f} m/s | "
                f"Batt {drone['Voltage']:.1f}V |")
            print("-------------------------------------------------------------------------")

        if (state == "MANUAL"):
            joy = read_joystick(serialConnection)
            if (joy):
                x, button = joy
                last_joy = time.time()

                # Button press (not hold) toggles moving on/off
                if (button == 1 and prev_button == 0):
                    cruising = not cruising
                    if (cruising):
                        print("Moving")
                    else:
                        print("Stopped")
                prev_button = button

                if (cruising):
                    # Stick right (x > 0) -> positive yaw rate -> nose turns right
                    send_velocity(mavConnect, FORWARD_SPEED, 0, 0, x*MAX_YAW) 
                else:
                    send_velocity(mavConnect, 0, 0, 0, x*MAX_YAW) # Turn in place
            elif (time.time() - last_joy > 0.5):
                cruising = False
                send_velocity(mavConnect, 0, 0, 0, 0)                  # no data -> stop

        # If no msgs received, then just continue
        # Request message
        msg = mavConnect.recv_match(blocking=True, timeout=0.2)
        if (msg is None):
            print("No message received")
            continue
        msg_type = msg.get_type()

        if (msg_type == "GLOBAL_POSITION_INT"):
            drone['Latitude'] = msg.lat * 1e-7
            drone['Longitude'] = msg.lon * 1e-7
            drone['Altitude'] = msg.relative_alt / 1000

            # Reached the detour point? Go back to the mission
            if (state == "DETOUR"):
                dist = distance_m(drone['Latitude'], drone['Longitude'], target_point[0], target_point[1])
                if (dist < 2.0):
                    print("Detour done, resuming mission")
                    mavConnect.set_mode("AUTO")
                    state = "MISSION"
        elif (msg_type == "VFR_HUD"):
            drone['Ground Speed'] = msg.groundspeed
        elif (msg_type == "BATTERY_STATUS"):
            drone['Voltage'] = msg.voltages[0]/1000.0
            drone['Current'] = msg.current_battery/100.0
        elif (msg_type == "MISSION_ITEM_REACHED"):
            print(f"Mission item reached: {msg.seq}")
            if msg.seq == mission_msg.count - 1:
                print("MISSION COMPLETED!!!")   
                mission_completed = True
        elif (msg_type == "HEARTBEAT"):
            if (msg.type != mavutil.mavlink.MAV_TYPE_GCS):
                armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                if (armed):
                    was_armed = True
                elif (was_armed):
                    print("Drone landed and disarmed")
                    print("MISSION COMPLETED!!!")
                    mission_completed = True
        else:
            continue

if __name__ == "__main__":
    main()