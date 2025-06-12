import time
from pymavlink import mavutil

# Connect to SITL on port 14551
connection_string = 'udp:127.0.0.1:14550'
master = mavutil.mavlink_connection(connection_string)

# Wait for the first heartbeat
master.wait_heartbeat()
print("Heartbeat received")

# Function to change mode
def set_mode(mode_name):
    # Get the mode number from the mode name using master.mode_mapping()
    mode_id = master.mode_mapping()[mode_name]
    
    if mode_id is None:
        print(f"Mode '{mode_name}' not recognized.")
        return
    
    print(f"Changing mode to: {mode_name} (ID: {mode_id})")
    
    # Set the mode
    master.set_mode(mode_id)

    # Wait until the mode is confirmed
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == mode_id:
            print(f"Mode {mode_name} changed successfully")
            break
import time

def arm_drone():
    while True:
        set_mode("GUIDED")
        print("Arming drone")
        master.arducopter_arm()  # Send arming command
        
        # Check for arming confirmation for up to 3 seconds
        armed = False
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = master.recv_match(type='HEARTBEAT', blocking=True)
            if msg:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    print("Drone armed")
                    armed = True
                    break
            time.sleep(0.1)  # Prevent high CPU usage
            
        if armed:
            break
        else:
            print("Arming failed. Retrying...")

def takeoff(altitude):
    print(f"Taking off to {altitude} meters")
    master.mav.command_long_send(master.target_system,
                                   master.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                   0,
                                   0, 0, 0, 0, 0, 0, altitude)
    
    # Wait until the drone reaches the desired altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print(msg.relative_alt / 1000)
        if msg.relative_alt / 1000 >= (altitude - 1):  # Convert from mm to m
            print("Reached target altitude")
            break

def set_position_target_local_ned(yaw=None, yaw_rate=None, vx=3, vy=0, vz=0):
    global master
    # Get the system time since boot
    start_time = time.time()
    time_boot_ms = int((time.time() - start_time) * 1000)

    # Set the coordinate frame to MAV_FRAME_BODY_NED
    coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_NED

    # Type mask for ignoring unused parameters
    type_mask = 0b011111000111  # Correct type mask for moving with velocities

    # Send the mavlink SET_POSITION_TARGET_LOCAL_NED message
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        100,
        coordinate_frame,
        type_mask,
        0, 0, 0,  # Position (ignored)
        vx, vy, vz,  # Velocity (specified by the function arguments)
        0, 0, 0,  # Acceleration (ignored)
        yaw if yaw is not None else 0,  # Yaw (if provided)
        yaw_rate if yaw_rate is not None else 0  # Yaw rate (if provided)
    )

def fly_square(side_length, velocity=3, yaw=None):
    # Move in a square with the given side length (meters), velocity (m/s), and optional yaw
    #for _ in range(4):
    set_position_target_local_ned(yaw=yaw, vx=velocity, vy=0, vz=0)  # Move forward
    time.sleep(side_length / velocity)  # Sleep for the duration of the side
    set_position_target_local_ned(yaw=yaw, vx=0, vy=velocity, vz=0)  # Move right
    time.sleep(side_length / velocity)  # Sleep for the duration of the side
    set_position_target_local_ned(yaw=yaw, vx=-velocity, vy=0, vz=0)  # Move backward
    time.sleep(side_length / velocity)  # Sleep for the duration of the side
    set_position_target_local_ned(yaw=yaw, vx=0, vy=-velocity, vz=0)  # Move left
    time.sleep(side_length / velocity)  # Sleep for the duration of the side

def land_drone():
    print("Landing drone")
    master.mav.command_long_send(master.target_system,  
                                   master.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_LAND,
                                   0,
                                   0, 0, 0, 0, 0, 0, 0)
    
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED == False:
            print("Drone landed")
            break

# Main execution flow
try:
    set_mode("GUIDED")  # Change to "GUIDED" mode
    arm_drone()
    
    takeoff(10)         # Take off to an altitude of 10 meters
    fly_square(15, velocity=1, yaw=30)  # Fly in a square with each side of length 5 meters, velocity 3 m/s, and optional yaw
    land_drone()
finally:
    master.close()
