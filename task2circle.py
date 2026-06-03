import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil


vehicle = connect('127.0.0.1:14550', wait_ready=True)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,5):
        vehicle.send_mavlink(msg)
        time.sleep(1)


while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(5)

print ("Arming Motors...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
print ("motors Armed!!")
time.sleep(2)
print ("taking off")
vehicle.simple_takeoff(10)
time.sleep(3)

while vehicle.location.global_relative_frame.alt < 9.5:
    time.sleep(1)
    
condition_yaw(1, relative=False)
print ("will travel in Circle")

#More like a polygon
for x in range (0,3600):

    send_ned_velocity(math.sin(math.degrees(0.1*x)),math.cos(math.degrees(0.1*x)),0,1)
    

'''sometimes circle is too small, too slow'''




