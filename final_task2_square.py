import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil


vehicle = connect('127.0.0.1:14550', wait_ready=True)



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
    for x in range(0,1):
        vehicle.send_mavlink(msg)
        time.sleep(1)


while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(5)

print ("Arming Motors")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
print ("motors Armed")
time.sleep(5)
print ("taking off")
vehicle.simple_takeoff(10)
time.sleep(3)

while vehicle.location.global_relative_frame.alt <= 9.5:
    time.sleep(1)
print ("will travel in square")
'''
send_ned_velocity(5,0,0,5)

while vehicle.groundspeed >= 0.1:
    time.sleep(1)
send_ned_velocity(0,5,0,5)
while vehicle.groundspeed >= 0.1:
    time.sleep(1)
send_ned_velocity(-5,0,0,5)
while vehicle.groundspeed >= 0.1:
    time.sleep(1)
send_ned_velocity(0,-5,0,5)
time.sleep(2)
'''
for x in range (0,4):
    send_ned_velocity(5*math.sin(math.radians(90*x)),5*math.cos(math.radians(90*x)),0,5)
    time.sleep(6)

vehicle.mode = VehicleMode("LAND")
time.sleep(4)
vehicle.close()
