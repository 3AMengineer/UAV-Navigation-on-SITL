from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil
import time
import math
import matplotlib.pyplot as plt

vehicle = connect('127.0.0.1:14550', wait_ready=True)



def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (math.sin(dlat/2)**2 +
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c   

def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360 






def send_ned_velocity(velocity_x, velocity_y, velocity_z):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    
    vehicle.send_mavlink(msg)



def takeoff():
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(5)

    print ("Arming Motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    print ("motors Armed")
    time.sleep(1)
    vehicle.simple_takeoff(10)
    print ("taking off")

    while vehicle.location.global_relative_frame.alt <= 9:
        time.sleep(1)
        continue

    print("reached target altitude")
    
    time.sleep(2)
    while vehicle.airspeed >= 0.1 : 
        time.sleep(1)

takeoff()



time_steps = []
error_values = []
#distance_values = []
#output_values = []
#desiredhvrs_values = []


lat1, lon1 = -37.598705,143.881744 # Home Location
lat2, lon2 = -37.60131931,143.88228917 # target Location

dt = 0.001
prvs_error = 0
integral = 0 

#error = haversine(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,-37.60131931,143.88228917)    


#TUNING
kp = 0.05
ki = 0.000002
kd = 0.000003




while True:
     
    bearing_deg = calculate_bearing(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon, lat2, lon2)
    error = haversine(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,-37.60131931,143.88228917)    

    integral += error*dt
    derivative = (error-prvs_error)/dt
    prvs_error = error


    output = kp*error + ki*integral + kd*derivative

    bearing_rad = math.radians(bearing_deg)


    '''error_values.append(prvs_error)
    distance_values.append(distance2)  
    output_values.append(output)'''


    send_ned_velocity(output*math.cos(bearing_rad),output*math.sin(bearing_rad),0)

    print(f"Distance: {error:.3f}")
    print(f"Speed: {output:.3f}")

    # Stop when within 0.2 meter
    if error < 0.2 and output < 0.2:

        send_ned_velocity(0, 0, 0)
        print("Arrived at target.")
        print (f"kp={kp},kd={kd},ki={ki}")
        break




    time.sleep(dt)
    














