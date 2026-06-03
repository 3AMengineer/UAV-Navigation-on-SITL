import time
from dronekit import connect, VehicleMode, LocationGlobalRelative



# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# vehicle.location.global_relative_frame.alt = 0 (DOES NOT WORK BCOZ IT IS READ ONLY) HOW TO RESET ALTITUDE



#Pre arms check - why? --- 


'''
M-1
if vehicle.is_armable == "True":
    print("prearms check succesful")
    
        
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle.simple_takeoff(30)

else:
    print ("prearms check failed")'''



'''
M-2
while vehicle.is_armable:
    print (" waiting for arming")
    time.sleep(1)
    
print (" waiting for initialization...")'''



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

while vehicle.location.global_relative_frame.alt <= 9:
    time.sleep(1)
    continue

print("reached target altitude")

point1 = LocationGlobalRelative(-37.60131931,143.88228917 ,30)
vehicle.simple_goto(point1)
time.sleep(2)
while vehicle.airspeed >= 0.1 : 
    time.sleep(1)
print ("returning to launch")
vehicle.mode = VehicleMode("RTL")





