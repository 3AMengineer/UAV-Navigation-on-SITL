from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.2f}")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0)
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

arm_and_takeoff(10)

print("Moving forward at 2 m/s for 5 seconds")
send_ned_velocity(2, 0, 0, 5)

print("Moving right at 2 m/s for 5 seconds")
send_ned_velocity(0, 2, 0, 5)

print("Stopping...")
send_ned_velocity(0, 0, 0, 1)

print("Landing")
vehicle.mode = VehicleMode("LAND")

vehicle.close()

