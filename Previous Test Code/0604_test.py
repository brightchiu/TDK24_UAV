# 0604 RPI to FC connect test

# !/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# setting connect parameter
print('Connecting the UAV ,please wait.')
uav = connect('/dev/serial0', wait_ready=True, baud=921600)
print('UAV is connected')

def takeoff(T_altitude):

    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.65

    print("Basic pre-arm checks")
    #while not uav.is_armable:
        #print(" Waiting for vehicle to initialise...")
        #time.sleep(1)

    print("Arming motors")
    uav.mode = VehicleMode("GUIDED_NOGPS")
    uav.armed = True

    while not uav.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST

    take_off = time.time()

    while True:
        current_altitude = uav.location.global_relative_frame.alt
        print(" Altitude: ", current_altitude)
        if (time.time()-take_off) > 10:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= T_altitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)
        time.sleep(0.2)

def set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0, thrust=0.5, duration=0):

    msg = uav.message_factory.set_attitude_target_encode(
        0,  # time boot
        0,  # target system
        0,  # target component
        0b00000000,  # type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # q
        0,  # body roll rate in radian (not supported)
        0,  # body pitch rate in radian (not supported)
        0,  # body yaw rate in radian (not supported)
        thrust)  # thrust
    uav.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the frational and integer parts

        elapsed_time = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(elapsed_time[0])

        # Send command to vehicle on 1 Hz cycle
        for x in range(0, int(elapsed_time[1])):
            time.sleep(1)
            uav.send_mavlink(msg)

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

'''Fligt CMD'''

takeoff(1.5)
set_attitude(duration=3)

print('set_attitude(pitch_angle=5, thrust=0.5, duration=3)')
set_attitude(pitch_angle=5, thrust=0.5, duration=3)
set_attitude(duration=3)
print('set_attitude(pitch_angle=-5, thrust=0.5, duration=3)')
set_attitude(pitch_angle=-5, thrust=0.5, duration=3)
set_attitude(duration=3)
print('set_attitude(roll_angle=5, thrust=0.5, duration=3)')
set_attitude(roll_angle=5, thrust=0.5, duration=3)
set_attitude(duration=3)
print('set_attitude(roll_angle=-5, thrust=0.5, duration=3)')
set_attitude(roll_angle=-5, thrust=0.5, duration=3)
set_attitude(duration=3)
print('set_attitude(yaw_angle=90, thrust=0.5, duration=3)')
set_attitude(yaw_angle=90, thrust=0.5, duration=3)
set_attitude(duration=3)
print('set_attitude(yaw_angle=-90, thrust=0.5, duration=3)')
set_attitude(yaw_angle=-90, thrust=0.5, duration=3)

# Landing
print("Setting LAND mode...")
uav.mode = VehicleMode("LAND")
time.sleep(1)

# Landed check
while True:
    if uav.location.global_relative_frame.alt <= 0.5:
        print("Close vehicle object")
        break

uav.close()

print("Completed")