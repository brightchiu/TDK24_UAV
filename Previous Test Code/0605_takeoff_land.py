# 0604 Takeoff and landing test

# !/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# Setting connect parameter
print('Connecting the UAV ,please wait.')
uav = connect('/dev/serial0', wait_ready=True, baud=921600)
print('UAV connected')

# Parameter Setting
DEFAULT_TAKEOFF_THRUST = 0.7
SMOOTH_TAKEOFF_THRUST = 0.65
HOVER_THRUST = uav.parameters.get['MOT_THST_HOVER']

def takeoff(T_altitude):

    print("Switch to Guided_NoGPS Mode")
    uav.mode = VehicleMode("GUIDED_NOGPS")
    print("Now is in Mode: %s" % uav.mode.name)

    print('Setting the UAV armed')
    uav.armed = True
    print("Armed = %s" % uav.armed)

    print("Taking off!")

    # Takeoff program
    while True:


        current_altitude = uav.location.global_relative_frame.alt
        print("Altitude: %s m", current_altitude)

        if current_altitude >= T_altitude * 0.95:
            print("Reached target altitude")
            break
        elif current_altitude >= T_altitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST

        set_attitude(thrust=thrust)
        time.sleep(0.2)

def landing():
    # Landing
    print("Setting LAND mode...")
    uav.mode = VehicleMode("LAND")
    print("Now is in Mode: %s" % uav.mode.name)

    # Buffer area
    time.sleep(2)
    current_altitude = uav.location.global_relative_frame.alt

    # Landed check
    while True:
        print("Altitude: %s m", current_altitude)
        time.sleep(0.5)

        if current_altitude <= 0.5:
            time.sleep(2)
            print("UAV landed.")
            break

def shutdown():
    print('Set the UAV disarmed.')
    uav.armed = False
    print("Armed = %s" % uav.armed)

    time.sleep(2)
    print('Close the UAV.')
    uav.close()
    print('Shutdown Completed')


def set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0,
                 roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0,
                 thrust=HOVER_THRUST, duration=0):
    msg = uav.message_factory.set_attitude_target_encode(
        0,  # time boot
        0,  # target system
        0,  # target component
        0b00000000,  # type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # q
        roll_rate,  # body roll rate in radian (not supported)
        pitch_rate,  # body pitch rate in radian (not supported)
        yaw_rate,  # body yaw rate in radian (not supported)
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

    print('q = ', w, x, y, z)

    return [w, x, y, z]


'''Flight Command Area'''

# Command Format
# set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0,
             # roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0,
             # thrust=0.5, duration=0)

takeoff(1.5)
set_attitude(duration=10)
landing()
shutdown()