# Gimbal test

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect
import RPi.GPIO as GPIO
import time
import math


# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# Camera Controller
SV_ROLL_PIN = 23
SV_PITCH_PIN = 24
SV_PWM_FREQ = 50
SV_PWM_LIMIT_DN = 400
SV_PWM_LIMIT_UP = 2350
SV_DC_MID = 6.875
FREQ_CAM = 20

# Connect to the UAV
print('Connecting to the UAV ,please wait.')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('UAV connected.')


# Camera Controller (Need to check rotate direction)
def camera_controller():
    # Set the pin mode
    GPIO.setmode(GPIO.BCM)
    output_list = [SV_ROLL_PIN, SV_PITCH_PIN]
    GPIO.setup(output_list, GPIO.OUT)

    # Creat PWM control item
    servo_r = GPIO.PWM(SV_ROLL_PIN, SV_PWM_FREQ)
    servo_p = GPIO.PWM(SV_PITCH_PIN, SV_PWM_FREQ)

    # Start at servo's middle point
    servo_r.start(SV_DC_MID)
    servo_p.start(SV_DC_MID)

    print('Camera Servo Controller ON.')

    while 1:
        # Calculate UAV degree to servo output degree
        roll_out = - math.degrees(uav.attitude.roll)
        pitch_out = - math.degrees(uav.attitude.pitch)

        # Change degree to duty cycle
        roll_dc = degree_to_dc(roll_out)
        pitch_dc = degree_to_dc(pitch_out)

        # Output the DC%
        servo_p.ChangeDutyCycle(roll_dc)
        servo_r.ChangeDutyCycle(pitch_dc)

        time.sleep(1 / FREQ_CAM)  # Send message at designated Hz

        # Close threading when STATE = False


# Change degree to duty cycle (For servo control)
def degree_to_dc(degree_cmd):
    degree_abs = degree_cmd + 90
    pwm_cmd = SV_PWM_LIMIT_DN + degree_abs * ((SV_PWM_LIMIT_UP - SV_PWM_LIMIT_DN) / 180)
    dc = SV_PWM_FREQ * (pwm_cmd / 1000000) * 100
    return dc
