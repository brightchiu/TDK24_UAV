# 0707 Velocity controller

# Use Python 2.7 or 3.7

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import threading
import time
import math
import numpy as np
import cv2

# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# Loop Frequency
FREQ_CMD = 5
FREQ_ALT = 10
FREQ_ACC = 10
FREQ_VEL = 10

# Target value
TARGET_ACC = 0.0
TARGET_PITCH = 0.0

# Current value
ACTUAL_VEL = 0.0

# Velocity Controller setting
BETA = 1
M = 1
SAMPLE_NUMBER = 5


# Acceleration Controller setting
ACC_KP = 1
ACC_KI = 0
ACC_KD = 0
ACC_Gain_Division = 1

# Auto mode and flag
STATE = True
MODE = ''

# Connect to the UAV
print('Connecting to the UAV ,please wait.')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('UAV connected.')


# Acceleration Generate
def vel_to_acc(v_cmd):
    global ACTUAL_VEL, TARGET_ACC
    # Calculate error
    delta_vel = v_cmd - ACTUAL_VEL

    # Calculate time point
    t1 = math.sqrt(math.fabs(delta_vel) / (M * (1 + BETA)))
    t2 = t1 + t1 * BETA
    t3 = 2 * t1 + t1 * BETA

    # Set timer
    t_start = time.time()
    t_now = time.time()

    # Generate loop
    while t_now-t_start <= t3:
        # Calculate t-term
        t = t_now-t_start

        # Calculate acceleration
        if t <= t1:
            a = M * t
            print('Rising.')
        elif t <= t2:
            a = M * t1
            print('Middle')
        else:
            a = M * (t1*(2+BETA)-t)
            print('Decreasing.')

        # Positive or negative output
        if delta_vel < 0:
            a *= -1

        # Renew the global variable
        TARGET_ACC = a

        # Wait and past timer value
        time.sleep(1/FREQ_VEL)
        t_now = time.time()

    # Renew the velocity and acceleration
    ACTUAL_VEL = v_cmd
    TARGET_ACC = 0


# Acceleration Controller
def acceleration_controller():
    # Preset variable
    global TARGET_PITCH
    int_error = 0
    previous_error = 0
    previous_time = time.time()
    a_coll = []

    print('Acceleration Controller ON.')

    # Controller loop
    while STATE:
        # Get sensor data
        ax = uav.velocity[0]
        ay = uav.velocity[1]

        # Calculate acceleration
        a = math.sqrt(math.pow(ax, 2) + math.pow(ay, 2))

        # Judge acceleration direction
        if a == 0:
            a = 0
        elif (math.cos(uav.attitude.yaw) * (ax/a)) > 0:
            a *= 1
        elif (math.cos(uav.attitude.yaw) * (ax/a)) < 0:
            a *= -1
        elif ax/a == 0 or math.cos(uav.attitude.yaw) == 0:
            if (math.sin(uav.attitude.yaw) * (ay / a)) > 0:
                a *= 1
            elif (math.sin(uav.attitude.yaw) * (ay / a)) < 0:
                a *= -1

        # Noise reduction and update values
        a_coll.insert(0, a)
        if len(a_coll) > SAMPLE_NUMBER:
            a_coll.pop()
        a_ave = np.average(a_coll)

        # Calculate error
        error = TARGET_ACC - a_ave

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * ACC_KP
        gain_i = int_error * ACC_KI
        gain_d = derivative * ACC_KD
        gain_sum = gain_p + gain_i + gain_d

        # Map output into specific range (thrust range -0.5~+0.5)
        pitch_out = gain_sum / ACC_Gain_Division

        # Change the direction
        pitch_out *= -1

        # Renew the thrust value
        TARGET_PITCH = pitch_out

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        # Record data

        time.sleep(1/FREQ_ACC)     # Send message at designated Hz

        # Close threading when STATE = False

    print('Altitude Controller Closed.')
