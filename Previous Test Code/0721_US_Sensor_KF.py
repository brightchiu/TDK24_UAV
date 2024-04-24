# 0721 Ultrasonic sensor with Kalman Filter
# Timeout prevent & Double filter & Auto calibrate

# Use Python 2.7 or 3.7

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math

# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# Altitude Sensor setting
US_TRIG_PIN = 2
US_ECHO_PIN = 3
SAMPLE_NUMBER = 5
AIR_TEMP = 28
ALT_SENSOR_OFFSET_US = 0
WEIGHT_US = 1
WEIGHT_BM = 0
WEIGHT_LS = 0
WEIGHT_US_FLAG = 1
WEIGHT_LS_FLAG = 1
FREQ_SENSOR = 20
ACTUAL_ALT_US = 0.0
ACTUAL_ALT_US_ROW = 0.0
ACTUAL_ALT_US_ROW0 = 0.0

STATE = True

# Record Data
R_NUM = []
R_ACTUAL_ALT = []
R_ACTUAL_ALT_US = []
R_ACTUAL_ALT_US_ROW = []
R_ACTUAL_ALT_US_ROW0 = []

# Connect to the UAV
print('Connecting to the UAV ,please wait.')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('UAV connected.')


# Altitude Sensor (Check ok)
def altitude_sensor():
    # Initializing
    height_us_register = []
    print('Altitude Sensor ON.')

    # Sensor loop
    for i in range(0, 100):
        global WEIGHT_US_FLAG, WEIGHT_LS_FLAG
        # Read sensor row data
        # Ultrasonic and weight enable flag
        alt_row_us = ACTUAL_ALT_US

        # Noise reduction and update values
        # Ultrasonic Sensor (US)
        height_us_register.insert(0, alt_row_us)
        if len(height_us_register) > SAMPLE_NUMBER:
            height_us_register.pop()
        alt_us = np.average(height_us_register)

        R_ACTUAL_ALT.append(alt_us)
        R_ACTUAL_ALT_US_ROW.append(ACTUAL_ALT_US_ROW)
        R_ACTUAL_ALT_US_ROW0.append(ACTUAL_ALT_US_ROW0)
        R_ACTUAL_ALT_US.append(ACTUAL_ALT_US)

        time.sleep(1 / FREQ_SENSOR)  # Activate sensors at designated Hz (Exclude loop time 10ms)

        # Close threading when STATE = False

    print('Altitude Sensor Closed.')

    global STATE
    STATE = False


# Ultra sonic sensor
def us_sensor():
    global ACTUAL_ALT_US, ACTUAL_ALT_US_ROW, ACTUAL_ALT_US_ROW0
    # Initializing
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(US_TRIG_PIN, GPIO.OUT)
    GPIO.setup(US_ECHO_PIN, GPIO.IN)
    GPIO.output(US_TRIG_PIN, False)

    # Timer settings for sensor
    pulse_end = time.time()
    pulse_start = time.time()

    # Initialize for Kalman Filter
    x_o = 0                       # Previous state
    p_o = 0                       # Previous covariance
    q = math.exp(-5)              # Covariance noise
    r = 2.92 * math.exp(-3)       # Covariance measurement noise

    print('Ultrasonic Sensor ON.')

    # Sensor Calibrate
    us_calibrate()
    print('Ultrasonic Sensor Calibrated.')

    while STATE:
        # Timeout flag
        timeout = True

        # Kalman Filter Predict
        x_p = x_o
        p_p = p_o + q

        # Sensor loop (Timeout prevention)
        while timeout:
            # Trig the emitter
            GPIO.output(US_TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(US_TRIG_PIN, False)
            timer_out = time.time()

            # Record a pulse time
            while not GPIO.input(US_ECHO_PIN):
                pulse_start = time.time()
                if pulse_start - timer_out > 0.1:
                    break
            while GPIO.input(US_ECHO_PIN):
                pulse_end = time.time()
                if pulse_start - pulse_end > 0.1:
                    timeout = True
                    break
                timeout = False

        # Calculate the height and set the filter
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2 + ALT_SENSOR_OFFSET_US
        ACTUAL_ALT_US_ROW0 = height

        # Convert to actual Attitude
        roll_angel = uav.attitude.roll
        pitch_angel = uav.attitude.pitch
        z = height * math.cos(roll_angel) * math.cos(pitch_angel)
        ACTUAL_ALT_US_ROW = z

        # Kalman Filter Innovation
        k = p_p / (p_p + r)         # Kalman Gain
        x = x_p + k * (z - x_p)     # State
        p = (1 - k) * p_p           # Covariance

        # Renew global variable
        ACTUAL_ALT_US = x

        # Pass value
        x_o = x
        p_o = p

        time.sleep(1 / FREQ_SENSOR)  # Activate sensors at designated Hz (Exclude loop time 10ms)

        # Close threading when STATE = False

    print('Ultrasonic Sensor Closed.')


# Ultrasonic Sensor Calibrate
def us_calibrate():
    global ALT_SENSOR_OFFSET_US
    height_ave = []
    # Timeout flag
    timeout = True

    # Timer settings for sensor
    pulse_end = time.time()
    pulse_start = time.time()

    for i in range(0, 10):
        # Sensor loop (Timeout prevention)
        while timeout:
            # Trig the emitter
            GPIO.output(US_TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(US_TRIG_PIN, False)
            timer_out = time.time()

            # Record a pulse time
            while not GPIO.input(US_ECHO_PIN):
                pulse_start = time.time()
                if pulse_start - timer_out > 0.1:
                    break
            while GPIO.input(US_ECHO_PIN):
                pulse_end = time.time()
                if pulse_start - pulse_end > 0.1:
                    timeout = True
                    break
                timeout = False

        # Calculate the height and
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2
        height_ave.append(height)

    ALT_SENSOR_OFFSET_US = -np.average(height_ave)
    print('Auto calibrate: ', ALT_SENSOR_OFFSET_US, ' m')


# Show flight data
def show_data():
    for i in range(1, len(R_ACTUAL_ALT)+1):
        R_NUM.append(i)

    plt.plot(R_NUM, R_ACTUAL_ALT, label="Actual Height (Average Filter)")
    plt.plot(R_NUM, R_ACTUAL_ALT_US, label="Sensor Height (After KF)")
    plt.plot(R_NUM, R_ACTUAL_ALT_US_ROW, label="Sensor Height (Compensate)")
    plt.plot(R_NUM, R_ACTUAL_ALT_US_ROW0, label="Sensor Height (No compensate)")

    # naming the x axis
    plt.ylabel('Height(m)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Ultrasonic Sensor Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()


def main():
    alt_sen = threading.Thread(target=altitude_sensor)
    alt_sen_us = threading.Thread(target=us_sensor)

    alt_sen_us.start()
    alt_sen.start()

    alt_sen.join()
    alt_sen_us.join()

    show_data()


main()
