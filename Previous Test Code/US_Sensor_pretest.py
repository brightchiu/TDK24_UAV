# US sensor test

# import modules
from __future__ import print_function
from dronekit import connect
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import time
import math
import collections


# Altitude Sensor setting
US_TRIG_PIN = 2
US_ECHO_PIN = 3
SAMPLE_NUMBER = 5
AIR_TEMP = 28
ALT_SENSOR_OFFSET_US = -0.026
WEIGHT_US = 1
WEIGHT_BM = 0
WEIGHT_LS = 0
WEIGHT_US_FLAG = 1
WEIGHT_LS_FLAG = 1
FREQ_SENSOR = 10

# Record Data
R_NUM = []
R_ACTUAL_ALT = []

# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# Connect to the UAV
print('Connecting to the UAV ,please wait.')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('UAV connected.')


# Altitude Sensor (Check ok)
def altitude_sensor():
    # Initializing
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(US_TRIG_PIN, GPIO.OUT)
    GPIO.setup(US_ECHO_PIN, GPIO.IN)
    GPIO.output(US_TRIG_PIN, False)
    height_us_register = []
    history = collections.deque(maxlen=10)

    print('Altitude Sensor ON.')

    # Sensor loop
    for i in range(0, 100):
        global WEIGHT_US_FLAG, WEIGHT_LS_FLAG
        # Read sensor row data
        # Ultrasonic and weight enable flag
        history.append(us_sensor())
        alt_row_us = np.median(history)

        # Noise reduction and update values
        # Ultrasonic Sensor (US)
        height_us_register.insert(0, alt_row_us)
        if len(height_us_register) > SAMPLE_NUMBER:
            height_us_register.pop()
        alt_us = np.average(height_us_register)

        R_ACTUAL_ALT.append(alt_us)

        time.sleep(1 / FREQ_SENSOR)  # Activate sensors at designated Hz (Exclude loop time 10ms)

        # Close threading when STATE = False

    print('Altitude Sensor Closed.')


# Ultra sonic sensor
def us_sensor():
    # Timer settings
    pulse_end = time.time()
    pulse_start = time.time()
    timeout = True

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
                break
            timeout = False

    # Calculate the height and set the filter
    pulse_duration = pulse_end - pulse_start
    height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2 + ALT_SENSOR_OFFSET_US

    # Convert to actual Attitude
    roll_angel = uav.attitude.roll
    pitch_angel = uav.attitude.pitch
    height = height * math.cos(roll_angel) * math.cos(pitch_angel)

    return height


# Show flight data
def show_data():
    for i in range(1, len(R_ACTUAL_ALT)+1):
        R_NUM.append(i)

    plt.plot(R_NUM, R_ACTUAL_ALT, label="Actual Height")

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


altitude_sensor()
show_data()
