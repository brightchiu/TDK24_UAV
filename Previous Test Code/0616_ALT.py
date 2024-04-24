# 0611 Distance Sensor use HC-SR04
# Single value version, with time spend.

# !/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time
import math

# Parameter
TRIG = 2
ECHO = 3
TEMP = 29
x = []
y = []
y1 = []
t = []

# Initialize
print("Initializing...")
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)
time.sleep(2)

# Setting connect parameter
print('Connecting to the UAV ,please wait.')
uav = connect('/dev/serial0', wait_ready=True, baud=921600)
print('UAV connected.')
print("Initialize complete.")

# Sensor loop
for i in range(1, 1000, 1):
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * (331.6 + 0.6 * TEMP)) / 2

    #Convert to actual Attitude
    roll_angel = uav.attitude.roll
    pitch_angel = uav.attitude.pitch
    distance = distance * (math.cos(math.radians(roll_angel)) * math.cos(math.radians(pitch_angel)))
    y.append(distance)

    current_altitude = uav.location.global_relative_frame.alt
    y1.append(current_altitude)

    print("Distance_Baro:", current_altitude, "m")
    print("Distance_UC:", distance, "m")
    time.sleep(0.1)

for i in range(1, len(y)+1, 1):
    x.append(i)

# Show data's trend
def show_data():
    plt.plot(x, y, label="Distance-UC")
    plt.plot(x, y1, label="Distance-Baro")
    # naming the x axis
    plt.ylabel('Distance-time - axis')
    # naming the y axis
    plt.xlabel('num - axis')
    # giving a title to my graph
    plt.title('Distance Sensor Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()

print("close sensor")
GPIO.cleanup()
