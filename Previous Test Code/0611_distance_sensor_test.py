# 0611 Distance Sensor use HC-SR04
# Single value version, with time spend.

# !/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time

# Parameter
TRIG = 2
ECHO = 3
TEMP = 29
x = []
y = []
t = []

# Initialize
print("Initializing...")
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)
time.sleep(2)
print("Initialize complete.")

# Sensor loop
for i in range(1, 500, 1):
    work_start = time.time()

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * (331.6 + 0.6 * TEMP)) / 2
    y.append(distance)

    work_end = time.time()
    time_spend = work_end - work_start
    t.append(time_spend)

    print("Distance:", distance, "m")
    print("Time spent:", time_spend, "sec")
    time.sleep(0.01)

for i in range(1, len(y)+1, 1):
    x.append(i)

# Show data's trend
def show_data():
    plt.plot(x, y, label="Distance")
    plt.plot(x, y, label="Time Spend")
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
