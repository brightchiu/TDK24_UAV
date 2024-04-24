# Use Python 2.7 or 3.7

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect
import time
import math
import matplotlib.pyplot as plt

# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/tty.SLAB_USBtoUART'
CONNECT_BAUD = 57600
# CONNECT_BAUD = 921600

R_YAW = []
R_HDG = []
R_NUM = []

# Connect to the UAV
print('Connecting to the UAV ,please wait.')
uav = connect(CONNECT_PORT, baud=CONNECT_BAUD)
print('UAV connected.')

for x in range(0, 100):
    yaw = uav.attitude.yaw
    roll = uav.attitude.roll
    pitch = uav.attitude.pitch
    hdg = uav.heading
    alt = uav.location.global_relative_frame.alt
    alt_g = uav.location.global_frame.alt
    R_YAW.append(yaw)
    R_HDG.append(hdg)
    R_NUM.append(x)
    print('\nroll=', math.degrees(roll))
    print('pitch=', math.degrees(pitch))
    print('yaw=', math.degrees(yaw))
    print(alt)
    print(alt_g)
    time.sleep(1)

plt.plot(R_NUM, R_HDG, label="Heading(degree)")
plt.plot(R_NUM, R_YAW, label="Yaw(rad)")

# naming the x axis
plt.ylabel('Value')
# naming the y axis
plt.xlabel('Data number')
# giving a title to my graph
plt.title('Output Data Diagram')
# show a legend on the plot
plt.legend()
# function to show the plot
plt.show()
