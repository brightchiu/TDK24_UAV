# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect
import time
import numpy as np
import math
import matplotlib.pyplot as plt

# Initializing
# Setting connect parameter
print('Connecting to the UAV ,please wait.')
uav = connect('/dev/serial0', wait_ready=True, baud=921600)
print('UAV connected.')

ACTUAL_VEL = 0.0
vx_coll = []
vy_coll = []
vz_coll = []
v_coll = []

r_vx = []
r_vy = []
r_vz = []
r_v = []
r_num = []
# Velocity Sensor


# Preset variable
previous_time = time.time()
vx = 0
vy = 0

print('Velocity Sensor ON.')

# Sensor loop
for i in range(0, 200):
    # Receive acceleration
    ax = uav.velocity[0]
    ay = uav.velocity[1]

    # Calculate delta time
    current_time = time.time()
    delta_time = current_time - previous_time

    # Calculate velocity on x and y direction
    vx += ax * delta_time
    vy += ay * delta_time

    # Calculate total velocity
    v = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))

    # Output the value to global variable
    v_coll.insert(0, v)
    if len(v_coll) > 5:
        v_coll.pop()
    v_ave = np.average(v_coll)
    ACTUAL_VEL = v_ave
    r_v.append(v_ave)

    # Pass values for next loop
    previous_time = current_time

'''
for i in range(0, 200):
    vx = uav.velocity[0]
    vy = uav.velocity[1]
    vz = uav.velocity[2]
    v = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))
    yaw = uav.attitude.yaw
    hdg = uav.heading
    vx_coll.insert(0, vx)
    if len(vx_coll) > 5:
        vx_coll.pop()
    vx_ave = np.average(vx_coll)

    vy_coll.insert(0, vy)
    if len(vy_coll) > 5:
        vy_coll.pop()
    vy_ave = np.average(vy_coll)

    vz_coll.insert(0, vz)
    if len(vz_coll) > 5:
        vz_coll.pop()
    vz_ave = np.average(vz_coll)

    v_coll.insert(0, v)
    if len(v_coll) > 5:
        v_coll.pop()
    v_ave = np.average(v_coll)

    r_vx.append(vx_ave)
    r_vy.append(vy_ave)
    r_vz.append(vz_ave)
    r_v.append(v_ave)
    r_num.append(i+1)
    time.sleep(0.1)
    '''

# plt.plot(r_num, r_vx, label="Vx")
# plt.plot(r_num, r_vy, label="Vy")
# plt.plot(r_num, r_vz, label="vz")
plt.plot(r_num, r_v, label="v")

# naming the x axis
plt.ylabel('Velocity (m/s)')
# naming the y axis
plt.xlabel('Data number')
# giving a title to my graph
plt.title('Velocity Sensor Diagram')
# show a legend on the plot
plt.legend()
# function to show the plot
plt.show()
