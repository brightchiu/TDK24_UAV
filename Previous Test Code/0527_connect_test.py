# 0527 RPI to FC connect test

# !/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

from dronekit import connect, Channels
from pymavlink import mavutil
import time

# setting connect parameter
uav = connect('/dev/serial0', wait_ready=True, baud=921600)
print('UAV connected')

t_s = time.time()

# show info
while 1:
    print('Heading = %s' % uav.heading)
    print("Velocity = %s" % uav.velocity)
    print("Battery = %s" % uav.battery)
    print("Mode = %s" % uav.mode.name)
    print("Armed = %s" % uav.armed)
    print("Altitude = " % uav.location.global_relative_frame.alt)
    time.sleep(3)
    if time.time()-t_s > 60:
        break



