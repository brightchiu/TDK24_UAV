#!/usr/bin/python

import time
import board
import busio
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)
LS = adafruit_vl53l0x.VL53L0X(i2c)

for i in range(0,100):
    height_ls = LS.range / 1000
    print('d= ',height_ls,' mm')
    time.sleep(1)