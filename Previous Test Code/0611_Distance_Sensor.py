# 0611 Distance Sensor use HC-SR04

# !/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

# GPIO PIN SETTING

TRIG = 2
ECHO = 3

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)

print("Waiting For Sensor To Settle")

time.sleep(2)

for i in range(1,20,1):

    GPIO.output(TRIG, True)

    time.sleep(0.00001)

    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    print("Distance:", distance, "cm")
    time.sleep(1)

GPIO.cleanup()