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

print("Ready!")

for i in range(1,500,1):
    #print("sent pulse")
    ttt_start = time.time()

    GPIO.output(TRIG, True)

    time.sleep(0.00001)

    GPIO.output(TRIG, False)
    
    while GPIO.input(ECHO)==0:
        if GPIO.input(ECHO)==1:
            break
    pulse_start = time.time()
        
    while GPIO.input(ECHO)==1:
        if GPIO.input(ECHO)==0:
            break
        
    pulse_end = time.time()
            
    #print("recieved pulse")

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    ttt_end = time.time()
    print("Distance:", distance, "cm")
    
    print("cost", ttt_end - ttt_start,"s")
    time.sleep(0.1)

print("close sensor")
GPIO.cleanup()