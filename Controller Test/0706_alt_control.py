# 0706 Altitude Controller
# Use Python 2.7 or 3.7

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import threading
import time
import math
import numpy as np
import cv2

# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# Loop Frequency
FREQ_CMD = 5
FREQ_ALT = 10
FREQ_VEL = 10
FREQ_HDG = 10
FREQ_CAM = 20
FREQ_SENSOR = 20

# Auto mode and flag
STATE = True
MODE = ''

# Target Value
TARGET_PITCH = 0.0
TARGET_ROLL = 0.0
TARGET_YAW = 0.0
INCREMENT_YAW = 0.0
TARGET_ALT = 0.0
TARGET_VEL = 0.0
TARGET_POS = [0.0, 0.0]
TARGET_THRUST = 0.0

# Actual Value (Sensor Feedback)
ACTUAL_PITCH = 0.0
ACTUAL_ROLL = 0.0
ACTUAL_YAW = 0.0
ACTUAL_VEL = 0.0
ACTUAL_POS = [0.0, 0.0]
ACTUAL_ALT = 0.0

# Altitude Controller setting
ALT_KP = 1
ALT_KI = 0
ALT_KD = 0
ALT_Gain_Division = 1

# Velocity Controller setting
VEL_KP = 1
VEL_KI = 0
VEL_KD = 0
VEL_Gain_Division = 1
VEL_SENSOR_OFFSET_X = 0

# Position Controller setting
POS_KP = [1, 1]
POS_KI = [0, 0]
POS_KD = [0, 0]
POS_Gain_Division = [1, 1]

# Camera Controller
SERVO_ROLL_PIN = 20
SERVO_PITCH_PIN = 21
PWM_FREQ = 50
PWM_LOWER_LIMIT = 400
PWM_UPPER_LIMIT = 2350
DC_MID = 6.875

# Altitude Sensor setting
US_TRIG_PIN = 2
US_ECHO_PIN = 3
SAMPLE_NUMBER = 10
AIR_TEMP = 30
ALT_SENSOR_OFFSET_US = -0.065
ALT_SENSOR_OFFSET_BM = 0
ALT_SENSOR_OFFSET_LS = 0
WEIGHT_US = 1
WEIGHT_BM = 0
WEIGHT_LS = 0

# Record Data
R_NUM = []
R_ACTUAL_ALT = []
R_TARGET_THRUST = []
R_TARGET_ALT = []


# Connect to the UAV
print('Connecting to the UAV ,please wait.')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('UAV connected.')


# Function Def. Area 1 (Background Function)
# Altitude Controller (Check ok)
def altitude_controller():
    # Preset variable
    global TARGET_THRUST
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    print('Altitude Controller ON.')

    # Controller loop
    while STATE:
        # Calculate error
        error = TARGET_ALT - ACTUAL_ALT

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * ALT_KP
        gain_i = int_error * ALT_KI
        gain_d = derivative * ALT_KD
        gain_sum = gain_p + gain_i + gain_d

        # Map output into specific range (thrust range -0.5~+0.5)
        thrust_out = gain_sum / ALT_Gain_Division

        # Offset to midpoint
        thrust_out += 0.5

        # Output Limitation
        if thrust_out > 1:
            thrust_out = 1
        if thrust_out < 0:
            thrust_out = 0

        # Renew the thrust value
        TARGET_THRUST = thrust_out

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        # Record data
        R_ACTUAL_ALT.append(ACTUAL_ALT)
        R_TARGET_ALT.append(TARGET_ALT)
        R_TARGET_THRUST.append(TARGET_THRUST)

        time.sleep(1/FREQ_ALT)     # Send message at designated Hz

        # Close threading when STATE = False

    print('Altitude Controller Closed.')


# Altitude Sensor (Check ok)
def altitude_sensor():
    # Initializing
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(US_TRIG_PIN, GPIO.OUT)
    GPIO.setup(US_ECHO_PIN, GPIO.IN)
    GPIO.output(US_TRIG_PIN, False)
    height_us_register = []
    height_bm_register = []
    # height_ls_register = []
    pulse_end = time.time()
    pulse_start = time.time()

    print('Altitude Sensor ON.')

    # Sensor loop
    while STATE:
        # Trig the emitter
        GPIO.output(US_TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(US_TRIG_PIN, False)

        # Record a pulse time
        while GPIO.input(US_ECHO_PIN) == 0:
            pulse_start = time.time()
        while GPIO.input(US_ECHO_PIN) == 1:
            pulse_end = time.time()

        # Calculate the height and set the filter
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2 + ALT_SENSOR_OFFSET_US
        if height > 2:
            height = 2
        if height < 0:
            height = 0

        # Convert to actual Attitude
        roll_angel = uav.attitude.roll
        pitch_angel = uav.attitude.pitch
        alt_row_us = height * math.cos(roll_angel) * math.cos(pitch_angel)

        # Read barometer and laser data
        alt_row_bm = uav.location.global_relative_frame.alt + ALT_SENSOR_OFFSET_BM
        if alt_row_bm < 0:
            alt_row_bm = 0
        # alt_row_ls = LS.range / 1000 + ALT_SENSOR_OFFSET_LS

        # Noise reduction and update values
        # Ultrasonic Sensor (US)
        height_us_register.insert(0, alt_row_us)
        if len(height_us_register) > SAMPLE_NUMBER:
            height_us_register.pop()
        alt_us = np.average(height_us_register)

        # Barometer Sensor (BM)
        height_bm_register.insert(0, alt_row_bm)
        if len(height_bm_register) > SAMPLE_NUMBER:
            height_bm_register.pop()
        alt_bm = np.average(height_bm_register)

        # Laser Sensor (LS)
        # height_ls_register.insert(0, alt_row_ls)
        # if len(height_ls_register) > SAMPLE_NUMBER:
        #    height_ls_register.pop()
        # alt_ls = np.average(height_ls_register)
        alt_ls = 0

        # Combined the height by weighted average method
        alt_ave = (WEIGHT_US * alt_us + WEIGHT_BM * alt_bm +
                   WEIGHT_LS * alt_ls) / (WEIGHT_US + WEIGHT_BM + WEIGHT_LS)
        if alt_ave > 2:
            alt_ave = 2
        global ACTUAL_ALT
        ACTUAL_ALT = alt_ave

        time.sleep(1 / FREQ_SENSOR)  # Activate sensors at designated Hz (Exclude loop time 10ms)

        # Close threading when STATE = False

    print('Altitude Sensor Closed.')


# Command transfer (Check ok)
def command_transfer():
    while STATE:
        msg = uav.message_factory.set_attitude_target_encode(
            0,  # time boot
            0,  # target system
            0,  # target component
            0b00000000,  # type mask: bit 1 is LSB
            to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # quaternion
            0,  # body roll rate in radian
            0,  # body pitch rate in radian
            0,  # body yaw rate in radian
            TARGET_THRUST)  # thrust
        uav.send_mavlink(msg)

        time.sleep(1/FREQ_CMD)     # Send message at designated Hz

        # Close threading when STATE = False

    print('Command Transfer Closed.')


# Camera Recording
def camera():
    video = cv2.VideoCapture(0)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    output = cv2.VideoWriter('D://opencv_test//output.avi', fourcc, 30.0, (480, 360))
    while (video.isOpened()):
        ret, cap = video.read()
        if ret == True:
            cap_re = cv2.resize(cap, (480, 360))
            output.write(cap_re)
            cv2.imshow("test", cap_re)

            if not STATE:
                break
        else:
            break
    output.release()
    video.release()
    cv2.destroyAllWindows()
    print('Camera Closed.')


# Function Def. Area 2 (Useful Function)
# Convert euler Angel to quaternion
def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


# Write the value into global variable (For VHT mission mode)
def value_write(h, t):
    global TARGET_VEL, TARGET_ALT
    print('CMD H=%.2f m, Duration=%d sec' % (h, t))
    TARGET_ALT = h
    for i in range(0, t):
        time.sleep(1)
        print('Now H=%.2f m, Remaining=%d sec' % (ACTUAL_ALT, t-i-1))


# Landing Procedure
def landing():
    # Landing & Safety check
    while ACTUAL_ALT > 0.05:

        print('Wait for landing...')
        time.sleep(0.5)

    print("UAV landed.")


# Function Def. Area 3 (Main Function)
# Threading procedure
def startup():
    # Check RC switch is in Stabilize mode
    # while not uav.mode.name == 'STABILIZE':
        # print('Please push RC mode switch to Stabilize mode')
        # time.sleep(1)
    print("Now is in Mode: %s" % uav.mode.name)
    time.sleep(1)

    # Changing to Guided_NoGPS Mode
    print("Switching to Guided_NoGPS Mode.")
    uav.mode = VehicleMode("GUIDED_NOGPS")
    # while not uav.mode.name == 'GUIDED_NOGPS':
        # time.sleep(0.5)
    print("Now is in Mode: %s" % uav.mode.name)

    # Set UAV Armed
    print('Setting the UAV armed')
    uav.armed = True
    # while not uav.is_armable:
        # time.sleep(0.5)
        # print('Wait for armed...')

    print("Armed = %s" % uav.armed)
    print('UAV is ready to go!')

    # Def. threading.thread
    alt_sen = threading.Thread(target=altitude_sensor)
    alt = threading.Thread(target=altitude_controller)
    cmd = threading.Thread(target=command_transfer)
    cam = threading.Thread(target=camera)
    m = threading.Thread(target=mission)

    # Start the threading
    alt_sen.start()
    alt.start()
    cmd.start()
    cam.start()
    m.start()

    # Wait for threading close
    m.join()
    cmd.join()
    alt.join()
    alt_sen.join()
    cam.join()

    print('All threading closed.')


# Close the UAV
def shutdown():
    # Change to Stabilize mode
    print("Switching to Stabilize Mode")
    uav.mode = VehicleMode("STABILIZE")
    while not uav.mode.name == 'STABILIZE':
        print('Wait for changing to stabilize mode')
        time.sleep(1)
    print("Now is in Mode: %s" % uav.mode.name)

    # Wait of disarmed
    # while not uav.is_armable:
        # print('Wait for disarmed.')
        # time.sleep(1)
    print("Armed = %s" % uav.armed)

    # Close the connection
    print('Close the connect to UAV.')
    uav.close()
    print('Shutdown Completed')


# Mission function
def mission():
    global MODE, STATE
    MODE = 'LINE'     # Use line tracking mode to control ALT & VEL

    time.sleep(2)

    print('Mission start.')
    # CMD format : value_write(height, duration)
    value_write(1, 10)
    value_write(0.5, 3)
    value_write(1, 5)

    # Go landing
    landing()
    print('Mission complete.')

    # Close all threading
    STATE = False


def showdata():
    for i in range(1,len(R_TARGET_ALT)+1):
        R_NUM.append(i)

    plt.plot(R_NUM, R_TARGET_ALT, label="Target Height")
    plt.plot(R_NUM, R_ACTUAL_ALT, label="Actual Height")
    plt.plot(R_NUM, R_TARGET_THRUST, label="Thrust")

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


# Program Procedure
startup()
shutdown()
showdata()