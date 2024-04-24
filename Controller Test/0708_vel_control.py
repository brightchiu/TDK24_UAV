# 0708 Velocity & Altitude Controller
# Separate the ultrasonic function
# Use Python 2.7 or 3.7

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math
import cv2

# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# Loop Frequency
FREQ_CMD = 20
FREQ_ALT = 10
FREQ_VEL = 10
FREQ_ACC = 10
FREQ_SENSOR = 10

# Auto mode and flag
STATE = True
MODE = ''

# Target Value
TARGET_PITCH = 0.0
TARGET_ROLL = 0.0
TARGET_YAW = 0.0
TARGET_ALT = 0.0
TARGET_VEL = 0.0
TARGET_ACC = 0.0
TARGET_THRUST = 0.0

# Actual Value (Sensor Feedback)
ACTUAL_PITCH = 0.0
ACTUAL_ROLL = 0.0
ACTUAL_YAW = 0.0
ACTUAL_ALT = 0.0
ACTUAL_VEL = 0.0
CURRENT_VEL = 0.0

# Altitude Controller setting
ALT_KP = 1
ALT_KI = 0
ALT_KD = 0
ALT_Gain_Division = 1

# Vel to Acc parameter
M = 1             # Acceleration slope
BETA = 1          # Coefficient of rising time and equal acceleration time

# Acceleration Controller setting
ACC_KP = 1
ACC_KI = 0
ACC_KD = 0
ACC_Gain_Division = 1

# Altitude Sensor setting
US_TRIG_PIN = 2
US_ECHO_PIN = 3
SAMPLE_NUMBER = 5
AIR_TEMP = 30
ALT_SENSOR_OFFSET_US = -0.065
ALT_SENSOR_OFFSET_BM = -0.07
ALT_SENSOR_OFFSET_LS = 0
WEIGHT_US = 1
WEIGHT_BM = 0
WEIGHT_LS = 0
WEIGHT_US_FLAG = 1
WEIGHT_LS_FLAG = 1

# Record Data
# For altitude controller
R_NUM_ALT = []
R_ACTUAL_ALT = []
R_TARGET_THRUST = []
R_TARGET_ALT = []
# For velocity controller
R_NUM_VEL = []
R_CURRENT_VEL = []
R_TARGET_ACC_VEL = []
R_TARGET_VEL = []
# For acceleration controller
R_NUM_ACC = []
R_ACTUAL_ACC = []
R_ACTUAL_ACC_X = []
R_ACTUAL_ACC_Y = []
R_ACTUAL_ACC_YAW = []
R_TARGET_PITCH = []
R_TARGET_ACC = []


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


# Velocity Controller
def velocity_controller():
    # Preset variable
    global ACTUAL_VEL, CURRENT_VEL, TARGET_ACC

    print('Velocity Controller ON.')

    # Controller loop
    while STATE:
        if not TARGET_VEL == ACTUAL_VEL:
            # Calculate error
            delta_vel = TARGET_VEL - ACTUAL_VEL

            # Calculate time point
            t1 = math.sqrt(math.fabs(delta_vel) / (M * (1 + BETA)))
            t2 = t1 + t1 * BETA
            t3 = 2 * t1 + t1 * BETA

            # Set timer
            t_start = time.time()
            t_now = time.time()

            # Generate loop
            while t_now - t_start <= t3:
                # Calculate t-term
                t = t_now - t_start

                # Calculate acceleration
                if t <= t1:        # Acceleration increasing
                    a = M * t
                elif t <= t2:      # Equal Acceleration
                    a = M * t1
                else:              # Acceleration decreasing
                    a = M * (t1 * (2 + BETA) - t)

                # Positive or negative acceleration output
                if delta_vel < 0:
                    a *= -1

                # Renew the global variable
                TARGET_ACC = a

                # Wait and past timer value
                time.sleep(1 / FREQ_VEL)
                t_now = time.time()

                # Calculate current velocity for display
                CURRENT_VEL += a * ((t_now - t_start) - t)

                # Record data
                R_CURRENT_VEL.append(CURRENT_VEL)
                R_TARGET_VEL.append(TARGET_VEL)
                R_TARGET_ACC_VEL.append(TARGET_ACC)

            # Renew the velocity and acceleration
            ACTUAL_VEL = TARGET_VEL
            CURRENT_VEL = TARGET_VEL
            TARGET_ACC = 0

        # Wait time to record
        time.sleep(1 / FREQ_VEL)

        # Record data
        R_CURRENT_VEL.append(CURRENT_VEL)
        R_TARGET_VEL.append(TARGET_VEL)
        R_TARGET_ACC_VEL.append(TARGET_ACC)

        # Close threading when STATE = False

    print('Acceleration Controller Closed.')


# Acceleration Controller
def acceleration_controller():
    # Preset variable
    global TARGET_PITCH
    int_error = 0
    previous_error = 0
    previous_time = time.time()
    a_coll = []

    print('Acceleration Controller ON.')

    # Controller loop
    while STATE:
        # Get sensor data
        ax = uav.velocity[0]
        ay = uav.velocity[1]

        # Calculate acceleration
        a = math.sqrt(math.pow(ax, 2) + math.pow(ay, 2))

        # Judge acceleration direction
        if a == 0:
            a = 0
        elif (math.cos(uav.attitude.yaw) * (ax/a)) > 0:
            a *= 1
        elif (math.cos(uav.attitude.yaw) * (ax/a)) < 0:
            a *= -1
        elif ax/a == 0 or math.cos(uav.attitude.yaw) == 0:
            if (math.sin(uav.attitude.yaw) * (ay / a)) > 0:
                a *= 1
            elif (math.sin(uav.attitude.yaw) * (ay / a)) < 0:
                a *= -1

        # Noise reduction and update values
        a_coll.insert(0, a)
        if len(a_coll) > SAMPLE_NUMBER:
            a_coll.pop()
        a_ave = np.average(a_coll)

        # Calculate error
        error = TARGET_ACC - a_ave

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * ACC_KP
        gain_i = int_error * ACC_KI
        gain_d = derivative * ACC_KD
        gain_sum = gain_p + gain_i + gain_d

        # Map output into specific range (thrust range -0.5~+0.5)
        pitch_out = gain_sum / ACC_Gain_Division

        # Change the direction
        pitch_out *= -1

        # Renew the thrust value
        TARGET_PITCH = pitch_out

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        # Record data
        R_ACTUAL_ACC.append(a_ave)
        R_ACTUAL_ACC_X.append(ax)
        R_ACTUAL_ACC_Y.append(ay)
        R_ACTUAL_ACC_YAW.append(uav.attitude.yaw)
        R_TARGET_ACC.append(TARGET_ACC)
        R_TARGET_PITCH.append(TARGET_PITCH)

        time.sleep(1/FREQ_ACC)     # Send message at designated Hz

        # Close threading when STATE = False

    print('Acceleration Controller Closed.')


# Altitude Sensor
def altitude_sensor():
    # Initializing
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(US_TRIG_PIN, GPIO.OUT)
    GPIO.setup(US_ECHO_PIN, GPIO.IN)
    GPIO.output(US_TRIG_PIN, False)
    height_us_register = []
    height_bm_register = []
    # height_ls_register = []

    print('Altitude Sensor ON.')

    # Sensor loop
    while STATE:
        global WEIGHT_US_FLAG, WEIGHT_LS_FLAG
        # Read sensor row data
        # Ultrasonic and weight enable flag
        alt_row_us = us_sensor()
        if 0 <= alt_row_us <= 2:
            WEIGHT_US_FLAG = 1
        else:
            WEIGHT_US_FLAG = 0

        # Barometer
        alt_row_bm = uav.location.global_relative_frame.alt + ALT_SENSOR_OFFSET_BM
        if alt_row_bm < 0:
            alt_row_bm = 0

        # Laser and weight enable flag
        # alt_row_ls = LS.range / 1000 + ALT_SENSOR_OFFSET_LS
        # if 0 >= alt_row_ls >= 1.2:
        #     WEIGHT_US_FLAG = 1
        # else:
        #     WEIGHT_US_FLAG = 0

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
        alt_ave = ((WEIGHT_US_FLAG * WEIGHT_US * alt_us
                   + WEIGHT_BM * alt_bm
                   + WEIGHT_LS_FLAG * WEIGHT_LS * alt_ls) /
                   (WEIGHT_US_FLAG * WEIGHT_US + WEIGHT_BM + WEIGHT_LS_FLAG * WEIGHT_LS))

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
    output = cv2.VideoWriter('output.avi', fourcc, 30.0, (480, 360))
    while video.isOpened():
        ret, cap = video.read()
        if ret:
            cap_re = cv2.resize(cap, (480, 360))
            output.write(cap_re)
            cv2.imshow("test", cap_re)

            if cv2.waitKey(5) and not STATE:
                break
        else:
            break
    output.release()
    video.release()
    cv2.destroyAllWindows()
    print('Camera Closed.')


# Function Def. Area 2 (Useful Function)
# Ultra sonic sensor
def us_sensor():
    # Timer settings
    pulse_end = time.time()
    pulse_start = time.time()

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

    # Convert to actual Attitude
    roll_angel = uav.attitude.roll
    pitch_angel = uav.attitude.pitch
    height = height * math.cos(roll_angel) * math.cos(pitch_angel)

    return height


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
def value_write(v, h, t):
    global TARGET_VEL, TARGET_ALT
    print('CMD V=%.2f m/s, H=%.2f m, Duration=%d sec' % (v, h, t))
    TARGET_VEL = v
    TARGET_ALT = h
    for i in range(0, t):
        time.sleep(1)
        print('Now V=%.2f m/s, H=%.2f m, Remaining=%d sec' % (CURRENT_VEL, ACTUAL_ALT, t-i-1))


# Landing Procedure
def landing():
    # Landing & Safety check
    while ACTUAL_ALT > 0.05:
        value_write(0, 0, 1)
        print('Wait for landing...')

    print("UAV landed.")


# Function Def. Area 3 (Main Function)
# Threading procedure
def startup():
    # Check RC switch is in Stabilize mode
    while not uav.mode.name == 'STABILIZE':
        print('Please push RC mode switch to Stabilize mode')
        time.sleep(1)
    print("Now is in Mode: %s" % uav.mode.name)
    time.sleep(1)

    # Changing to Guided_NoGPS Mode
    print("Switching to Guided_NoGPS Mode.")
    uav.mode = VehicleMode("GUIDED_NOGPS")
    while not uav.mode.name == 'GUIDED_NOGPS':
        time.sleep(0.5)
    print("Now is in Mode: %s" % uav.mode.name)

    # Set UAV Armed
    print('Setting the UAV armed')
    uav.armed = True
    time.sleep(1)
    print('UAV is ready to go!')

    # Def. threading.thread
    alt_sen = threading.Thread(target=altitude_sensor)
    alt = threading.Thread(target=altitude_controller)
    vel = threading.Thread(target=velocity_controller)
    acc = threading.Thread(target=acceleration_controller)
    cmd = threading.Thread(target=command_transfer)
    cam = threading.Thread(target=camera)
    m = threading.Thread(target=mission)

    # Start the threading
    alt_sen.start()
    alt.start()
    vel.start()
    acc.start()
    cmd.start()
    cam.start()
    m.start()

    # Wait for threading close
    m.join()
    cmd.join()
    alt.join()
    acc.join()
    vel.join()
    alt_sen.join(2)
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

    # Close the connection
    print('Close the connect to UAV.')
    uav.close()
    print('Shutdown Completed')


# Mission function
def mission():
    global MODE, STATE
    MODE = 'LINE'     # Use line tracking mode to control ALT & VEL

    time.sleep(2)     # Wait for all controller standby

    print('Mission start.')
    # CMD format : value_write(velocity, height, duration)
    value_write(0, 1, 5)
    value_write(1, 1, 3)
    value_write(-1, 1, 3)

    # Go landing
    landing()
    print('Mission complete.')

    # Close all threading
    STATE = False


# Show flight data
def show_data():
    # Altitude control recorder
    for i in range(1, len(R_TARGET_ALT)+1):
        R_NUM_ALT.append(i)

    plt.plot(R_NUM_ALT, R_TARGET_ALT, label="Target Height (m)")
    plt.plot(R_NUM_ALT, R_ACTUAL_ALT, label="Actual Height (m)")
    plt.plot(R_NUM_ALT, R_TARGET_THRUST, label="Thrust")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Altitude Output Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()

    # Velocity control recorder
    for i in range(1, len(R_TARGET_VEL)+1):
        R_NUM_VEL.append(i)

    plt.plot(R_NUM_VEL, R_TARGET_VEL, label="Target Velocity (m/s)")
    plt.plot(R_NUM_VEL, R_CURRENT_VEL, label="Current Velocity (m/s)")
    plt.plot(R_NUM_VEL, R_TARGET_ACC_VEL, label="Acceleration Output (m/s^2)")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Velocity Output Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()

    # Acceleration control recorder
    for i in range(1, len(R_TARGET_ACC)+1):
        R_NUM_ACC.append(i)

    plt.plot(R_NUM_ACC, R_TARGET_ACC, label="Target Acceleration (m/s^2)")
    plt.plot(R_NUM_ACC, R_ACTUAL_ACC, label="Actual Acceleration (m/s^2)")
    plt.plot(R_NUM_ACC, R_TARGET_PITCH, label="Pitch Output (Degree)")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Acceleration Output Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()

    # Acceleration sensor recorder
    plt.plot(R_NUM_ACC, R_ACTUAL_ACC_X, label="Actual Acceleration - X (m/s^2)")
    plt.plot(R_NUM_ACC, R_ACTUAL_ACC_Y, label="Actual Acceleration - Y (m/s^2)")
    plt.plot(R_NUM_ACC, R_ACTUAL_ACC_YAW, label="Yaw Direction (Radius)")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Acceleration Sensor Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()


# Program Procedure
startup()
shutdown()
show_data()
