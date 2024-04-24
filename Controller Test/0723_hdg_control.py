# Altitude Controller Final ver.

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

# Parameter
# UAV connection setting
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# Loop Frequency
FREQ_CMD = 20
FREQ_ALT = 20
FREQ_VEL = 20
FREQ_ACC = 20
FREQ_HDG = 20
FREQ_SENSOR = 50

# Auto mode and flag
MODE = ''
STATE = True
SENSOR_STATE = False

# Target Value
TARGET_PITCH = 0.0
TARGET_ROLL = 0.0
TARGET_YAW = 0.0
TARGET_ANGLE = 0.0
TARGET_ROLL_RATE = 0.0
TARGET_PITCH_RATE = 0.0
TARGET_YAW_RATE = 0.0
TARGET_ALT = 0.0
TARGET_VEL = 0.0
TARGET_ACC = 0.0
TARGET_THRUST = 0.0

# Actual Value (Sensor Feedback)
ACTUAL_PITCH = 0.0
ACTUAL_ROLL = 0.0
ACTUAL_YAW = 0.0
ACTUAL_ALT = 0.0
ACTUAL_ALT_US = 0.0
ACTUAL_VEL = 0.0
ACTUAL_ANGLE = 0.0
CURRENT_VEL = 0.0

# Altitude Controller setting
ALT_KP = 1
ALT_KI = 0
ALT_KD = 0
ALT_Gain_Division = 1

# Altitude Sensor setting
SAMPLE_NUMBER = 5
ALT_SENSOR_OFFSET_BM = -0.06
ALT_SENSOR_OFFSET_LS = 0
WEIGHT_US = 1
WEIGHT_BM = 0
WEIGHT_LS = 0
WEIGHT_US_FLAG = 1
WEIGHT_LS_FLAG = 0

# Ultrasonic Sensor Setting
US_TRIG_PIN = 2
US_ECHO_PIN = 3
AIR_TEMP = 28
ALT_SENSOR_OFFSET_US = 0

# Vel to Acc parameter
M = 1             # Acceleration slope
BETA = 1          # Coefficient of rising time and equal acceleration time

# Acceleration Controller setting
ACC_KP = 1
ACC_KI = 0
ACC_KD = 0
ACC_Gain_Division = 1

# Heading Controller setting
HDG_KP = 1
HDG_KI = 0
HDG_KD = 0
HDG_Gain_Division = 1

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
# For heading controller
R_NUM_HDG = []
R_INCREMENT_YAW = []
R_TARGET_YAW_RATE = []

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

    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

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

        time.sleep(1 / FREQ_ALT)  # Send message at designated Hz

        # Close threading when STATE = False

    # Turn to minimum thrust (Prevent jump)
    TARGET_THRUST = 0

    print('Altitude Controller Closed.')


# Altitude Sensor (Check ok)
def altitude_sensor():
    # Initializing
    height_us_register = []
    height_bm_register = []
    # height_ls_register = []

    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

    print('Altitude Sensor ON.')

    # Sensor loop
    while STATE:
        global WEIGHT_US_FLAG, WEIGHT_LS_FLAG, ACTUAL_ALT
        # Read sensor row data
        # Ultrasonic and weight enable flag
        alt_row_us = ACTUAL_ALT_US
        if alt_row_us < 0:
            alt_row_us = 0

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
        alt_num = (WEIGHT_US_FLAG * WEIGHT_US * alt_us
                   + WEIGHT_BM * alt_bm
                   + WEIGHT_LS_FLAG * WEIGHT_LS * alt_ls)
        alt_den = (WEIGHT_US_FLAG * WEIGHT_US + WEIGHT_BM + WEIGHT_LS_FLAG * WEIGHT_LS)
        alt_ave = alt_num / alt_den

        # Renew the altitude data
        ACTUAL_ALT = alt_ave

        time.sleep(1 / FREQ_SENSOR)  # Activate sensors at designated Hz (Exclude loop time 10ms)

        # Close threading when STATE = False

    print('Altitude Sensor Closed.')


# Ultrasonic sensor (Check ok)
def ultrasonic_sensor():
    global ACTUAL_ALT_US
    # Initializing
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(US_TRIG_PIN, GPIO.OUT)
    GPIO.setup(US_ECHO_PIN, GPIO.IN)
    GPIO.output(US_TRIG_PIN, False)

    # Timer settings for sensor
    pulse_end = time.time()
    pulse_start = time.time()

    # Initialize for Kalman Filter
    x_o = 0  # Previous state
    p_o = 0  # Previous covariance
    z_o = 0  # Previous measurement data
    q = math.exp(-5)  # Covariance noise
    r = 2.92 * math.exp(-3)  # Covariance measurement noise

    # Sensor Calibrate
    us_calibrate()

    print('Ultrasonic Sensor ON.')

    while STATE:
        # Timeout flag
        timeout = True

        # Kalman Filter Predict
        x_p = x_o
        p_p = p_o + q

        # Sensor loop (Timeout prevention)
        while timeout:
            # Trig the emitter
            GPIO.output(US_TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(US_TRIG_PIN, False)
            timer_out = time.time()

            # Record a pulse time
            while not GPIO.input(US_ECHO_PIN):
                pulse_start = time.time()
                if pulse_start - timer_out > 0.1:
                    break
            while GPIO.input(US_ECHO_PIN):
                pulse_end = time.time()
                if pulse_start - pulse_end > 0.1:
                    timeout = True  # Sensor detect unsuccessful
                    break
                timeout = False  # Sensor detect successful

        # Calculate the height and set the filter
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2 + ALT_SENSOR_OFFSET_US

        # Convert to actual Attitude
        roll_angel = uav.attitude.roll
        pitch_angel = uav.attitude.pitch
        height = height * math.cos(roll_angel) * math.cos(pitch_angel)
        if height < 2:
            z = height
        else:
            z = z_o

        # Kalman Filter Innovation
        k = p_p / (p_p + r)  # Kalman Gain
        x = x_p + k * (z - x_p)  # State
        p = (1 - k) * p_p  # Covariance

        # Renew global variable
        ACTUAL_ALT_US = x

        # Pass value
        x_o = x
        p_o = p
        z_o = z

        time.sleep(1 / FREQ_SENSOR)  # Activate sensors at designated Hz (Exclude loop time 10ms)

        # Close threading when STATE = False

    print('Ultrasonic Sensor Closed.')


# Velocity Controller
def velocity_controller():
    # Preset variable
    global ACTUAL_VEL, CURRENT_VEL, TARGET_ACC

    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

    print('Velocity Controller ON.')

    # Controller loop
    while STATE:
        if not TARGET_VEL == ACTUAL_VEL:
            # Save the command
            cmd_vel = TARGET_VEL

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
            ACTUAL_VEL = cmd_vel
            CURRENT_VEL = cmd_vel
            TARGET_ACC = 0

        # Wait time to record
        time.sleep(1 / FREQ_VEL)

        # Record data
        R_CURRENT_VEL.append(CURRENT_VEL)
        R_TARGET_VEL.append(TARGET_VEL)
        R_TARGET_ACC_VEL.append(TARGET_ACC)

        # Close threading when STATE = False

    print('Velocity Controller Closed.')


# Acceleration Controller
def acceleration_controller():
    # Preset variable
    global TARGET_PITCH, TARGET_PITCH_RATE
    int_error = 0
    previous_error = 0
    previous_time = time.time()
    a_coll = []

    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

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

        # Map output into specific range
        pitch_out = gain_sum / ACC_Gain_Division

        # Change the direction
        pitch_out *= -1

        # Renew the pitch and pitch_rate value
        TARGET_PITCH = pitch_out
        if TARGET_PITCH - uav.attitude.pitch > 0:
            TARGET_PITCH_RATE = 1
        elif TARGET_PITCH - uav.attitude.pitch < 0:
            TARGET_PITCH_RATE = -1
        else:
            TARGET_PITCH_RATE = 0

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


# Heading Controller
def heading_controller():
    # Preset variable
    global TARGET_YAW_RATE
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

    print('Heading Controller ON.')

    # Controller loop
    while STATE:
        # Calculate error
        error = TARGET_ANGLE - ACTUAL_ANGLE

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * HDG_KP
        gain_i = int_error * HDG_KI
        gain_d = derivative * HDG_KD
        gain_sum = gain_p + gain_i + gain_d

        # Map output into specific range
        yaw_rate_out = gain_sum / HDG_Gain_Division

        # Renew the yaw rate value
        TARGET_YAW_RATE = yaw_rate_out

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        # Record data
        R_INCREMENT_YAW.append(ACTUAL_ANGLE)
        R_TARGET_YAW_RATE.append(TARGET_YAW_RATE)

        time.sleep(1 / FREQ_HDG)  # Send message at designated Hz

        # Close threading when STATE = False

    print('Heading Controller Closed.')


# Command transfer (Check ok)
def command_transfer():
    while STATE:
        msg = uav.message_factory.set_attitude_target_encode(
            0,  # time boot
            0,  # target system
            0,  # target component
            0b00000000,  # type mask: bit 1 is LSB
            to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # quaternion
            TARGET_ROLL_RATE,  # body roll rate in radian
            TARGET_PITCH_RATE,  # body pitch rate in radian
            TARGET_YAW_RATE,  # body yaw rate in radian
            TARGET_THRUST)  # thrust
        uav.send_mavlink(msg)

        time.sleep(1 / FREQ_CMD)  # Send message at designated Hz

        # Close threading when STATE = False

    print('Command Transfer Closed.')


# Function Def. Area 2 (Useful Function)
# Ultrasonic Sensor Calibrate
def us_calibrate():
    global ALT_SENSOR_OFFSET_US, SENSOR_STATE
    # Height value storage
    height_ave = []

    # Timeout flag
    timeout = True

    # Timer settings for sensor
    pulse_end = time.time()
    pulse_start = time.time()

    # Measure data
    for i in range(0, 10):
        # Sensor loop (Timeout prevention)
        while timeout:
            # Trig the emitter
            GPIO.output(US_TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(US_TRIG_PIN, False)
            timer_out = time.time()

            # Record a pulse time
            while not GPIO.input(US_ECHO_PIN):
                pulse_start = time.time()
                if pulse_start - timer_out > 0.1:
                    break
            while GPIO.input(US_ECHO_PIN):
                pulse_end = time.time()
                if pulse_start - pulse_end > 0.1:
                    timeout = True  # Sensor detect unsuccessful
                    break
                timeout = False  # Sensor detect successful

        # Calculate the height
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2
        height_ave.append(height)

    # Write the offset value
    ALT_SENSOR_OFFSET_US = -np.average(height_ave)
    print('Auto calibrate:', ALT_SENSOR_OFFSET_US, 'm')
    SENSOR_STATE = True
    print('Ultrasonic Sensor Calibrated.')


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


# Write the value into global variable (For AVHT mission mode)
def value_write(angle, v, h, t):
    global TARGET_VEL, TARGET_ALT, TARGET_ANGLE
    print('CMD V=%.2f m/s, H=%.2f m, Duration=%d sec' % (v, h, t))
    TARGET_ANGLE = angle
    TARGET_VEL = v
    TARGET_ALT = h
    for i in range(0, t):
        time.sleep(1)
        print('    Now V=%.2f m/s, H=%.2f m, Remaining=%d sec' % (CURRENT_VEL, ACTUAL_ALT, t-i-1))


# Landing Procedure
def landing():
    # Landing & Safety check
    while ACTUAL_ALT > 0.05:
        value_write(0, 0, 0, 1)
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
    us_sen = threading.Thread(target=ultrasonic_sensor)
    alt_sen = threading.Thread(target=altitude_sensor)
    alt = threading.Thread(target=altitude_controller)
    vel = threading.Thread(target=velocity_controller)
    acc = threading.Thread(target=acceleration_controller)
    cmd = threading.Thread(target=command_transfer)
    m = threading.Thread(target=mission)

    # Start the threading
    us_sen.start()
    alt_sen.start()
    alt.start()
    vel.start()
    acc.start()
    cmd.start()
    m.start()

    # Wait for threading close
    m.join()
    cmd.join()
    alt.join()
    acc.join()
    vel.join()
    alt_sen.join()
    us_sen.join()

    print('All threading closed.')


# Close the UAV
def shutdown():
    # Change to Stabilize mode
    print("Switching to Stabilize Mode")
    uav.mode = VehicleMode("STABILIZE")
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
    # CMD format : value_write(angle, velocity, height, duration)
    value_write(0, 0, 1, 5)
    value_write(0, 1, 1, 3)
    value_write(0, 0, 1, 3)
    value_write(90, 0, 1, 3)
    value_write(0, -1, 1, 3)

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
    plt.plot(R_NUM_ACC, R_ACTUAL_ACC_YAW, label="Yaw Direction (Radian)")

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

    # Heading control recorder
    for i in range(1, len(R_INCREMENT_YAW)+1):
        R_NUM_HDG.append(i)
    plt.plot(R_NUM_HDG, R_INCREMENT_YAW, label="Actual Acceleration (degree)")
    plt.plot(R_NUM_HDG, R_TARGET_YAW_RATE, label="Target Yaw Rate (radian/s)")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Yaw-Rate Output Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()


# Program Procedure
startup()
shutdown()
show_data()
