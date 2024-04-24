# 0621 Altitude, Attitude, Heading Controller
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

        time.sleep(1/FREQ_ALT)     # Send message at designated Hz

        # Close threading when STATE = False

    print('Altitude Controller Closed.')


# Velocity Controller (Need to check sensor value)
def velocity_controller():
    # Preset variable
    global TARGET_PITCH, TARGET_ROLL
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    print('Altitude Controller ON.')

    # Controller loop
    while STATE:
        # Calculate error
        error = TARGET_VEL - ACTUAL_VEL

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * VEL_KP
        gain_i = int_error * VEL_KI
        gain_d = derivative * VEL_KD
        gain_sum = gain_p + gain_i + gain_d

        # Map output into specific range (angle range -10~+10 degree)
        angle_out = gain_sum / VEL_Gain_Division

        # Output Limitation
        if angle_out > 10:
            angle_out = 10
        if angle_out < -10:
            angle_out = -10

        # Renew the pitch value when in line tracking mode
        if MODE == 'LINE':
            TARGET_PITCH = angle_out
            TARGET_ROLL = 0.0          # Fix roll at 0 degree

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        time.sleep(1/FREQ_VEL)     # Send message at designated Hz

        # Close threading when STATE = False

    print('Altitude Controller Closed.')


# Heading Controller
def heading_controller():
    global TARGET_YAW
    while STATE:
        actual_yaw = uav.attitude.yaw

        if MODE == 'LINE':
            TARGET_YAW = actual_yaw + INCREMENT_YAW

        time.sleep(1 / FREQ_HDG)  # Send message at designated Hz

        # Close threading when STATE = False

    print('Altitude Controller Closed.')


# Position Controller (ACTUAL_POS's value wait for check + or -)
def position_controller():
    # Preset variable
    global ACTUAL_POS, TARGET_ROLL, TARGET_PITCH, TARGET_YAW
    int_error = [0.0, 0.0]
    previous_error = [0.0, 0.0]
    previous_time = [time.time(), time.time()]

    print('Altitude Controller ON.')

    # Controller loop
    while STATE:
        # Calculate error
        error = np.subtract(np.array(TARGET_POS), np.array(ACTUAL_POS))

        # Error Value Preprocess
        current_time = time.time()
        delta_error = np.subtract(np.array(error), np.array(previous_error))
        delta_time = np.subtract(np.array(current_time), np.array(previous_time))
        error_delta_time = np.multiply(np.array(error), np.array(delta_time))
        int_error = np.add(np.array(int_error), np.array(error_delta_time))
        derivative = np.divide(np.array(delta_error), np.array(delta_time))

        # PID Gain and Summation
        gain_p = np.multiply(np.array(error), np.array(POS_KP))
        gain_i = np.multiply(np.array(int_error), np.array(POS_KI))
        gain_d = np.multiply(np.array(derivative), np.array(POS_KD))
        gain_sum = np.add(np.array(gain_p), np.array(gain_i))
        gain_sum = np.add(np.array(gain_sum), np.array(gain_d))

        # Map output into specific range (angle range -10~+10 degree)
        angle_out = np.divide(np.array(gain_sum), np.array(POS_Gain_Division))

        # Take out the value from array
        roll_out = angle_out[0]
        pitch_out = angle_out[1]

        # Output Limitation
        if roll_out > 10:
            roll_out = 10
        if roll_out < -10:
            roll_out = -10
        if pitch_out > 10:
            pitch_out = 10
        if pitch_out < -10:
            pitch_out = -10

        # Renew the pitch value when in line tracking mode
        if MODE == 'POS':
            TARGET_ROLL = roll_out
            TARGET_PITCH = pitch_out

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        time.sleep(1/FREQ_VEL)     # Send message at designated Hz

        # Close threading when STATE = False

    print('Position Controller Closed.')


# Camera Controller (Need to check rotate direction)
def camera_controller():
    # Set the pin mode
    GPIO.setmode(GPIO.BCM)
    output_list = [SERVO_ROLL_PIN, SERVO_PITCH_PIN]
    GPIO.setup(output_list, GPIO.OUT)

    # Creat PWM control item
    servo_r = GPIO.PWM(SERVO_ROLL_PIN, PWM_FREQ)
    servo_p = GPIO.PWM(SERVO_PITCH_PIN, PWM_FREQ)

    # Start at servo's middle point
    servo_r.start(DC_MID)
    servo_p.start(DC_MID)

    print('Camera Servo Controller ON.')

    while STATE:
        # Calculate UAV degree to servo output degree
        roll_out = - uav.attitude.roll
        pitch_out = - uav.attitude.pitch

        # Change degree to duty cycle
        roll_dc = degree_to_dc(roll_out)
        pitch_dc = degree_to_dc(pitch_out)

        # Output the DC%
        servo_p.ChangeDutyCycle(roll_dc)
        servo_r.ChangeDutyCycle(pitch_dc)

        time.sleep(1 / FREQ_CAM)  # Send message at designated Hz

        # Close threading when STATE = False

    print('Camera Servo Controller Closed.')


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


# Velocity Sensor
def velocity_sensor():
    # Preset variable
    global ACTUAL_VEL
    previous_time = time.time()
    vx = 0
    vy = 0

    print('Velocity Sensor ON.')

    # Sensor loop
    while STATE:
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
        ACTUAL_VEL = v

        # Pass values for next loop
        previous_time = current_time

        # Close threading when STATE = False

    print('Velocity Sensor Closed.')


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


# Change degree to duty cycle (For servo control)
def degree_to_dc(degree_cmd):
    degree_abs = degree_cmd + 90
    pwm_cmd = PWM_LOWER_LIMIT + degree_abs * ((PWM_UPPER_LIMIT - PWM_LOWER_LIMIT) / 180)
    dc = PWM_FREQ * (pwm_cmd / 1000000) * 100
    return dc


# Write the value into global variable (For VHT mission mode)
def value_write(v, h, t):
    global TARGET_VEL, TARGET_ALT
    print('CMD V=%.2f m/s, H=%.2f m, Duration=%d sec' % (v, h, t))
    TARGET_VEL = v
    TARGET_ALT = h
    for i in range(0, t):
        time.sleep(1)
        print('Now V=%.2f m/s, H=%.2f m, Remaining=%d sec' % (ACTUAL_VEL, ACTUAL_ALT, t-i-1))


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
    while not uav.is_armable:
        time.sleep(0.5)
        print('Wait for armed...')

    print("Armed = %s" % uav.armed)
    print('UAV is ready to go!')

    # Def. threading.thread
    alt_sen = threading.Thread(target=altitude_sensor)
    vel_sen = threading.Thread(target=velocity_sensor)
    cam = threading.Thread(target=camera_controller)
    alt = threading.Thread(target=altitude_controller)
    vel = threading.Thread(target=velocity_controller)
    hdg = threading.Thread(target=heading_controller)
    pos = threading.Thread(target=position_controller)
    cmd = threading.Thread(target=command_transfer)
    m = threading.Thread(target=mission)

    # Start the threading
    alt_sen.start()
    vel_sen.start()
    cam.start()
    alt.start()
    vel.start()
    hdg.start()
    pos.start()
    cmd.start()
    m.start()

    # Wait for threading close
    m.join()
    cmd.join(3)
    alt.join(3)
    vel.join(3)
    hdg.join(3)
    pos.join(3)
    alt_sen.join(3)
    vel_sen.join(3)
    cam.join(3)

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
    while not uav.is_armable:
        print('Wait for disarmed.')
        time.sleep(1)
    print("Armed = %s" % uav.armed)

    # Close the connection
    print('Close the connect to UAV.')
    uav.close()
    print('Shutdown Completed')


# Mission function
def mission():
    global MODE, STATE
    MODE = 'LINE'     # Use line tracking mode to control ALT & VEL

    print('Mission start.')
    # CMD format : value_write(velocity, height, duration)
    value_write(0, 1, 10)
    value_write(1, 1, 3)
    value_write(-1, 1, 3)

    # Go landing
    landing()
    print('Mission complete.')

    # Close all threading
    STATE = False


# Program Procedure
startup()
shutdown()
