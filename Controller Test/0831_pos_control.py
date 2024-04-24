# 原地起飛降落的定位測試(驗證定高與定位功能，新的簡易版）

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

# Auto mode and flag
MODE = ''
STATE = True
SENSOR_STATE = False
CAMERA_STATE = False

# Loop Frequency
FREQ_ALT = 20
FREQ_POS = 20
FREQ_CMD = 20
FREQ_CAM = 30
FREQ_SENSOR = 30

# Target Value (Command Value)
TARGET_ROLL = 0
TARGET_PITCH = 0
TARGET_YAW = 0
TARGET_YAW_OFFSET = 0
TARGET_ALT = 0
TARGET_THRUST = 0
TARGET_ROLL_RATE = 0
TARGET_PITCH_RATE = 0
TARGET_YAW_RATE = 0

# Actual Value (Sensor Feedback)
ACTUAL_POS = [0.0, 0.0]
ACTUAL_ALT = 0
ACTUAL_ALT_US = 0

# Altitude Controller setting
ALT_KP = 1
ALT_KI = 0
ALT_KD = 0
ALT_DZ = 0         # Thrust dead zone (oneway)

# Position Controller setting [x,y]
POS_ANGLE_LIMIT = 5  # degree
POS_DZ = 20          # Position dead zone (oneway, unit:px)

# Altitude Sensor setting
SAMPLE_NUMBER = 10
WEIGHT_US = 1
WEIGHT_BM = 0.3

# Ultrasonic Sensor Setting
US_TRIG_PIN = 2
US_ECHO_PIN = 3
AIR_TEMP = 28
ALT_SENSOR_OFFSET_US = 0
Cali_Data_Num_US = 10

# Barometer Sensor Setting
ALT_SENSOR_OFFSET_BM = 0
Cali_Data_Num_BM = 10

# Camera and video processing setting
BINARY_UPPER = 255                        # 固定上限
BINARY_LOWER = 70                         # 0~255 調高比較能看到線，但可能有雜訊
IMAGE_WEIGHT = 480                        # 畫面寬度
IMAGE_HEIGHT = 340                        # 畫面高度
IMAGE_RE = 0
IMAGE_RE_HSV = 0
LOWER_BLUE = np.array([85, 76, 90])
HIGH_BLUE = np.array([127, 255, 255])
LOWER_GREEN = np.array([44, 104, 27])
HIGH_GREEN = np.array([87, 255, 255])
LOWER_RED = np.array([160, 111, 56])
HIGH_RED = np.array([180, 255, 255])
BODY_REFX = int(IMAGE_WEIGHT / 2)         # 機身位置X座標
BODY_REFY = int(IMAGE_HEIGHT / 2)         # 機身位置Y座標
FPS = 10                                  # 畫面刷新延遲時間
VIDEO = cv2.VideoCapture(0)
BLOCK_COLOR = 'red'                       # Target color

# Record Data
# For altitude controller
R_NUM_ALT = []
R_ACTUAL_ALT = []
R_TARGET_THRUST = []
R_TARGET_ALT = []

# For position controller
R_NUM_POS = []
R_ACTUAL_POS_X = []
R_ACTUAL_POS_Y = []
R_TARGET_PITCH = []
R_TARGET_ROLL = []
R_ACTUAL_PITCH = []
R_ACTUAL_ROLL = []

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
        thrust_out = gain_p + gain_i + gain_d

        # Offset to midpoint
        thrust_out += 0.5

        # Thrust dead zone ignore
        # if (0.5+ALT_DZ) > thrust_out > (0.5-ALT_DZ):
        # thrust_out = 0.5

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


# Position Controller (Combine the positioning and CMD output)
def position_controller():
    global TARGET_PITCH, TARGET_ROLL, CAMERA_STATE

    print('Wait for camera on.')
    while not CAMERA_STATE:
        time.sleep(0.1)
    print('Position Controller ON.')

    while STATE:
        # Camera Section
        light_blur = cv2.cvtColor(IMAGE_RE_HSV, cv2.COLOR_BGR2HSV)
        # 閉運算 還有黑線就把矩陣加大
        light_blur = cv2.morphologyEx(light_blur, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        # 依照目標色抓該顏色輪廓
        if BLOCK_COLOR == 'red':
            mask = cv2.inRange(light_blur, LOWER_RED, HIGH_RED)
            w, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pos_cal(contours)
        elif BLOCK_COLOR == 'blue':
            mask = cv2.inRange(light_blur, LOWER_BLUE, HIGH_BLUE)
            w, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pos_cal(contours)
        elif BLOCK_COLOR == 'green':
            mask = cv2.inRange(light_blur, LOWER_GREEN, HIGH_GREEN)
            w, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pos_cal(contours)

        # Judge Section
        if ACTUAL_POS[0] > POS_DZ:             # Right
            TARGET_ROLL = POS_ANGLE_LIMIT
        elif ACTUAL_POS[0] < -POS_DZ:           # Left
            TARGET_ROLL = -POS_ANGLE_LIMIT
        else:                                  # In dead zone
            TARGET_ROLL = 0

        if ACTUAL_POS[1] > POS_DZ:             # Forward
            TARGET_PITCH = POS_ANGLE_LIMIT
        elif ACTUAL_POS[1] < -POS_DZ:           # Backward
            TARGET_PITCH = -POS_ANGLE_LIMIT
        else:                                  # In dead zone
            TARGET_PITCH = 0

        # Record data
        R_ACTUAL_POS_X.append(ACTUAL_POS[0])
        R_ACTUAL_POS_Y.append(ACTUAL_POS[1])
        R_TARGET_ROLL.append(TARGET_ROLL)
        R_TARGET_PITCH.append(TARGET_PITCH)
        R_ACTUAL_ROLL.append(uav.attitude.roll)
        R_ACTUAL_PITCH.append(uav.attitude.pitch)

        time.sleep(1 / FREQ_POS)  # Send message at designated Hz

        # Close threading when STATE = False

    print('Position Controller Closed.')


# Altitude Sensor (Check ok)
def altitude_sensor():
    # Initializing
    height_us_register = []
    height_bm_register = []

    # Wait for sensor settle
    bm_calibrate()              # Barometer Calibrate
    while not SENSOR_STATE:     # Wait for Ultrasonic Sensor Calibrate
        time.sleep(0.1)

    print('Altitude Sensor ON.')

    # Sensor loop
    while STATE:
        global ACTUAL_ALT
        # Read sensor row data
        # Ultrasonic and weight enable flag
        alt_row_us = ACTUAL_ALT_US
        if alt_row_us < 0:
            alt_row_us = 0

        # Barometer
        alt_row_bm = uav.location.global_relative_frame.alt + ALT_SENSOR_OFFSET_BM
        if alt_row_bm < 0:
            alt_row_bm = 0

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

        # Combined the height by weighted average method
        alt_num = (WEIGHT_US * alt_us + WEIGHT_BM * alt_bm)
        alt_den = (WEIGHT_US + WEIGHT_BM)
        alt_ave = alt_num / alt_den

        # Renew the altitude data
        ACTUAL_ALT = alt_ave

        time.sleep(1 / FREQ_SENSOR)  # Activate sensors at designated Hz (Exclude loop time 10ms)

        # Close threading when STATE = False

    print('Altitude Sensor Closed.')


# Ultrasonic sensor (Check ok)
def ultrasonic_sensor():
    global ACTUAL_ALT_US, SENSOR_STATE
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
    SENSOR_STATE = True

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
        roll_angel = uav.attitude.roll           # Remove if use gimbal
        pitch_angel = uav.attitude.pitch         # Remove if use gimbal
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


# Grab the picture
def camera():
    global IMAGE_RE, IMAGE_RE_HSV, CAMERA_STATE

    print('Camera ON.')

    # Camera loop
    while VIDEO.isOpened() and STATE:
        ret = VIDEO.grab()
        if ret:
            ret, line_video = VIDEO.retrieve()
            IMAGE_RE = cv2.resize(line_video, (IMAGE_WEIGHT, IMAGE_HEIGHT))
            IMAGE_RE_HSV = cv2.resize(line_video, (IMAGE_WEIGHT, IMAGE_HEIGHT))

            cv2.imshow("origin", IMAGE_RE)

            if cv2.waitKey(int(1000/FREQ_CAM)) & 0xff == ord("q"):
                break
        else:
            break

        # Set CAMERA_STATE true to enable position_controller()
        CAMERA_STATE = True

        # Close threading by CAMERA_STATE flag (Prevent function conflict)
        if not CAMERA_STATE:
            break

    # Close window
    VIDEO.release()
    cv2.destroyAllWindows()
    print('Camera Closed.')


# Command transfer (Check ok)
def command_transfer():
    print('Command Transfer On.')
    while STATE:
        msg = uav.message_factory.set_attitude_target_encode(
            0,  # time boot
            0,  # target system
            0,  # target component
            0b00000111,  # type mask: bit 1 is ignore
            to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW + TARGET_YAW_OFFSET),  # quaternion
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
    global ALT_SENSOR_OFFSET_US
    # Height value storage
    height_ave = []

    # Timeout flag
    timeout = True

    # Timer settings for sensor
    pulse_end = time.time()
    pulse_start = time.time()

    # Measure data
    for i in range(0, Cali_Data_Num_US):
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
        time.sleep(0.1)

    # Write the offset value
    ALT_SENSOR_OFFSET_US = -np.average(height_ave)
    print('US sensor auto calibrate:', ALT_SENSOR_OFFSET_US, 'm')
    print('Ultrasonic Sensor Calibrated.')


# Barometer Sensor Calibrate
def bm_calibrate():
    global ALT_SENSOR_OFFSET_BM
    # Height value storage
    height_ave = []

    # Measure data
    for i in range(0, Cali_Data_Num_BM):
        # Read the data
        height = uav.location.global_relative_frame.alt
        height_ave.append(height)
        time.sleep(0.1)

    # Write the offset value
    ALT_SENSOR_OFFSET_BM = -np.average(height_ave)
    print('Barometer auto calibrate:', ALT_SENSOR_OFFSET_BM, 'm')
    print('Barometer Sensor Calibrated.')


# Yaw Calibrate
def yaw_calibrate():
    global TARGET_YAW_OFFSET
    TARGET_YAW_OFFSET = -uav.attitude.yaw

    print('Yaw axis auto calibrate:', TARGET_YAW_OFFSET, 'degree')
    print('Yaw axis Calibrated')


# Calculate the moment of block
def pos_cal(contours):
    global ACTUAL_POS

    for c in contours:
        m = cv2.moments(c)

        cx = int(m["m10"] / (m["m00"] + 0.001))
        cy = int(m["m01"] / (m["m00"] + 0.001))
        area = cv2.contourArea(c)
        if area >= 6000:
            ACTUAL_POS[0] = cx - BODY_REFX  # 色塊座標-畫面中央x座標 (右正左負)
            ACTUAL_POS[1] = BODY_REFY - cy  # 畫面中央y座標-色塊座標 (上正下負)
            # 畫箭頭
            cv2.arrowedLine(IMAGE_RE, (BODY_REFX, BODY_REFY), (cx, cy),
                            (255, 255, 255), 3, 0, 0)
            print("相對座標為:%4.1f,%4.1f" % (ACTUAL_POS[0], ACTUAL_POS[1]))


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


# Take off Procedure
def take_off(target_height):
    global TARGET_ALT
    # Altitude command
    TARGET_ALT = target_height

    # Altitude check
    while ACTUAL_ALT < target_height * 0.95:
        print('Now ALT = %.2f m, wait for reach target ALT ...' % ACTUAL_ALT)
        time.sleep(0.5)

    print("UAV Reach target altitude.")


# Landing Procedure
def landing():
    global TARGET_ALT, TARGET_THRUST
    TARGET_ALT = -0.2

    # Landing & Safety check
    while ACTUAL_ALT > 0.05:
        print('Now ALT = %.2f m, wait for landing...' % ACTUAL_ALT)
        time.sleep(0.5)

    # Turn to minimum thrust (Prevent jump)
    TARGET_THRUST = 0
    print("UAV landed.")


# Function Def. Area 3 (Main Function)
# Threading procedure
def startup():
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
    cam = threading.Thread(target=camera)
    alt = threading.Thread(target=altitude_controller)
    pos = threading.Thread(target=position_controller)
    cmd = threading.Thread(target=command_transfer)
    m = threading.Thread(target=mission)

    # Start the threading
    print('Start the threading procedure.')
    us_sen.start()
    alt_sen.start()
    cam.start()
    alt.start()
    pos.start()
    cmd.start()
    m.start()

    # Wait for threading close
    m.join()
    cmd.join()
    alt.join()
    pos.join()
    cam.join()
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
    global TARGET_ALT, STATE

    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

    # Wait for camera settle
    while not CAMERA_STATE:
        time.sleep(0.1)

    print('Mission start.')
    take_off(1)
    time.sleep(5)

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

    # Position-X control recorder
    for i in range(1, len(R_TARGET_ROLL) + 1):
        R_NUM_POS.append(i)

    plt.plot(R_NUM_POS, R_ACTUAL_POS_X, label="Position X (pixel)")
    plt.plot(R_NUM_POS, R_ACTUAL_ROLL, label="Actual Roll (degree)")
    plt.plot(R_NUM_POS, R_TARGET_ROLL, label="Target Roll (degree)")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Position-X Output Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()

    # Position-Y control recorder
    plt.plot(R_NUM_POS, R_ACTUAL_POS_Y, label="Position Y (pixel)")
    plt.plot(R_NUM_POS, R_ACTUAL_PITCH, label="Actual Pitch (degree)")
    plt.plot(R_NUM_POS, R_TARGET_PITCH, label="Target Pitch (degree)")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Position-Y Output Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()


# Program Procedure
startup()
shutdown()
show_data()
