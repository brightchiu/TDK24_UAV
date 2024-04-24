# 原地起飛降落的定位測試(驗證定高與定位功能，改由模擬遙控器操作）
# 0808 增加修訂即時切換（用背面左側撥桿CH7切換手控）

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
FREQ_ALT = 20
FREQ_POS = 20
FREQ_CAM = 30
FREQ_IMG = 30
FREQ_SENSOR = 50

# Auto mode and flag
MODE = ''
STATE = True
SENSOR_STATE = False
CAMERA_STATE = False

# Target Value
# RC CMD
TARGET_ROLL = 1500          # PWM trim value (us)
TARGET_PITCH = 1500         # PWM trim value (us)
TARGET_YAW = 1500           # PWM trim value (us)
TARGET_THR = 1000           # PWM min value (us)
# CMD input
TARGET_ALT = 0              # Alt controller input
TARGET_POS = [0.0, 0.0]     # Pos Controller input (Unused)

# Actual Value (From Sensor)
ACTUAL_ALT = 0
ACTUAL_ALT_US = 0
ACTUAL_POS = [0.0, 0.0]

# Altitude Controller setting
ALT_KP = 1
ALT_KI = 0
ALT_KD = 0
ALT_Gain_Multiply = 500

# Position Controller setting [x,y]
POS_KP = [1, 1]
POS_KI = [0, 0]
POS_KD = [0, 0]
POS_Gain_Multiply = [500, 500]
POS_ANGLE_LIMIT = 5

# Altitude Sensor setting
SAMPLE_NUMBER = 5

# Ultrasonic Sensor Setting
US_TRIG_PIN = 2
US_ECHO_PIN = 3
AIR_TEMP = 28
ALT_SENSOR_OFFSET_US = 0

# RC PWM Limit
RC_PWM_LIMIT_UP = 1750
RC_PWM_LIMIT_DN = 1250
RC_PWM_TRIM = 1500

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
R_TARGET_THR = []
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
    global TARGET_THR
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

        # Map output into specific range (throttle range +500~-500)
        thr_out = gain_sum * ALT_Gain_Multiply

        # Offset to midpoint
        thr_out += RC_PWM_TRIM

        # PWM Output Limitation
        if thr_out > RC_PWM_LIMIT_UP:
            thr_out = RC_PWM_LIMIT_UP
        if thr_out < RC_PWM_LIMIT_DN:
            thr_out = RC_PWM_LIMIT_DN

        # Renew the thrust value
        TARGET_THR = int(thr_out)

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        # Record data
        R_ACTUAL_ALT.append(ACTUAL_ALT)
        R_TARGET_ALT.append(TARGET_ALT)
        R_TARGET_THR.append(TARGET_THR/1000)

        time.sleep(1 / FREQ_ALT)  # Renew controller at designated Hz

        # Close threading when STATE = False

    # Turn to minimum thrust (Prevent jump)
    TARGET_THR = 0

    print('Altitude Controller Closed.')


# Position Controller (ACTUAL_POS's value wait for check + or -)
def position_controller():
    # Preset variable
    global ACTUAL_POS, TARGET_ROLL, TARGET_PITCH
    int_error = [0.0, 0.0]
    previous_error = [0.0, 0.0]
    previous_time = [time.time(), time.time()]

    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

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

        # Map output into specific range (PWM range -500~+500 us)
        angle_out = np.multiply(np.array(gain_sum), np.array(POS_Gain_Multiply))

        # Take out the value from array
        roll_out = - angle_out[0]
        pitch_out = - angle_out[1]

        # Offset to midpoint
        roll_out += RC_PWM_TRIM
        pitch_out += RC_PWM_TRIM

        # Output Limitation
        if roll_out > RC_PWM_LIMIT_UP:
            roll_out = RC_PWM_LIMIT_UP
        elif roll_out < RC_PWM_LIMIT_DN:
            roll_out = RC_PWM_LIMIT_DN
        if pitch_out > RC_PWM_LIMIT_UP:
            pitch_out = RC_PWM_LIMIT_UP
        elif pitch_out < RC_PWM_LIMIT_DN:
            pitch_out = RC_PWM_LIMIT_DN

        # Renew the roll & pitch value
        TARGET_ROLL = int(roll_out)
        TARGET_PITCH = int(pitch_out)

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        # Record data
        R_ACTUAL_POS_X.append(ACTUAL_POS[0])
        R_ACTUAL_POS_Y.append(ACTUAL_POS[1])
        R_TARGET_ROLL.append(TARGET_ROLL/1000)
        R_TARGET_PITCH.append(TARGET_PITCH/1000)
        R_ACTUAL_ROLL.append(uav.attitude.roll)
        R_ACTUAL_PITCH.append(uav.attitude.pitch)

        time.sleep(1 / FREQ_POS)     # Renew controller at designated Hz

        # Close threading when STATE = False

    print('Position Controller Closed.')


# RC Channel Override
def rc_command():
    # Wait for sensor settle
    while not SENSOR_STATE:
        time.sleep(0.1)

    print('RC Command Transfer ON.')

    while STATE:
        # Use RC Override when switch on
        if uav.channels['7'] < 1500:
            uav.channels.overrides = {'1': TARGET_ROLL, '2': TARGET_PITCH, '3': TARGET_THR, '4': TARGET_YAW}

        # Close RC Override when switch off
        if uav.channels['7'] > 1500:
            uav.channels.overrides = {}

        time.sleep(1 / FREQ_CMD)  # Send message at designated Hz

    uav.channels.overrides = {}    # Clear RC channel override
    print('RC Command Transfer Closed.')


# Altitude Sensor (Check ok)
def altitude_sensor():
    # Initializing
    height_us_register = []

    # Wait for sensor settle
    while not SENSOR_STATE:
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

        # Noise reduction and update values
        # Ultrasonic Sensor (US)
        height_us_register.insert(0, alt_row_us)
        if len(height_us_register) > SAMPLE_NUMBER:
            height_us_register.pop()
        alt_us = np.average(height_us_register)

        # Renew the altitude data
        ACTUAL_ALT = alt_us

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

        # Set CAMERA_STATE true to enable img_process()
        CAMERA_STATE = True

    # Close windows
    VIDEO.release()
    cv2.destroyAllWindows()
    print('Camera Closed.')


# Process pictures from Camera
def img_process():
    global CAMERA_STATE

    # Wait for camera on
    while not CAMERA_STATE:
        time.sleep(0.1)

    print('Image process ON.')

    # Image Processing loop
    while STATE:
        light_blur = cv2.cvtColor(IMAGE_RE_HSV, cv2.COLOR_BGR2HSV)
        # 閉運算 還有黑線就把矩陣加大
        light_blur = cv2.morphologyEx(light_blur, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # 依照目標色抓該顏色輪廓
        if BLOCK_COLOR == 'red':
            mask = cv2.inRange(light_blur, LOWER_RED, HIGH_RED)
            x, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pos_cal(contours)
        elif BLOCK_COLOR == 'blue':
            mask = cv2.inRange(light_blur, LOWER_BLUE, HIGH_BLUE)
            x, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pos_cal(contours)
        elif BLOCK_COLOR == 'green':
            mask = cv2.inRange(light_blur, LOWER_GREEN, HIGH_GREEN)
            x, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pos_cal(contours)

        time.sleep(1 / FREQ_IMG)

    print('Image process Closed.')


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
            # Draw arrow
            cv2.arrowedLine(IMAGE_RE, (BODY_REFX, BODY_REFY), (cx, cy),
                            (255, 255, 255), 3, 0, 0)
            print("Relative Position:%4.2f,%4.2f" % (ACTUAL_POS[0], ACTUAL_POS[1]))


# Altitude command input
def alt_cmd(alt):
    global TARGET_ALT
    TARGET_ALT = alt
    print('Go to target altitude %.2f m' % TARGET_ALT)
    while ACTUAL_ALT < TARGET_ALT * 0.95:
        time.sleep(0.1)
    print('Reach target altitude.')


# Landing procedure
def landing():
    global TARGET_ALT
    TARGET_ALT = 0
    print('Wait for landing...')
    while ACTUAL_ALT > 0.05:
        time.sleep(0.1)
    print("UAV landed.")


# Function Def. Area 3 (Main Function)
# Threading procedure
def startup():
    # Changing to Stabilize mode
    print("Switching to Guided_NoGPS Mode.")
    uav.mode = VehicleMode("STABILIZE")
    while not uav.mode.name == 'STABILIZE':
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
    cam_pos = threading.Thread(target=img_process)
    alt = threading.Thread(target=altitude_controller)
    pos = threading.Thread(target=position_controller)
    cmd = threading.Thread(target=rc_command)
    m = threading.Thread(target=mission)

    # Start the threading
    us_sen.start()
    alt_sen.start()
    cam.start()
    cam_pos.start()
    alt.start()
    pos.start()
    cmd.start()
    m.start()

    # Wait for threading close
    m.join()
    cmd.join()
    alt.join()
    pos.join()
    cam_pos.join()
    cam.join()
    alt_sen.join()
    us_sen.join()

    print('All threading closed.')


# Close the UAV
def shutdown():
    # Close the connection
    print('Close the connect to UAV.')
    uav.close()
    print('Shutdown Complete.')


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
    alt_cmd(1)
    time.sleep(5)

    # Go landing
    landing()

    # Close all threading
    STATE = False
    print('Mission complete.')


# Show flight data
def show_data():
    # Altitude control recorder
    for i in range(1, len(R_TARGET_ALT)+1):
        R_NUM_ALT.append(i)

    plt.plot(R_NUM_ALT, R_TARGET_ALT, label="Target Height (m)")
    plt.plot(R_NUM_ALT, R_ACTUAL_ALT, label="Actual Height (m)")
    plt.plot(R_NUM_ALT, R_TARGET_THR, label="Throttle PWM(ms)")

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
    plt.plot(R_NUM_POS, R_ACTUAL_PITCH, label="Actual Pitch (degree)")
    plt.plot(R_NUM_POS, R_TARGET_PITCH, label="Target Pitch PWM (ms)")

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
    plt.plot(R_NUM_POS, R_ACTUAL_ROLL, label="Actual Roll (degree)")
    plt.plot(R_NUM_POS, R_TARGET_ROLL, label="Target Roll PWM (ms)")

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
