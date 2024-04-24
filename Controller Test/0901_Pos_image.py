# 原地起飛降落的定位測試(影像測試版）

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
from __future__ import print_function
from dronekit import connect
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
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
ALT_DZ = 0.1         # Thrust dead zone (oneway)

# Position Controller setting [x,y]
POS_ANGLE_LIMIT = 5  # degree
POS_DZ = 20          # Position dead zone (oneway, unit:px)

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


# Function Def. Area 2 (Useful Function)
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


# Function Def. Area 3 (Main Function)
# Threading procedure
def startup():
    # Def. threading.thread
    cam = threading.Thread(target=camera)
    pos = threading.Thread(target=position_controller)
    m = threading.Thread(target=mission)

    # Start the threading
    cam.start()
    pos.start()
    m.start()

    # Wait for threading close
    m.join()
    pos.join()
    cam.join()

    print('All threading closed.')


# Mission function
def mission():
    global STATE
    # Wait for camera settle
    while not CAMERA_STATE:
        time.sleep(0.1)

    print('Mission start.')
    for i in range(0, 20):
        print("剩餘%d秒" % (20-i))
        time.sleep(1)

    print('Mission complete.')

    # Close all threading
    STATE = False


# Show flight data
def show_data():
    # Position-X control recorder
    for i in range(1, len(R_TARGET_ROLL) + 1):
        R_NUM_POS.append(i)

    plt.plot(R_NUM_POS, R_ACTUAL_POS_X, label="Position X (pixel)")
    plt.plot(R_NUM_POS, R_ACTUAL_PITCH, label="Actual Pitch (degree)")
    plt.plot(R_NUM_POS, R_TARGET_PITCH, label="Target Pitch (degree)")

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
    plt.plot(R_NUM_POS, R_TARGET_ROLL, label="Target Roll (degree)")

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
show_data()
