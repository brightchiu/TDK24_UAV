# TDK24屆比賽用正式程式碼
# GUI控制介面
# 樹莓派專用版本
# C08 台灣大艦隊

# 更動日誌：
# 0912 完成初版程式碼
# 0913 完成GUI介面
# 0914 樹莓派介面套用完成
# 尚待增加視訊顯示

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# 載入模組
from __future__ import print_function
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math
import cv2

# ========== 設定全域變數 ===============================================
print('<啟動程序> 載入全域變數設定')

# UAV 通訊參數
# 樹莓派直連
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# MacBook直連
# CONNECT_PORT = '/dev/tty.SLAB_USBtoUART'
# CONNECT_BAUD = 57600

# ========== 旗標與變數 ================================================
# 旗標
STATE_SYSTEM = True             # 系統旗標(關閉線程用)
STATE_INFO = True               # GUI資訊更新旗標
STATE_THREAD = False            # 多線程旗標
STATE_ALT = False               # 高度控制器旗標
STATE_POS = False               # 位置控制器旗標
STATE_HDG = False               # 航向控制器旗標
STATE_CMD = False               # 命令傳輸程式旗標
STATE_CAM = False               # 影像擷取程式旗標
STATE_SENSOR_SONAR = False             # 超音波高度計旗標
STATE_SENSOR = False            # 高度感測器旗標
STATE_BOX = False               # 沙包盒艙門
STATE_STAGE = 0                 # 任務所在階段
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)
MISSION_COLOR = ''              # 任務顏色(blue or green)

# 多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# 目標值(命令值)
TARGET_ALT = 0                  # 目標高度，主程式輸入於高度控制器(m)
TARGET_ROLL = 0                 # 目標滾轉角(degree)
TARGET_PITCH = 0                # 目標俯仰角(degree)
TARGET_YAW = 0                  # 目標偏航角(degree)，不使用
TARGET_YAW_OFFSET = 0           # 目標偏航角補正值(degree)
TARGET_THRUST = 0               # 目標推力，表示升降速度(0.5為中點)
TARGET_ROLL_RATE = 0            # 滾轉角速度(degree/s)，不使用
TARGET_PITCH_RATE = 0           # 俯仰角速度(degree/s)，不使用
TARGET_YAW_RATE = 0             # 偏航角速度(degree/s)，需搭配控制器控制角度

# 實際值(量測值)
ACTUAL_ALT = 0                  # 實際高度
ACTUAL_ALT_SONAR = 0            # 超音波高度計
ACTUAL_ALT_BAROM = 0            # 氣壓高度計
ACTUAL_POS = [0, 0]             # 色塊實際位置
ACTUAL_AREA = 0                 # 色塊面積
ACTUAL_HDG_ERR = 0              # 中線夾角

# ========== 控制器參數設定 =============================================
# 迴圈頻率(預設30Hz)
FREQ_ALT = 30                   # 高度控制器頻率(不快於高度感測器頻率)
FREQ_POS = 30                   # 位置控制器頻率(不快於影像禎率)
FREQ_HDG = 30                   # 航向控制器頻率(不快於影像禎率)
FREQ_SONAR = 30                 # 超音波感測器頻率(5~50Hz，越高雜訊越多)
FREQ_SENSOR = 30                # 高度感測器頻率(不快於超音波感測器頻率)
FREQ_CMD = 30                   # 命令傳輸頻率(不快於控制器頻率)
FREQ_CAM = 30                   # 影像禎率(fps，依系統負載調整)
FREQ_INFO = 5                   # GUI畫面資訊更新率(影響影像顯示速度)

# 高度控制器參數
ALT_KP = 1                      # P增益值*
ALT_KI = 0                      # I增益值*
ALT_KD = 0.1                    # D增益值*
ALT_DZ = 0                      # 油門平飛安定區間(越小時操作較細膩，加大則可以避免震盪，單向)*

# 位置控制器參數
POS_COLOR = ''                      # 定位的色塊顏色
POS_SAMPLE_NUM_AVE = 10             # 色塊座標均值濾波取樣次數*
POS_DZ_IN_X = 10                    # 色塊定位作動忽略區間(單向，px)*
POS_DZ_IN_Y = POS_DZ_IN_X*(3/4)     # 色塊定位作動忽略區間(單向，px)
POS_DZ_OUT_X = 200                  # 外側範圍，固定輸出下限(單向，px)*
POS_DZ_OUT_Y = POS_DZ_OUT_X*(3/4)   # 外側範圍，固定輸出下限(單向，px)
POS_ANGLE_LIMIT = 5                 # 定位控制最大飛行角度(degree)

# 航向控制器參數
HDG_KP = 1                      # P增益值*
HDG_KI = 0                      # I增益值*
HDG_KD = 0                      # D增益值*
HDG_DZ = 5                      # 航向安定區間(單向)*
HDG_SAMPLE_NUM_AVE = 10         # 角度差均值濾波取樣次數*
HDG_YAW_RATE_LIMIT = 10         # 角速度限制(degree/s)*
HDG_PITCH_ANGLE_LIMIT = 5       # 前進傾角限制(degree，建議五度以下)*
HDG_YAW_ROLL_RATIO = 0.3        # 滾轉輔助轉向比率*
HDG_ROLL_LIMIT = 4              # 滾轉角限制(degree)*

# 命令傳輸程式參數
CMD_MSG = 3                     # 命令編碼模式(1:角度 2:角度補正 3:角速度，本程式限用模式三)

# 飛行程序參數
TAKEOFF_SETTLE = 95             # 安定高度百分比(%，高度確認門檻)*
TURN_SETTLE = 95                # 安定轉角百分比(%，停止轉向輸出)*
TURN_YAW_RATE = 20              # 轉角速度(degree/s)*
LANDING_CUTOFF = 0.07           # 油門關閉高度(m，停止油門輸出)*
LANDING_POS_LIMIT_X = 50                            # 降落定位X範圍限制(單向，px)*
LANDING_POS_LIMIT_Y = LANDING_POS_LIMIT_X*(3/4)     # 降落定位Y範圍限制(單向，px)*

# ========== 影像參數設定 ===============================================
# 相機設定參數
CAM_VIDEO = cv2.VideoCapture(0)             # 影像來源(預設來源為0)
CAM_IMAGE = 0                               # 儲存影像
CAM_IMAGE_LIVE = 0                          # 即時影像
CAM_IMAGE_WIDTH = 480                       # 影像寬度(px，4:3設定 )
CAM_IMAGE_HEIGHT = 360                      # 影像高度(px)
VIDEO_FOURCC = cv2.VideoWriter_fourcc(*'XVID')
VIDEO_SAVE = cv2.VideoWriter('Flight Camera.avi',
                             VIDEO_FOURCC, FREQ_CAM, (CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT))

# 影像計算參數
IMG_BLUE_L = np.array([85, 76, 90])         # 藍色色域-下限
IMG_BLUE_H = np.array([127, 255, 255])      # 藍色色域-上限
IMG_GREEN_L = np.array([44, 104, 27])       # 綠色色域-下限
IMG_GREEN_H = np.array([87, 255, 255])      # 綠色色域-上限
IMG_RED_L = np.array([160, 111, 56])        # 紅色色域-下限
IMG_RED_H = np.array([180, 255, 255])       # 紅色色域-上限
IMG_BLACK_L = np.array([0, 0, 0])           # 黑色色域-下限
IMG_BLACK_H = np.array([180, 255, 75])      # 黑色色域-上限
IMG_MORPHOLOGY_EX_KERNEL = (5, 5)           # 閉運算矩陣大小(奇數，越大降噪能力越強)*
IMG_AREA_LIMIT = 3000                       # 取用面積下限值*
BODY_REF_X = int(CAM_IMAGE_WIDTH / 2)       # 畫面參考X座標中心
BODY_REF_Y = int(CAM_IMAGE_HEIGHT / 2)      # 畫面位置Y座標中心
BODY_OFFSET_X = 0                           # 機身中心X偏移量(px，左負右正)*
BODY_OFFSET_Y = 0                           # 機身中心Y偏移量(px，上正下負)*

# 線角度計算
ANG_THRESHOLD = 40              # 二值化門檻值(調高比較能看到線，但可能有雜訊)*
ANG_THRESHOLD_NEW = 255         # 取得黑白照填入最高值
ANG_GB_KERNEL_SIZE = (7, 7)     # 高斯模糊矩陣大小(奇數，越大越模糊)*
ANG_GB_SIGMA = 2                # 高斯模糊色域標準差(越大越模糊)*
ANG_LINE_TRACK_AREA_L = 450     # 線追蹤型心計算面積下限*

# ========== 貨物艙參數 ================================================
# 沙包盒參數
BOX_PWM_PIN = 4                 # 沙包盒PWM輸出腳位
BOX_PWM_FREQ = 50               # PWM頻率(方波Hz)
BOX_PWM_OPEN = 400              # 開門位置(400~2350us)*
BOX_PWM_CLOSE = 2350            # 關門位置(400~2350us)*
DROP_POS_LIMIT_X = 100                      # 空投定位X範圍限制(單向，px)*
DROP_POS_LIMIT_Y = DROP_POS_LIMIT_X*(3/4)   # 空投定位X範圍限制(單向，px)*

# ========== 高度感測器參數 =============================================
# ->高度感測器整合參數
BAROM_SAMPLE_NUM_AVE = 10       # 均值濾波取樣次數*
WEIGHTED_SONAR = 0              # 超音波高度計權重*
WEIGHTED_BAROM = 1              # 氣壓高度計權重*

# ->超音波高度計參數
SONAR_AIR_TEMP = 28             # 氣溫(攝氏)*
SONAR_TRIG_PIN = 2              # TRIG輸出腳位
SONAR_ECHO_PIN = 3              # ECHO輸入腳位
SONAR_TIMEOUT = 0.015           # 無回應超時限制(秒，過低時遠距離偵測將會變成超時)*
SONAR_OFFSET = 0                # 高度校正偏移量(m，向上為正)
SONAR_SAMPLE_NUM_CAL = 10       # 高度校正取樣次數*
SONAR_SAMPLE_NUM_AVE = 10       # 均值濾波取樣次數*
SONAR_KF_INPUT_MAX = 3          # 卡爾曼濾波器輸入值限制(高度，m)*
SONAR_KF_Q = 5                  # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*
SONAR_ERR_COUNT = 0             # 量測超時次數記錄
SONAR_FILTER_ALT = 0.2          # 突波濾波作用起始高度(m)
SONAR_MAX_CHANGE = 0.25         # 突波容許值(m)

# ->氣壓計參數(可能是IMU)
BAROM_OFFSET = 0                # 高度校正偏移量(m，向上為正)
BAROM_SAMPLE_NUM_CAL = 10       # 高度校正取樣次數*
BAROM_KF_INPUT_MAX = 3          # 卡爾曼濾波器輸入值限制(高度，m)*
BAROM_KF_Q = 5                  # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*

# ========== 數值記錄 ==================================================
# 高度控制器
R_TARGET_ALT = []                       # 記錄目標高
R_ACTUAL_ALT_CTRL = []                  # 記錄實際高
R_TARGET_THRUST = []                    # 記錄目標推力
R_ALT_ERROR = []                        # 記錄高度誤差
R_ALT_GP = []                           # 記錄P增益
R_ALT_GI = []                           # 記錄I增益
R_ALT_GD = []                           # 記錄D增益

# 位置控制器
R_TARGET_ROLL = []                      # 記錄目標滾轉角
R_TARGET_PITCH = []                     # 記錄目標俯仰角
R_ACTUAL_ROLL = []                      # 記錄實際滾轉角
R_ACTUAL_PITCH = []                     # 記錄實際俯仰角
R_BLOCK_X_RAW = []                      # 記錄X座標原始值
R_BLOCK_Y_RAW = []                      # 記錄Y座標原始值
R_BLOCK_AREA = []                       # 記錄面積變化
R_RECORD_SCALE = 10000                  # 面積縮放比
R_BLOCK_X = []                          # 記錄X座標
R_BLOCK_Y = []                          # 記錄Y座標

# 航向控制器
R_HDG_ERROR_RAW = []                    # 記錄原始差角變化
R_HDG_ERROR = []                        # 記錄差角變化
R_TARGET_YAW_RATE = []                  # 記錄目標偏航角速度
R_HDG_GP = []                           # 記錄P增益
R_HDG_GI = []                           # 記錄I增益
R_HDG_GD = []                           # 記錄D增益
R_HDG_TARGET_ROLL = []                  # 記錄滾轉角

# 高度感測器
R_ACTUAL_ALT = []                       # 記錄加權值
R_ACTUAL_ALT_SONAR = []                 # 記錄超音波高度計值
R_ACTUAL_ALT_BAROM = []                 # 記錄氣壓高度計值

# 超音波高度計
R_ACTUAL_ALT_SONAR_RAW = []             # 記錄原始值
R_ACTUAL_ALT_SONAR_OFFSET = []          # 記錄補正值
R_ACTUAL_ALT_SONAR_AVE = []             # 記錄平均值
R_ACTUAL_ALT_SONAR_KF = []              # 記錄KF值

# 氣壓高度計
R_ACTUAL_ALT_BAROM_RAW = []             # 記錄原始值
R_ACTUAL_ALT_BAROM_OFFSET = []          # 記錄補正值
R_ACTUAL_ALT_BAROM_AVE = []             # 記錄平均值
R_ACTUAL_ALT_BAROM_KF = []              # 記錄KF值

print('<啟動程序> 載入全域變數設定完畢\n')

# 與UAV執行連線
print('<啟動程序> 與UAV連線中，請稍候...')
uav = connect(CONNECT_PORT, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線，啟動GUI介面')


# ====================================================================
# ========== 背景主函式函式區塊 =========================================
# ====================================================================
# 本區為主要運作的函式，以控制器為主，包含高度、位置、航向、雙高度感測器、
# 相機、命令傳送，多線程執行時能達到協作的處理功能
# (但仍受每個迴圈處理時間不一影響，為非同步處理)

# ========== 高度控制器 =========
# 說明：
# 為PID控制器型態，於任務期間全程作用，輸入目標高度後，依據高度感測器執行誤差PID增益，
# 最終輸出成爬升速率型態(符合MavLink指令)，設有輸出範圍限制器、中點飽和區間功能
def altitude_controller():
    global TARGET_THRUST, STATE_ALT
    # PID變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 等待感測器啟動
    if not STATE_SENSOR:
        msg_window.insert(1.0, '<啟動程序> 高度控制器：等待高度感測器啟動\n')
    while not STATE_SENSOR:
        time.sleep(0.1)

    # 回報高度控制器啟用
    STATE_ALT = True
    msg_window.insert(1.0, '<啟動程序> 高度控制器：啟動\n')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 計算誤差值
        error = TARGET_ALT - ACTUAL_ALT

        # 誤差值微積分計算
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # 誤差增益處理
        gain_p = error * ALT_KP
        gain_i = int_error * ALT_KI
        gain_d = derivative * ALT_KD
        thrust_out = gain_p + gain_i + gain_d

        # 補正至中點
        thrust_out += 0.5

        # 推力平飛安定區間
        if (0.5+ALT_DZ) > thrust_out > (0.5-ALT_DZ):
            thrust_out = 0.5

        # 輸出限制(0下降max ~ 1上升max)
        if thrust_out > 1:
            thrust_out = 1
        if thrust_out < 0:
            thrust_out = 0

        # 更新推力值至全域變數
        TARGET_THRUST = thrust_out

        # 傳遞值給下次迴圈使用
        previous_error = error
        previous_time = current_time

        # 記錄數值
        R_TARGET_ALT.append(TARGET_ALT)
        R_ACTUAL_ALT_CTRL.append(ACTUAL_ALT)
        R_TARGET_THRUST.append(TARGET_THRUST)
        R_ALT_ERROR.append(error)
        R_ALT_GP.append(gain_p)
        R_ALT_GI.append(gain_i)
        R_ALT_GD.append(gain_d)

        # 同捆執行終點
        THREAD_OCCUPY.release()

        # 迴圈執行間隔
        time.sleep(1 / FREQ_ALT)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_ALT = False
    msg_window.insert(1.0, '<關閉程序> 高度控制器：關閉\n')


# ========== 位置控制器 ==========
# 說明：
# 為條件式P控制器型態，於定位全程作用，輸入目標高度後，依據誤差於作用區間作線性增益
# 每個迴圈讀取一次色塊資訊，並對資料做均值濾波，確保數值平穩運作
# 座標設定切斷(Cut off)/作動(Act)/飽和(Saturation)區間，對應相對應輸出
def position_controller():
    global TARGET_PITCH, TARGET_ROLL, ACTUAL_POS, ACTUAL_AREA, STATE_POS
    # 均值濾波器設定
    x_register = []
    y_register = []

    # 等待相機啟動
    if not STATE_CAM:
        msg_window.insert(1.0, '<啟動程序> 位置控制器：等待影像擷取程式啟動\n')
    while not STATE_CAM:
        time.sleep(0.1)

    # 回報定位控制器啟用
    STATE_POS = True
    msg_window.insert(1.0, '<啟動程序> 位置控制器：啟動\n')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        # 定位作動迴圈(於定位模式作動)
        while MODE == 'POS' and STATE_SYSTEM:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 取得色塊XY座標
            x_raw, y_raw, block_area = img_calculate(POS_COLOR)

            # X座標均值濾波器
            x_register.insert(0, x_raw)
            if len(x_register) > POS_SAMPLE_NUM_AVE:
                x_register.pop()
            block_x = np.average(x_register)

            # Y座標均值濾波器
            y_register.insert(0, y_raw)
            if len(y_register) > POS_SAMPLE_NUM_AVE:
                y_register.pop()
            block_y = np.average(y_register)

            # 輸出座標至全域變數
            ACTUAL_POS = [int(block_x), int(block_y)]
            ACTUAL_AREA = int(block_area)

            # 判斷與輸出姿態命令(使用比例式控制)
            # X座標
            if block_x > POS_DZ_OUT_X:  # 向右最快飛行
                TARGET_ROLL = POS_ANGLE_LIMIT
            elif block_x < -POS_DZ_OUT_X:  # 向左最快飛行
                TARGET_ROLL = -POS_ANGLE_LIMIT
            elif POS_DZ_IN_X <= block_x <= POS_DZ_OUT_X:  # 向右比例飛行
                percent_x = (block_x - POS_DZ_IN_X) / (POS_DZ_OUT_X - POS_DZ_IN_X)
                TARGET_ROLL = POS_ANGLE_LIMIT * percent_x
            elif -POS_DZ_IN_X >= block_x >= -POS_DZ_OUT_X:  # 向左比例飛行
                percent_x = (-POS_DZ_IN_X - block_x) / (POS_DZ_OUT_X - POS_DZ_IN_X)
                TARGET_ROLL = -POS_ANGLE_LIMIT * percent_x
            else:  # 定位作動忽略區間
                TARGET_ROLL = 0

            # Y座標
            if block_y > POS_DZ_OUT_Y:  # 向前最快飛行
                TARGET_PITCH = -POS_ANGLE_LIMIT
            elif block_y < -POS_DZ_OUT_Y:  # 向後最快飛行
                TARGET_PITCH = POS_ANGLE_LIMIT
            elif POS_DZ_IN_Y <= block_y <= POS_DZ_OUT_Y:  # 向前比例飛行
                percent_y = (block_y - POS_DZ_IN_Y) / (POS_DZ_OUT_Y - POS_DZ_IN_Y)
                TARGET_PITCH = -POS_ANGLE_LIMIT * percent_y
            elif -POS_DZ_IN_Y >= block_y >= -POS_DZ_OUT_Y:  # 向後比例飛行
                percent_y = (-POS_DZ_IN_Y - block_y) / (POS_DZ_OUT_Y - POS_DZ_IN_Y)
                TARGET_PITCH = POS_ANGLE_LIMIT * percent_y
            else:  # 定位作動忽略區間
                TARGET_PITCH = 0

            # 傾角安全限制(設定30，固定設定)
            if TARGET_ROLL > 30:        # Roll安全設定
                TARGET_ROLL = 30
            elif TARGET_ROLL < -30:
                TARGET_ROLL = -30
            if TARGET_PITCH > 30:       # Pitch安全設定
                TARGET_PITCH = 30
            elif TARGET_PITCH < -30:
                TARGET_PITCH = -30

            # 記錄數值
            R_BLOCK_X_RAW.append(x_raw)
            R_BLOCK_Y_RAW.append(y_raw)
            R_BLOCK_AREA.append(block_area / R_RECORD_SCALE)
            R_BLOCK_X.append(block_x)
            R_BLOCK_Y.append(block_y)
            R_TARGET_ROLL.append(TARGET_ROLL)
            R_TARGET_PITCH.append(TARGET_PITCH)
            R_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
            R_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_POS)

            # 當飛行模式不為定位時，跳出迴圈並暫停函式
            if not MODE == 'POS':
                # 歸零姿態數值
                TARGET_ROLL = 0
                STATE_POS = False
                msg_window.insert(1.0, '<飛行狀態> 位置控制器：暫停\n')

        # 無定位作動區間
        # 記錄值
        R_BLOCK_X_RAW.append(0)
        R_BLOCK_Y_RAW.append(0)
        R_BLOCK_AREA.append(0)
        R_BLOCK_X.append(0)
        R_BLOCK_Y.append(0)
        R_TARGET_ROLL.append(0)
        R_TARGET_PITCH.append(0)
        R_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
        R_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

        # 迴圈執行間隔
        time.sleep(1 / FREQ_POS)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_SYSTEM:
            STATE_POS = True
            # 重設暫存器
            x_register = []
            y_register = []
            msg_window.insert(1.0, '<飛行狀態> 位置控制器：啟動\n')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_POS = False
    msg_window.insert(1.0, '<關閉程序> 位置控制器：關閉\n')


# ========== 航向控制器 ==========
# 說明：
# 採用PID控制器型態，每一迴圈讀取一次影像計算線條型心，並將型心值做均值綠波
# 濾波完畢後再以該值計算出型心至畫面下方中心之夾角，並以此值為控制器的誤差值
# 以該值來做PID增益處理，輸出角速度來追尋角度，此控制器可處理平行、轉角、Ｔ路口轉向
# 為不可逆型循跡控制器，參數可調最大角速度輸出、切斷區間、濾波等級、面積門檻值
def heading_controller():
    global ACTUAL_HDG_ERR, TARGET_ROLL, TARGET_PITCH, TARGET_YAW_RATE, STATE_HDG
    # PID變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 均值濾波器設定
    block_x_register = []
    block_y_register = []
    hdg_register = []

    # 執行偏航角校正
    yaw_calibrate()

    # 等待相機啟動
    if not STATE_CAM:
        msg_window.insert(1.0, '<啟動程序> 位置控制器：等待影像擷取程式啟動\n')
    while not STATE_CAM:
        time.sleep(0.1)

    # 回報定位控制器啟用
    STATE_HDG = True
    msg_window.insert(1.0, '<啟動程序> 航向控制器：啟動\n')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        # 定位作動迴圈(於循跡模式作動)
        while MODE == 'LINE' and STATE_SYSTEM:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 取得型心位置
            block_x_raw, block_y_raw = img_moment()

            # 型心位置均值濾波器
            # X座標均值濾波器
            block_x_register.insert(0, block_x_raw)
            if len(block_x_register) > HDG_SAMPLE_NUM_AVE:
                block_x_register.pop()
            block_x = np.average(block_x_register)

            # Y座標均值濾波器
            block_y_register.insert(0, block_y_raw)
            if len(block_y_register) > HDG_SAMPLE_NUM_AVE:
                block_y_register.pop()
            block_y = np.average(block_y_register)

            # 取得現在角度誤差值(中心為目標值，右斜為負，左斜為正)
            error_raw = angle_calculate(block_x, block_y)

            # 誤差角均值濾波器
            hdg_register.insert(0, error_raw)
            if len(hdg_register) > HDG_SAMPLE_NUM_AVE:
                hdg_register.pop()
            error = np.average(hdg_register)
            ACTUAL_HDG_ERR = error  # 輸出角度誤差

            # 誤差值微積分計算
            current_time = time.time()
            delta_error = error - previous_error
            delta_time = current_time - previous_time
            int_error += (error * delta_time)
            derivative = delta_error / delta_time

            # 誤差增益處理
            gain_p = error * HDG_KP
            gain_i = int_error * HDG_KI
            gain_d = derivative * HDG_KD
            angular_rate = gain_p + gain_i + gain_d

            # 航向固定安定區間
            if HDG_DZ > angular_rate > - HDG_DZ:
                angular_rate = 0

            # 角速度上下限制
            if angular_rate > HDG_YAW_RATE_LIMIT:
                angular_rate = HDG_YAW_RATE_LIMIT
            elif angular_rate < -HDG_YAW_RATE_LIMIT:
                angular_rate = -HDG_YAW_RATE_LIMIT

            # 輸出滾轉做轉向輔助(未驗證)
            roll = angular_rate * HDG_YAW_ROLL_RATIO
            if roll > HDG_ROLL_LIMIT:
                roll = HDG_ROLL_LIMIT
            elif roll < -HDG_ROLL_LIMIT:
                roll = -HDG_ROLL_LIMIT

            # 更新目標值至全域變數
            TARGET_YAW_RATE = angular_rate
            TARGET_ROLL = roll
            TARGET_PITCH = -5

            # 傳遞值給下次迴圈使用
            previous_error = error
            previous_time = current_time

            # 記錄數值
            R_HDG_ERROR_RAW.append(error_raw)
            R_HDG_ERROR.append(error)
            R_HDG_GP.append(gain_p)
            R_HDG_GI.append(gain_i)
            R_HDG_GD.append(gain_d)
            R_TARGET_YAW_RATE.append(TARGET_YAW_RATE)
            R_HDG_TARGET_ROLL.append(TARGET_ROLL)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_HDG)

            # 當飛行模式不為循跡時，跳出迴圈並暫停函式
            if not MODE == 'LINE':
                # 歸零姿態數值
                TARGET_PITCH = 0
                TARGET_YAW_RATE = 0
                STATE_HDG = False
                msg_window.insert(1.0, '<飛行狀態> 航向控制器：暫停\n')

        # 無循跡作動區間
        # 回報迴圈休眠狀態
        if MODE == 'POS':
            STATE_HDG = False

        # 重設暫存器
        hdg_register = []

        # 記錄值
        R_HDG_ERROR_RAW.append(0)
        R_HDG_ERROR.append(0)
        R_HDG_GP.append(0)
        R_HDG_GI.append(0)
        R_HDG_GD.append(0)
        R_TARGET_YAW_RATE.append(TARGET_YAW_RATE)
        R_HDG_TARGET_ROLL.append(0)

        # 迴圈執行間隔
        time.sleep(1 / FREQ_HDG)

        # 當飛行模式為循跡時，跳入迴圈並啟動函式
        if MODE == 'LINE' and STATE_SYSTEM:
            STATE_HDG = True
            msg_window.insert(1.0, '<飛行狀態> 航向控制器：啟動\n')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_HDG = False
    msg_window.insert(1.0, '<關閉程序> 航向控制器：關閉\n')


# =========== 高度感測器整合程式 ==========
# 說明：
# 負責整合超音波高度計、飛控氣壓計(可能為IMU來源)，做訊號整理給控制器使用
# 各訊號先經均值濾波後，再用卡爾曼濾波處理，最後再針對所有來源做加權平均
# 本函式處理氣壓計之濾波、高度感測器整合濾波
def altitude_sensor():
    global ACTUAL_ALT, ACTUAL_ALT_BAROM, STATE_SENSOR
    # 平均降噪設定
    height_register_barom = []

    # 卡爾曼濾波器初始化(氣壓高度計用)
    x_o = 0                                # 前一次狀態
    p_o = 0                                # 前一次斜方差
    z_o = 0                                # 前一次量測值
    q = math.exp(-BAROM_KF_Q)              # 斜方差噪音值
    r = 2.92 * math.exp(-3)                # 斜方差量測噪音值(定值不更改)

    # 執行氣壓高度計校正程序
    barometer_calibrate()

    # 等待超音波高度計啟用
    if not STATE_SENSOR_SONAR:
        msg_window.insert(1.0, '<啟動程序> 高度感測器：等待超音波高度計啟動\n')
    while not STATE_SENSOR_SONAR:
        time.sleep(0.1)

    # 回報高度感測器啟用
    STATE_SENSOR = True
    msg_window.insert(1.0, '<啟動程序> 高度感測器：啟動\n')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 讀取超音波高度計資料
        height_sonar = ACTUAL_ALT_SONAR

        # 氣壓高度計數據處理
        # 卡爾曼濾波-預測
        x_p = x_o
        p_p = p_o + q

        # 讀值與補正
        height_raw_barom = uav.location.global_relative_frame.alt + BAROM_OFFSET

        # 均值濾波器
        height_register_barom.insert(0, height_raw_barom)
        if len(height_register_barom) > BAROM_SAMPLE_NUM_AVE:
            height_register_barom.pop()
        height_barom_ave = np.average(height_register_barom)

        # 卡爾曼濾波器輸入限制(防止上衝突波)
        if height_barom_ave <= BAROM_KF_INPUT_MAX:
            z = height_barom_ave       # 若低於限制則使用新值
        else:
            z = z_o                    # 若超出限制則使用舊值

        # 卡爾曼濾波-更新
        k = p_p / (p_p + r)            # 卡爾曼增益
        x = x_p + k * (z - x_p)        # 狀態(濾波輸出值)
        p = (1 - k) * p_p              # 斜方差

        # 更新高度值至變數
        height_barom = x
        ACTUAL_ALT_BAROM = height_barom

        # 卡爾曼濾波：傳遞值給下次迴圈使用
        x_o = x
        p_o = p
        z_o = z

        # 加權平均(可調整加權值)
        alt_num = (WEIGHTED_SONAR * height_sonar + WEIGHTED_BAROM * height_barom)
        alt_den = (WEIGHTED_SONAR + WEIGHTED_BAROM)
        alt_ave = alt_num / alt_den

        # 更新高度值至全域變數
        ACTUAL_ALT = alt_ave
        ACTUAL_ALT_BAROM = height_barom

        # 記錄數值
        R_ACTUAL_ALT_SONAR.append(height_sonar)                 # 記錄超音波高度計值
        R_ACTUAL_ALT_BAROM_RAW.append(uav.location.global_relative_frame.alt)  # 記錄氣壓高度計原始值
        R_ACTUAL_ALT_BAROM_OFFSET.append(height_raw_barom)      # 記錄補正值
        R_ACTUAL_ALT_BAROM_AVE.append(height_barom_ave)         # 記錄均值濾波值
        R_ACTUAL_ALT_BAROM_KF.append(height_barom)              # 記錄KF值
        R_ACTUAL_ALT.append(ACTUAL_ALT)                         # 記錄總高度值

        # 同捆執行終點
        THREAD_OCCUPY.release()

        time.sleep(1 / FREQ_SENSOR)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_SENSOR = False
    msg_window.insert(1.0, '<關閉程序> 高度感測器：關閉\n')


# ========== 超音波高度計 ==========
# 使用HC-SR04為超音波感測模組，發出一脈波訊號使模組執行量測後傳回經過時間長的脈波
# 實際使用上雜訊相對多(可能受飛行時噪音影響)，故施加多重濾波條件：
# 超時防止(防止過長的無效等待)、角度補正、突波保護、均值濾波、卡爾曼濾波
# 經處理後的值堪用度大幅提高，對吸音面也能有效處理，使高度控制器能穩定運作
def sonar():
    global ACTUAL_ALT_SONAR, STATE_SENSOR_SONAR, SONAR_ERR_COUNT
    # 初始化腳位
    GPIO.setmode(GPIO.BCM)                 # 設定為BCM模式
    GPIO.setwarnings(False)                # 忽略警告
    GPIO.setup(SONAR_TRIG_PIN, GPIO.OUT)   # TRIG腳位輸出設定
    GPIO.setup(SONAR_ECHO_PIN, GPIO.IN)    # ECHO腳位輸入設定
    GPIO.output(SONAR_TRIG_PIN, False)     # 預設輸出為0

    # 脈波變化時間值變數
    pulse_start = time.time()              # 脈波起始時間
    pulse_end = time.time()                # 脈波結束時間

    # 卡爾曼濾波器初始化
    x_o = 0                                # 前一次狀態
    p_o = 0                                # 前一次斜方差
    z_o = 0                                # 前一次量測值
    q = math.exp(-SONAR_KF_Q)              # 斜方差噪音值
    r = 2.92 * math.exp(-3)                # 斜方差量測噪音值(定值不更改)

    # 平均降噪設定
    height_register = []
    height_ave = 0

    # 執行超音波高度計校正程序
    sonar_calibrate()

    # 回報超音波高度計啟用
    STATE_SONAR = True
    msg_window.insert(1.0, '<啟動程序> 超音波高度計：啟動\n')

    # 程式迴圈
    while STATE_SYSTEM:
        # 重設超時旗標
        timeout = True

        # 卡爾曼濾波-預測
        x_p = x_o
        p_p = p_o + q

        # 量測迴圈(線程優先執行，超時防止)
        THREAD_OCCUPY.acquire()
        while timeout:
            # 腳位初始化(因有報錯記錄，於每次前重新初始化)
            GPIO.setmode(GPIO.BCM)                  # 設定為BCM模式
            GPIO.setwarnings(False)                 # 忽略警告
            GPIO.setup(SONAR_TRIG_PIN, GPIO.OUT)    # TRIG腳位輸出設定

            # 輸出啟用脈波
            GPIO.output(SONAR_TRIG_PIN, True)
            time.sleep(0.00001)                     # 發出10us脈波
            GPIO.output(SONAR_TRIG_PIN, False)
            timeout_start = time.time()             # 記錄超時基準時間

            # 接收計時脈波
            # 於低電位迴圈等待，跳出時記錄脈波起始時間
            while not GPIO.input(SONAR_ECHO_PIN):
                pulse_start = time.time()             # 記錄脈波起始時間
                timeout = False                       # 成功記錄脈波時間，切換旗標
                if pulse_start - timeout_start > SONAR_TIMEOUT:
                    timeout = True                    # 等待接收時發生超時，跳出迴圈
                    break

            # 於高電位迴圈等待，跳出時記錄脈波結束時間(若發生超時則不執行本迴圈)
            while GPIO.input(SONAR_ECHO_PIN) and not timeout:
                pulse_end = time.time()               # 記錄脈波結束時間
                timeout = False                       # 成功記錄脈波時間，切換旗標
                if pulse_end - pulse_start > SONAR_TIMEOUT:
                    timeout = True                    # 接收時長發生超時，跳出迴圈
                    break

            # 若發生超時則顯示於訊息(可停用)
            if timeout:
                SONAR_ERR_COUNT += 1               # 超時次數計(統計用)
                # print('<錯誤> 超音波高度計：量測超時')

        # 計算高度值
        pulse_duration = pulse_end - pulse_start
        height_raw = (pulse_duration * (331.3 + 0.606 * SONAR_AIR_TEMP)) / 2

        # 高度角度補正
        roll_angle = uav.attitude.roll
        pitch_angle = uav.attitude.pitch
        height_offset = (height_raw * math.cos(roll_angle) * math.cos(pitch_angle)) + SONAR_OFFSET
        height_offset_r = height_offset     # 記錄用值

        # 上下降突波防止
        if ACTUAL_ALT > SONAR_FILTER_ALT:
            if math.fabs(height_offset-z_o) > SONAR_MAX_CHANGE:
                height_offset = height_ave

        # 均值濾波器(使線條滑順)
        height_register.insert(0, height_offset)
        if len(height_register) > SONAR_SAMPLE_NUM_AVE:
            height_register.pop()
        height_ave = np.average(height_register)       # 依據取樣次數計算即時平均值(會產生響應延遲)

        # 卡爾曼濾波器輸入限制(防止突波)
        if height_ave <= SONAR_KF_INPUT_MAX:
            z = height_ave       # 若低於限制則使用新值
        else:
            z = z_o              # 若超出限制則使用舊值

        # 卡爾曼濾波-更新
        k = p_p / (p_p + r)      # 卡爾曼增益
        x = x_p + k * (z - x_p)  # 狀態(濾波輸出值)
        p = (1 - k) * p_p        # 斜方差

        # 更新高度值至全域變數
        ACTUAL_ALT_SONAR = x

        # 傳遞值給下次迴圈使用
        x_o = x
        p_o = p
        z_o = z

        # 記錄數值
        R_ACTUAL_ALT_SONAR_RAW.append(height_raw)           # 記錄原始值
        R_ACTUAL_ALT_SONAR_OFFSET.append(height_offset_r)   # 記錄補正值(突波防止前)
        R_ACTUAL_ALT_SONAR_AVE.append(height_ave)           # 記錄平均值
        R_ACTUAL_ALT_SONAR_KF.append(ACTUAL_ALT_SONAR)      # 記錄KF值

        # 結束優先執行
        THREAD_OCCUPY.release()

        # 迴圈執行間隔
        time.sleep(1 / FREQ_SONAR)

    # 當系統裝態為否時，跳出迴圈並關閉函式
    STATE_SONAR = False
    msg_window.insert(1.0, '<關閉程序> 超音波高度計：關閉；量測超時次數共 %d 次\n' % SONAR_ERR_COUNT)


# ========== 命令傳輸程式 ==========
# 根據Ardupilot與MavLink協定規範，僅能使用set_attitude_target命令
# 另DroneKit套件提供函式封裝上也是依據上述規範編碼，故所使用的命令包括：
# 姿態四元數、各軸角速度、推力(昇降率)、操作模式(角度、角速度)
# 姿態四元數將控制器的角度值傳至函式做計算後返還、角速度採徑度制
# 根據測試及參考習慣操作，故選用姿態控制但於Yaw軸採角速度控制，符合手控習慣
# 流程上為將各控制器所輸出的命令值做統一編碼封裝後傳送
def command_transfer():
    global STATE_CMD

    # 回報命令傳輸程式啟動
    STATE_CMD = True
    msg_window.insert(1.0, '<啟動程序> 命令傳輸程式：啟動\n')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 命令編碼
        # 模式一：使用角度控制
        if CMD_MSG == 1:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # 開機時間
                0,  # 目標系統
                0,  # 目標裝置
                0b00000111,  # 命令遮罩(1忽略，0啟用)
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # 姿態四元數
                math.radians(TARGET_ROLL_RATE),  # Roll滾轉角速度(radian/s)
                math.radians(TARGET_PITCH_RATE),  # Pitch滾轉角速度(radian/s)
                math.radians(TARGET_YAW_RATE),  # Yaw滾轉角速度(radian/s)
                TARGET_THRUST)  # 推力升降比(0~1，中點0.5)

        # 模式二：使用偏航角度補正
        elif CMD_MSG == 2:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # 開機時間
                0,  # 目標系統
                0,  # 目標裝置
                0b00000111,  # 命令遮罩(1忽略，0啟用)
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW + TARGET_YAW_OFFSET),  # 姿態四元數
                math.radians(TARGET_ROLL_RATE),  # Roll滾轉角速度(radian/s)
                math.radians(TARGET_PITCH_RATE),  # Pitch滾轉角速度(radian/s)
                math.radians(TARGET_YAW_RATE),  # Yaw滾轉角速度(radian/s)
                TARGET_THRUST)  # 推力升降比(0~1，中點0.5)

        # 模式三：使用偏航角速度控制(餘採姿態控制)
        else:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # 開機時間
                0,  # 目標系統
                0,  # 目標裝置
                0b00000011,  # 命令遮罩(1忽略，0啟用)
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # 姿態四元數
                math.radians(TARGET_ROLL_RATE),  # Roll滾轉角速度(radian/s)
                math.radians(TARGET_PITCH_RATE),  # Pitch滾轉角速度(radian/s)
                math.radians(TARGET_YAW_RATE),  # Yaw滾轉角速度(radian/s)
                TARGET_THRUST)  # 推力升降比(0~1，中點0.5)

        # 傳輸命令
        uav.send_mavlink(msg)

        # 同捆執行終點
        THREAD_OCCUPY.release()

        # 迴圈執行間隔
        time.sleep(1 / FREQ_CMD)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_CMD = False
    msg_window.insert(1.0, '<關閉程序> 命令傳輸程式：關閉\n')


# ========== 影像擷取程式 ==========
# 負責截取機載影像畫面，儲存至全域值供控制器使用
# 因主線程衝突故將畫面顯示整合至GUI介面中方便瀏覽
# 另將飛行影像存成影片以供記錄(處理後之影像若需錄下，須經格式轉換處理，暫不增加)
def camera():
    global CAM_IMAGE, STATE_CAM, STATE_SYSTEM
    # 程式迴圈(同捆執行模式)
    while CAM_VIDEO.isOpened() and STATE_SYSTEM:
        # 確認影像擷取狀態
        state_grab = CAM_VIDEO.grab()

        # 擷取成功
        if state_grab:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 擷取影像並儲存
            state_retrieve, image = CAM_VIDEO.retrieve()
            CAM_IMAGE = cv2.resize(image, (CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT))
            VIDEO_SAVE.write(CAM_IMAGE)

            # 回報影像擷取程式啟動
            if not STATE_CAM:
                STATE_CAM = True
                msg_window.insert(1.0, '<啟動程序> 影像擷取程式：啟動\n')

            # 顯示影像視窗(CV外顯版，須於主程序使用，因會與GUI衝突不啟用，但程式保留)
            # if MODE == 'POS' or MODE == 'LINE':
            #    cv2.imshow("Flight Camera", CAM_IMAGE_LIVE)
            # else:
            # cv2.imshow("Flight Camera", CAM_IMAGE)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 手動關閉影像顯示
            if cv2.waitKey(int(1000/FREQ_CAM)) & 0xff == ord("q"):
                STATE_SYSTEM = False
                msg_window.insert(1.0, '<關閉程序> 已手動關閉相機，請手動重啟程式\n')
                break

        # 擷取失敗
        else:
            msg_window.insert(1.0, '<錯誤> 影像擷取程式：無法擷取影像\n')
            STATE_SYSTEM = False
            msg_window.insert(1.0, '<嚴重錯誤> 請檢查相機並手動重啟程式!!!!\n')
            break

    # 當系統狀態為否時，跳出迴圈並關閉函式
    CAM_VIDEO.release()
    VIDEO_SAVE.release()
    # cv2.destroyAllWindows()                # 關閉影像顯示
    STATE_CAM = False
    msg_window.insert(1.0, '<關閉程序> 影像擷取程式：關閉\n')


# ====================================================================
# ========== 子函式區塊 ================================================
# ====================================================================
# 本區放置各主函式下會使用到的功能函式，分有校正類、計算類、裝備控制類
# 將常使用的重複功能函式化以節省程式大小(僅限無傳遞值類的函式)

# ========== 超音波高度計校正程序 ==========
# 於啟動前自動校正高度，為避免單次量測不準確，故以多次取樣後平均為基準來補正
def sonar_calibrate():
    global SONAR_OFFSET
    # 平均降噪設定
    height_register = []

    # 脈波變化時間值變數
    pulse_start = time.time()             # 脈波起始時間
    pulse_end = time.time()               # 脈波結束時間

    # 量測迴圈(線程優先執行)
    THREAD_OCCUPY.acquire()
    for i in range(0, SONAR_SAMPLE_NUM_CAL):
        # 設定超時旗標
        timeout = True

        # 量測迴圈(超時防止)
        while timeout:
            # 腳位初始化(因有報錯記錄，於每次前重新初始化)
            GPIO.setmode(GPIO.BCM)                      # 設定為BCM模式
            GPIO.setwarnings(False)                     # 忽略警告
            GPIO.setup(SONAR_TRIG_PIN, GPIO.OUT)        # TRIG腳位輸出設定

            # 輸出啟用脈波
            GPIO.output(SONAR_TRIG_PIN, True)
            time.sleep(0.00001)                          # 發出10us脈波
            GPIO.output(SONAR_TRIG_PIN, False)
            timeout_start = time.time()                  # 記錄超時基準時間

            # 接收計時脈波
            # 於低電位迴圈等待，跳出時記錄脈波起始時間
            while not GPIO.input(SONAR_ECHO_PIN):
                pulse_start = time.time()                # 記錄脈波起始時間
                timeout = False                          # 成功記錄脈波時間，切換旗標
                if pulse_start - timeout_start > SONAR_TIMEOUT:
                    timeout = True                       # 等待接收時發生超時，跳出迴圈
                    break

            # 於高電位迴圈等待，跳出時記錄脈波結束時間(若發生超時則不執行本迴圈)
            while GPIO.input(SONAR_ECHO_PIN) and not timeout:
                pulse_end = time.time()                  # 記錄脈波結束時間
                timeout = False                          # 成功記錄脈波時間，切換旗標
                if pulse_end - pulse_start > SONAR_TIMEOUT:
                    timeout = True                       # 接收時長發生超時，跳出迴圈
                    break

            # 若發生超時則顯示於訊息
            if timeout:
                # print('<錯誤> 超音波感測器：量測超時')
                pass

        # 計算高度值
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * SONAR_AIR_TEMP)) / 2

        # 均值濾波器
        height_register.append(height)

        # 等待10ms執行下次量測
        time.sleep(0.1)

    # 結束優先執行
    THREAD_OCCUPY.release()

    # 更新高度校正偏移量
    SONAR_OFFSET = -np.average(height_register)
    msg_window.insert(1.0, '<啟動程序> 超音波高度計校正完畢，偏移 %.3f m\n' % SONAR_OFFSET)


# ========== 氣壓高度計校正程序 ==========
# 因氣壓計會有波動(但也可能IMU震盪與積分誤差影響)，需做補正
def barometer_calibrate():
    global BAROM_OFFSET
    # 平均降噪設定
    height_register = []

    # 校正氣壓計
    uav.send_calibrate_barometer()

    # 量測迴圈(線程優先執行)
    THREAD_OCCUPY.acquire()
    for i in range(0, BAROM_SAMPLE_NUM_CAL):
        # 讀取氣壓計高度值
        height = uav.location.global_relative_frame.alt

        # 均值濾波器
        height_register.append(height)

        # 等待200ms執行下次量測
        time.sleep(0.2)

    # 結束優先執行
    THREAD_OCCUPY.release()

    # 更新高度校正偏移量
    BAROM_OFFSET = -np.average(height_register)
    msg_window.insert(1.0, '<啟動程序> 氣壓高度計校正完畢，偏移 %.3f m\n' % BAROM_OFFSET)


# ========== 偏航角校正程序 ==========
# 因無人機羅盤會有飄移現象(發生情形不定)，以及機頭朝向非正北方
# 為方便於姿態控制時以機頭為正北，故先施加補正以方便後續操控(適用MSG_CMD模式二，但程式皆帶入執行)
def yaw_calibrate():
    global TARGET_YAW_OFFSET
    # 更新偏航角校正偏移量
    TARGET_YAW_OFFSET = -math.degrees(uav.attitude.yaw)
    msg_window.insert(1.0, '<啟動程序> 偏航角校正完畢，偏移 %.3f degree\n' % TARGET_YAW_OFFSET)


# ========== 轉換尤拉角到四元數 ==========
# 因命令格式針對姿態需使用四元數格式，因此需先把尤拉角作轉換，此格式可避免萬向節鎖產生
# 輸入格式為度(degree)，輸出為四元數(q,x,y,z)
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


# ========== 影像計算程序 ==========
# 為計算當禎影像中，目標顏色的型心位置、色塊面積大小，主要供位置控制器、飛行條件判斷使用
# 為防止雜訊可設置最低面積值，因此若色塊大小在此之下將無法被偵測到
# 處理流程為將影像先依目標顏色做過濾，再依此做型心、面積運算
def img_calculate(color):
    global CAM_IMAGE_LIVE
    # 色域轉換BGR->HSV
    image_hsv = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2HSV)
    # 閉運算
    image_close = cv2.morphologyEx(image_hsv, cv2.MORPH_CLOSE,
                                   np.ones(IMG_MORPHOLOGY_EX_KERNEL, np.uint8))

    # 依照目標色抓該顏色輪廓
    if color == 'red':          # 紅色
        image_mask = cv2.inRange(image_close, IMG_RED_L, IMG_RED_H)
    elif color == 'green':      # 綠色
        image_mask = cv2.inRange(image_close, IMG_GREEN_L, IMG_GREEN_H)
    elif color == 'blue':       # 藍色
        image_mask = cv2.inRange(image_close, IMG_BLUE_L, IMG_BLUE_H)
    else:                       # 黑色(預設顏色)
        image_mask = cv2.inRange(image_close, IMG_BLACK_L, IMG_BLACK_H)

    # 輸出影像(GUI顯示用)
    if MODE == 'POS':
        CAM_IMAGE_LIVE = image_mask

    # 取得影像、輪廓點與索引
    image_c, contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)

    # 設定型心暫存列表
    moment_x_register = []
    moment_y_register = []
    area_register = []

    # 計算型心與面積
    for c in contours:
        # 計算色塊面積
        block_area = cv2.contourArea(c)

        # 面積符合使用門檻方取用(降噪處理)
        if block_area >= IMG_AREA_LIMIT:
            # 取得型心位置(左上為原點，加法增量)
            moment = cv2.moments(c)
            if moment["m00"] != 0:
                moment_x_raw = int(moment["m10"] / moment["m00"])
                moment_y_raw = int(moment["m01"] / moment["m00"])
            else:
                moment_x_raw = 0
                moment_y_raw = 0

            # 計算色塊位置(相對於畫面中心)
            block_x_offset = moment_x_raw - BODY_REF_X - BODY_OFFSET_X
            block_y_offset = BODY_REF_Y - moment_y_raw - BODY_OFFSET_Y

            moment_x_register.append(block_x_offset)
            moment_y_register.append(block_y_offset)
            area_register.append(block_area)

    # 設定型心運算暫存列表
    moment_area_x = 0
    moment_area_y = 0
    area_sum = 0

    # 計算面積一次矩
    for i in range(len(moment_x_register)):
        moment_area_x += moment_x_register[i] * area_register[i]
        moment_area_y += moment_y_register[i] * area_register[i]
        area_sum += area_register[i]

    # 求得組合型心位置
    if len(moment_x_register) != 0:
        block_x = moment_area_x / area_sum
        block_y = moment_area_y / area_sum
    else:
        block_x = 0
        block_y = 0

    return block_x, block_y, area_sum


# ========== 影像二值化與型心計算 ==========
# 將影像做二值化後再做型心與面積計算，除影響去用過程與畫面中心設定點不同外，其餘與前函式相同
# 但因中間有程式碼不同，為避免增加樹狀複雜度，故不再做副程式呼叫
# 本函式供航向控制器使用，僅回傳型心位置值，面積閥值亦獨立設定
def img_moment():
    global CAM_IMAGE_LIVE
    # 色域轉換BGR->GRAY
    img_gray = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    img_blur = cv2.GaussianBlur(img_gray, ANG_GB_KERNEL_SIZE, ANG_GB_SIGMA)
    # 二值化
    ret, img_th = cv2.threshold(img_blur, ANG_THRESHOLD,
                                ANG_THRESHOLD_NEW, cv2.THRESH_BINARY_INV)
    # 取得影像、輪廓點與索引
    image_c, contours, hierarchy = cv2.findContours(img_th, cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)

    # 輸出影像
    if MODE == 'LINE':
        CAM_IMAGE_LIVE = img_th

    # 設定型心暫存列表
    moment_x_register = []
    moment_y_register = []
    area_register = []

    # 計算型心與面積
    for c in contours:
        # 計算色塊面積
        block_area = cv2.contourArea(c)

        # 面積符合使用門檻方取用(降噪處理)
        if block_area >= ANG_LINE_TRACK_AREA_L:
            # 取得型心位置(左上為原點，加法增量)
            moment = cv2.moments(c)
            if moment["m00"] != 0:
                moment_x_raw = int(moment["m10"] / moment["m00"])
                moment_y_raw = int(moment["m01"] / moment["m00"])
            else:
                moment_x_raw = 0
                moment_y_raw = 0

            # 計算色塊位置(相對於畫面下方中心)
            block_x_offset = moment_x_raw - BODY_REF_X - BODY_OFFSET_X
            block_y_offset = 2 * BODY_REF_Y - moment_y_raw - BODY_OFFSET_Y

            moment_x_register.append(block_x_offset)
            moment_y_register.append(block_y_offset)
            area_register.append(block_area)

    # 設定型心運算暫存列表
    moment_area_x = 0
    moment_area_y = 0
    area_sum = 0

    # 計算面積一次矩
    for i in range(len(moment_x_register)):
        moment_area_x += moment_x_register[i] * area_register[i]
        moment_area_y += moment_y_register[i] * area_register[i]
        area_sum += area_register[i]

    # 求得組合型心位置
    if len(moment_x_register) != 0:
        block_x = moment_area_x / area_sum
        block_y = moment_area_y / area_sum
    else:
        block_x = 0
        block_y = 0

    return block_x, block_y


# ========== 角度計算 ==========
# 本函式供計算型心點與原點連線和畫面Y軸夾角計算，於航向控制器使用
# 可解決平行線夾角為零的問題，所得的計算值相當於空氣誤差值為本式特點，整合目標與回授
def angle_calculate(block_x, block_y):
    # 計算與中心線夾角
    if block_x == 0:
        angle = 0
    elif block_y == 0 and block_x > 0:
        angle = 90
    elif block_y == 0 and block_x < 0:
        angle = -90
    else:
        angle = math.degrees(math.atan(block_x/block_y))

    return angle


# ========= 沙包盒關門 ==========
# 於起飛時送出脈波訊號固定艙門，防止貨物掉出，並維持較佳的空氣動力學外型
def box_close():
    global STATE_BOX
    # 初始化腳位
    GPIO.setmode(GPIO.BCM)                      # 設定為BCM模式
    GPIO.setwarnings(False)                     # 忽略警告
    GPIO.setup(BOX_PWM_PIN, GPIO.OUT)           # PWM腳位輸出設定
    box = GPIO.PWM(BOX_PWM_PIN, BOX_PWM_FREQ)   # 建立PWM控制物件

    # 啟動PWM控制，使門維持關閉
    dc = BOX_PWM_FREQ * (BOX_PWM_CLOSE / 1000000) * 100
    box.start(dc)

    # 回報艙門狀態
    STATE_BOX = True
    msg_window.insert(1.0, '<飛行狀態> 沙包盒艙門：關閉\n')


# ======== 沙包盒開門 ==========
# 輸出訊號使艙門開啟，使貨物卸出
def box_open():
    global STATE_BOX
    # 初始化腳位
    GPIO.setmode(GPIO.BCM)                      # 設定為BCM模式
    GPIO.setwarnings(False)                     # 忽略警告
    GPIO.setup(BOX_PWM_PIN, GPIO.OUT)           # PWM腳位輸出設定
    box = GPIO.PWM(BOX_PWM_PIN, BOX_PWM_FREQ)   # 建立PWM控制物件

    # 啟動PWM控制，使門打開
    dc = BOX_PWM_FREQ * (BOX_PWM_OPEN / 1000000) * 100
    box.start(dc)

    # 回報艙門狀態
    STATE_BOX = False
    msg_window.insert(1.0, '<飛行狀態> 沙包盒艙門：開啟\n')


# ====================================================================
# ========== 飛行程序指令(條件與命令) ====================================
# ====================================================================
# 任務中會使用的區段飛行命令、模式切換命令、觸發條件

# ========== 設定為定位模式 ==========
# 切換至定位模式，並輸入目標顏色來做目標色塊
def pos_mode(color):
    global MODE, POS_COLOR
    MODE = 'POS'                # 模式旗標切為定位
    POS_COLOR = color           # 定位目標顏色設置
    # 回報模式啟用狀態
    msg_window.insert(1.0, '<飛行狀態> 切換至定位模式，顏色追蹤：%s\n' % POS_COLOR)


# ========== 設定為循跡模式 ==========
def line_mode():
    global MODE, POS_COLOR, TARGET_PITCH
    MODE = 'LINE'               # 模式旗標切為循跡
    POS_COLOR = ''              # 清空顏色目標
    TARGET_PITCH = -5           # 以固定傾角往前飛行
    # 回報模式啟用狀態
    msg_window.insert(1.0, '<飛行狀態> 切換至循跡模式，開始往前飛行\n')


# ========= 顏色抵達偵測 =========
# 條件式，佔用目前程序，直到偵測抵達目標顏色為止，需在循跡模式中使用
def detect_arrive(color):
    global STATE_STAGE
    # 初始化顏色面積
    img_area = 0

    # 色塊面積偵測
    while img_area < IMG_AREA_LIMIT:
        x_raw, y_raw, img_area = img_calculate(color)
        msg_window.insert(1.0, '<飛行狀態> 循跡中，差角%.2f度，角速度%.2f度/秒\n' % (ACTUAL_HDG_ERR, TARGET_YAW_RATE))
        time.sleep(1)

    # 回報偵測到顏色
    STATE_STAGE += 1
    msg_window.insert(1.0, '<飛行狀態> 抵達%s色塊上方\n' % color)


# ========== 顏色離開偵測 ==========
# 條件式，佔用目前程序，直到偵測到離開目標色塊為止，需在循跡模式中使用
# 為避免發生離開後瞬間偵測抵達情形發生，故將面積閥值拉高作預防
def detect_leave(color):
    # 初始化顏色面積
    img_area = 100000

    # 色塊面積偵測
    while img_area > IMG_AREA_LIMIT * 1.1:
        x_raw, y_raw, img_area = img_calculate(color)
        msg_window.insert(1.0, '<飛行狀態> 循跡中，差角%.2f度，角速度%.2f度/秒\n' % (ACTUAL_HDG_ERR, TARGET_YAW_RATE))
        time.sleep(1)

    # 回報離開的顏色
    msg_window.insert(1.0, '<飛行狀態> 已離開%s色塊上方\n' % color)


# ========== 啟動前檢查 ==========
# 本程式為飛行前的安全檢查項目，除確認控制器是否上線外，也提醒控制員檢查安全程序
# 項目有：安全裝置移除(槳片銷)、起飛區淨空，完成後即交付起飛控制
def pre_check():
    global STATE_STAGE
    # 執行起飛前檢查
    msg_window.insert(1.0, '<啟動程序> 啟動前檢查程序：啟動\n')

    # 階段一：控制器檢查
    # 等待高度控制器啟動
    if not STATE_ALT:
        msg_window.insert(1.0, '<啟動程序> 主程序：等待高度控制器啟動\n')
    while not STATE_ALT:
        time.sleep(0.1)

    # 等待位置控制器啟動
    if not STATE_POS:
        msg_window.insert(1.0, '<啟動程序> 主程序：等待位置控制器啟動\n')
    while not STATE_POS:
        time.sleep(0.1)

    # 等待航向控制器啟動
    if not STATE_HDG:
        msg_window.insert(1.0, '<啟動程序> 主程序：等待航向控制器啟動\n')
    while not STATE_HDG:
        time.sleep(0.1)

    # 等待命令傳輸啟動
    if not STATE_CMD:
        msg_window.insert(1.0, '<啟動程序> 主程序：等待命令傳輸啟動\n')
    while not STATE_CMD:
        time.sleep(0.1)

    # 階段二：艙門關閉
    # 初始化沙包盒，並關閉艙門
    box_close()
    time.sleep(1)

    # 階段三：安全檢查程序
    # 確認安全裝置解除
    safe_pin = False
    while not safe_pin:
        safe_pin = messagebox.askyesno('<啟動程序>', '請確認安全裝置已移除')
    msg_window.insert(1.0, '<啟動程序> 安全裝置已移除\n')
    time.sleep(1)

    # 確認起飛區已淨空
    lunch_clear = False
    while not lunch_clear:
        lunch_clear = messagebox.askyesno('<啟動程序>', '請確認起飛區已淨空')
    msg_window.insert(1.0, '<啟動程序> 起飛區已淨空\n')

    # 回報起飛前檢點完成
    msg_window.insert(1.0, '<啟動程序> 啟動前檢查程序：完成\n')
    time.sleep(1)

    # 任務開始確認
    go_or_nogo = False
    while not go_or_nogo:
        go_or_nogo = messagebox.askyesno('<啟動程序>', '請求任務開始')

    # 回報任務執行開始
    STATE_STAGE += 1
    msg_window.insert(1.0, '<啟動程序> TDK飛行任務，開始!\n')
    time.sleep(1)


# ========== 起飛程序 ==========
# 執行模式切換、無人機解鎖，並由高度控制器將無人機送至目標飛行高度
# 過程搭配定位程序使用，因起飛區線佈置關係，會使定位前偏，為正常現象
def take_off(target_alt):
    global TARGET_ALT, STATE_STAGE
    # 切換至無衛星導航模式
    uav.mode = VehicleMode("GUIDED_NOGPS")
    while not uav.mode.name == 'GUIDED_NOGPS':
        time.sleep(0.5)
    msg_window.insert(1.0, '<啟動程序> 無人機已切換至無衛星導航模式\n')

    # 解鎖無人機
    uav.arm(wait=True)
    msg_window.insert(1.0, '<啟動程序> 無人機已解鎖\n')

    # 設定為定位模式(定位黑色，會因賽道而偏前，為容許情形)
    pos_mode('black')

    # 設定目標高度
    msg_window.insert(1.0, '<飛行命令> 起飛至目標高度 %.2f m\n' % target_alt)
    TARGET_ALT = target_alt

    # 等待抵達目標高度
    while ACTUAL_ALT < TARGET_ALT * (TAKEOFF_SETTLE / 100):
        msg_window.insert(1.0, '<飛行狀態> 起飛中，目前高度 %.2f m，距離目標高度尚有 %.2f m\n' %
                          (ACTUAL_ALT, TARGET_ALT-ACTUAL_ALT))
        msg_window.insert(1.0, '<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' %
                          (ACTUAL_POS[0], ACTUAL_POS[1]))
        time.sleep(1)

    # 回報抵達目標高度
    STATE_STAGE += 1
    msg_window.insert(1.0, '<飛行狀態> 已抵達目標高度 %.2f m\n' % TARGET_ALT)


# ========== 轉向控制 ==========
# 抵達號誌區後，進行燈號等待，並於偵測到顏色變化後做紀錄，並作相對應轉向
def turn():
    global TARGET_YAW_RATE, MISSION_COLOR, POS_COLOR, STATE_STAGE
    # 轉彎等待與執行迴圈
    while 1:      # 藍右，綠左
        # 讀取顏色面積值做判斷
        block_x, block_y, image_area_blue = img_calculate('blue')
        block_x, block_y, image_area_green = img_calculate('green')
        msg_window.insert(1.0, '<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' %
                          (ACTUAL_POS[0], ACTUAL_POS[1]))

        # 讀取偏航角
        yaw_start = math.degrees(uav.attitude.yaw)
        yaw_current = math.degrees(uav.attitude.yaw)

        # 若偵測到藍色
        if image_area_blue >= IMG_AREA_LIMIT:
            # 記錄任務顏色並更改定位顏色
            MISSION_COLOR = 'blue'
            POS_COLOR = MISSION_COLOR
            msg_window.insert(1.0, '<飛行狀態> 偵測到藍色為任務顏色\n')

            # 右轉
            msg_window.insert(1.0, '<飛行命令> 往右旋轉90度\n')
            TARGET_YAW_RATE = TURN_YAW_RATE
            while math.fabs(yaw_current - yaw_start) < 90 * (TURN_SETTLE / 100):
                yaw_current = math.degrees(uav.attitude.yaw)
            TARGET_YAW_RATE = 0
            break

        # 若偵測到綠色
        elif image_area_green >= IMG_AREA_LIMIT:
            # 記錄任務顏色並更改定位顏色
            MISSION_COLOR = 'green'
            POS_COLOR = MISSION_COLOR
            msg_window.insert(1.0, '<飛行狀態> 偵測到綠色為任務顏色\n')

            # 左轉
            msg_window.insert(1.0, '<飛行命令> 往左旋轉90度\n')
            TARGET_YAW_RATE = -TURN_YAW_RATE
            while math.fabs(yaw_current - yaw_start) < 90 * (TURN_SETTLE / 100):
                yaw_current = math.degrees(uav.attitude.yaw)
            TARGET_YAW_RATE = 0
            break

        time.sleep(0.5)

    # 回報轉向完成
    STATE_STAGE += 1
    msg_window.insert(1.0, '<飛行狀態> 轉向完成\n')


# ========== 空投程序 ==========
# 於投擲區上方做定位穩定，並執行空投程序
def drop():
    global STATE_STAGE
    # 切換定位模式
    pos_mode(MISSION_COLOR)

    # 定位迴圈，等待移至空投點上方
    msg_window.insert(1.0, '<飛行命令> 移動至空投點上方\n')
    while 1:
        # 取得目前型心位置
        block_x, block_y, image_area = img_calculate(MISSION_COLOR)
        msg_window.insert(1.0, '<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' % (ACTUAL_POS[0], ACTUAL_POS[1]))

        # 判斷是否已抵達中心
        if -DROP_POS_LIMIT_X < block_x < DROP_POS_LIMIT_X:
            if -DROP_POS_LIMIT_Y < block_y < DROP_POS_LIMIT_Y:
                time.sleep(1)
                break

        time.sleep(0.5)

    # 打開沙包盒空投
    msg_window.insert(1.0, '<飛行命令> 位置已確認，執行空投\n')
    box_open()
    time.sleep(2)

    # 回報空投成功
    STATE_STAGE += 1
    msg_window.insert(1.0, '<飛行狀態> 沙包已成功空投\n')

    # 關閉沙包盒
    box_close()


# ========== 降落程序 ==========
# 抵達降落區中心後，切換至定位模式追尋目標色塊降落
# 當抵達目標色塊上方後，改追中心黑點，防止原色塊降落時發生飽和無作用情形
# 於最後執行結束關閉程序
def landing():
    global TARGET_ALT, POS_COLOR, STATE_SYSTEM, STATE_STAGE
    # 切換定位模式
    pos_mode(MISSION_COLOR)

    # 定位迴圈，等待移至降落點上方
    msg_window.insert(1.0, '<飛行命令> 移動至降落點上方\n')
    while 1:
        # 取得目前型心位置
        block_x, block_y, image_area = img_calculate(MISSION_COLOR)
        msg_window.insert(1.0, '<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' % (ACTUAL_POS[0], ACTUAL_POS[1]))

        # 判斷是否已抵達中心
        if -LANDING_POS_LIMIT_X < block_x < LANDING_POS_LIMIT_X:
            if -LANDING_POS_LIMIT_Y < block_y < LANDING_POS_LIMIT_Y:
                break
        time.sleep(0.5)

    # 設定目標高度與定位顏色
    msg_window.insert(1.0, '<飛行命令> 位置已確認，開始降落至地面\n')
    TARGET_ALT = -0.1           # 負值以確保不會著地後彈升
    POS_COLOR = 'black'         # 改追黑色防止降落超出
    msg_window.insert(1.0, '<飛行狀態> 切換顏色追蹤：%s\n' % POS_COLOR)

    # 等待抵達目標高度
    while ACTUAL_ALT > LANDING_CUTOFF:
        msg_window.insert(1.0, '<飛行狀態> 降落中，目前高度 %.2f m\n' % ACTUAL_ALT)
        msg_window.insert(1.0, '<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' % (ACTUAL_POS[0], ACTUAL_POS[1]))
        time.sleep(1)

    # 回報降落成功
    STATE_STAGE += 1
    msg_window.insert(1.0, '<飛行狀態> 已降落地面\n')

    # 返回手動模式
    uav.mode = VehicleMode("STABILIZE")
    while not uav.mode.name == 'STABILIZE':
        time.sleep(0.5)
    msg_window.insert(1.0, '<關閉程序> 無人機已切換至手動模式\n')

    # 結束與無人機連線
    uav.close()
    msg_window.insert(1.0, '<關閉程序> 無人機連線關閉\n')

    # 清空GPIO設定
    GPIO.cleanup()
    msg_window.insert(1.0, '<關閉程序> GPIO已關閉\n')


# ========== 任務主函式 ==========
# 任務程序，採SFC式PLC撰寫程序
def mission():
    global STATE_SYSTEM, STATE_INFO, STATE_THREAD, MODE
    # 回報主程式啟動
    msg_window.insert(1.0, '<啟動程序> 任務主程序：啟動\n')

    # 啟動前檢查
    pre_check()

    # 飛行流程
    # 起飛與紅燈區
    take_off(1.2)                       # 起飛
    line_mode()                         # 往前循跡飛行
    detect_arrive('red')                # 偵測抵達紅燈區
    pos_mode('red')                     # 紅燈上定位

    # 循線A區
    turn()                              # 偵測到顏色後轉向
    line_mode()                         # 往前循跡飛行
    detect_leave(MISSION_COLOR)         # 偵測已離開號誌區

    # 投擲與循線B區
    detect_arrive(MISSION_COLOR)        # 偵測抵達投擲色塊上方
    drop()                              # 空投程序
    line_mode()                         # 往前循跡飛行
    detect_leave(MISSION_COLOR)         # 偵測已離開投擲區

    # T路口與降落區
    detect_arrive(MISSION_COLOR)        # 偵測抵達降落區
    landing()                           # 執行降落

    # 關閉背景多線程
    STATE_SYSTEM = False
    STATE_THREAD = False
    MODE = ''
    time.sleep(2)
    msg_window.insert(1.0, '<關閉程序> 主程序：關閉\n')

    # 關閉資訊更新
    STATE_INFO = False
    msg_window.insert(1.0, '<關閉程序> 任務完成!\n')


# ========== 資訊更新程式 ==========
# 更新GUI畫面上資訊
def info_update():
    print('<啟動程序> 資訊更新程式：啟動')
    while STATE_INFO:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 2-1 狀態更新
        if STATE_SYSTEM:
            state_sys.configure(fg='green', text='[ON]')
        else:
            state_sys.configure(fg='red', text='[OFF]')

        if STATE_THREAD:
            state_thr.configure(fg='green', text='[ON]')
        else:
            state_thr.configure(fg='red', text='[OFF]')

        if MODE == 'LINE':
            state_mode.configure(fg='green', text='[循跡]')
        elif MODE == 'POS':
            state_mode.configure(fg='green', text='[定位]')
        else:
            state_mode.configure(fg='red', text='[無]')

        if STATE_ALT:
            state_alt.configure(fg='green', text='[ON]')
        else:
            state_alt.configure(fg='red', text='[OFF]')

        if STATE_POS:
            state_pos.configure(fg='green', text='[ON]')
        else:
            state_pos.configure(fg='red', text='[OFF]')

        if STATE_HDG:
            state_hdg.configure(fg='green', text='[ON]')
        else:
            state_hdg.configure(fg='red', text='[OFF]')

        if STATE_CMD:
            state_cmd.configure(fg='green', text='[ON]')
        else:
            state_cmd.configure(fg='red', text='[OFF]')

        if STATE_CAM:
            state_cam.configure(fg='green', text='[ON]')
        else:
            state_cam.configure(fg='red', text='[OFF]')

        if STATE_SENSOR:
            state_alsen.configure(fg='green', text='[ON]')
        else:
            state_alsen.configure(fg='red', text='[OFF]')

        if STATE_SENSOR_SONAR:
            state_sonar.configure(fg='green', text='[ON]')
        else:
            state_sonar.configure(fg='red', text='[OFF]')

        if STATE_BOX:
            state_box.configure(fg='green', text='[ON]')
        else:
            state_box.configure(fg='red', text='[OFF]')

        # 3-1 姿態更新
        info_roll.configure(text='%.1f' % math.degrees(uav.attitude.roll))
        info_pitch.configure(text='%.1f' % math.degrees(uav.attitude.pitch))
        info_yaw.configure(text=int(math.degrees(uav.attitude.yaw)))
        info_hdg.configure(text=uav.heading)
        info_alt.configure(text='%.2f' % ACTUAL_ALT)
        info_alt_sonar.configure(text='%.2f' % ACTUAL_ALT_SONAR)
        info_alt_barom.configure(text='%.2f' % ACTUAL_ALT_BAROM)

        # 3-2 任務更新
        if STATE_STAGE == 1:
            info_mission.configure(text='[起飛]', fg='black')
        elif STATE_STAGE == 2:
            info_mission.configure(text='[前往號誌區]')
        elif STATE_STAGE == 3:
            info_mission.configure(text='[等待任務燈號]')
        elif STATE_STAGE == 4:
            info_mission.configure(text='[前往投擲區]')
        elif STATE_STAGE == 5:
            info_mission.configure(text='[執行空投]')
        elif STATE_STAGE == 6:
            info_mission.configure(text='[前往降落區]')
        elif STATE_STAGE == 7:
            info_mission.configure(text='[降落]')
        elif STATE_STAGE == 8:
            info_mission.configure(text='[任務完成]', fg='green')

        # 4-1 高度控制器更新
        alt_target.configure(text='%.2f' % TARGET_ALT)
        alt_actual.configure(text='%.2f' % ACTUAL_ALT)
        alt_thrust.configure(text=int((TARGET_THRUST * 2 - 1) * 100))

        # 4-2位置控制器更新
        if MISSION_COLOR == 'green':
            pos_mis_color.configure(fg='green', text='綠色')
        elif MISSION_COLOR == 'blue':
            pos_mis_color.configure(fg='blue', text='藍色')
        else:
            pos_mis_color.configure(fg='black', text='無')

        if POS_COLOR == 'green':
            pos_color.configure(fg='green', text='綠色')
        elif POS_COLOR == 'blue':
            pos_color.configure(fg='blue', text='藍色')
        elif POS_COLOR == 'red':
            pos_color.configure(fg='red', text='紅色')
        else:
            pos_color.configure(fg='black', text='黑色')

        pos_actual.configure(text=('(%d,%d)' % (ACTUAL_POS[0], ACTUAL_POS[1])))
        pos_roll.configure(text=int(TARGET_ROLL))
        pos_pitch.configure(text=int(TARGET_PITCH))

        # 4-3航向控制器
        hdg_error.configure(text=int(ACTUAL_HDG_ERR))
        hdg_yaw_rate.configure(text=int(TARGET_YAW_RATE))
        hdg_roll.configure(text=int(TARGET_ROLL))

        # 5 影像更新
        if STATE_CAM:
            if MODE == 'POS' or MODE == 'LINE':
                cam_process = Image.fromarray(CAM_IMAGE_LIVE)
                cam_process = ImageTk.PhotoImage(cam_process)
            else:
                cam_process = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2RGBA)
                cam_process = Image.fromarray(cam_process)
                cam_process = ImageTk.PhotoImage(cam_process)

            cam_image.configure(image=cam_process)

        # GUI畫面更新
        flight_monitor.update()

        # 同捆執行終點
        THREAD_OCCUPY.release()

        time.sleep(1 / FREQ_INFO)

    # 資訊更新已停止
    msg_window.insert(1.0, '<關閉程序> 資訊更新程式：關閉\n')


# 多線程函式
def thread_procedure1():
    global STATE_THREAD
    # 定義多線程
    sonar_sen = threading.Thread(target=sonar)
    alt_sen = threading.Thread(target=altitude_sensor)
    alt_ctrl = threading.Thread(target=altitude_controller)
    cam = threading.Thread(target=camera)
    pos_ctrl = threading.Thread(target=position_controller)
    hdg_ctrl = threading.Thread(target=heading_controller)
    cmd = threading.Thread(target=command_transfer)

    # 執行多線程(依啟動順序排放)
    sonar_sen.start()
    alt_sen.start()
    alt_ctrl.start()
    cam.start()
    pos_ctrl.start()
    hdg_ctrl.start()
    cmd.start()

    # 回報多線程啟動
    STATE_THREAD = True
    msg_window.insert(1.0, '<啟動程序> 多線程程序：啟動\n')


# 任務啟動用函式
def thread_procedure2():
    # 定義多線程
    m = threading.Thread(target=mission)

    # 執行多線程
    m.start()


# 記錄輸出用函式
def thread_procedure3():
    f = threading.Thread(target=show_data)
    f.start()
    f.join()


# ========== 數值紀錄區塊 ================================================
def show_data():
    THREAD_OCCUPY.acquire()
    # ======高度控制器================
    # 高度控制器響應圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT_CTRL) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT_CTRL, label="Actual Altitude")
    plt.plot(num_alt, R_TARGET_ALT, label="Target Altitude")
    plt.plot(num_alt, R_TARGET_THRUST, label="Climb Rate")

    plt.xlabel('Data number')
    plt.ylabel('Height(m), Velocity(m/s)')
    plt.title('Altitude Controller Data')
    plt.legend()
    plt.savefig('Altitude Controller Data.png')
    plt.show()

    # 高度控制器PID輸出圖
    num_alt = []
    for i in range(1, len(R_ALT_ERROR) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ALT_ERROR, label="Error")
    plt.plot(num_alt, R_ALT_GP, label="Gain P")
    plt.plot(num_alt, R_ALT_GI, label="Gain I")
    plt.plot(num_alt, R_ALT_GD, label="Gain D")

    plt.xlabel('Data number')
    plt.ylabel('Height(m), Velocity(m/s)')
    plt.title('Altitude Controller PID Output Data')
    plt.legend()
    plt.savefig('Altitude Controller PID Output Data.png')
    plt.show()

    # ======位置控制器================
    # 位置控制器X響應圖
    num_pos = []
    for i in range(1, len(R_BLOCK_X) + 1):
        num_pos.append(i)

    plt.plot(num_pos, R_BLOCK_X_RAW, label="X Position - Raw")
    plt.plot(num_pos, R_BLOCK_X, label="X Position")
    plt.plot(num_pos, R_TARGET_ROLL, label="Target Roll")
    plt.plot(num_pos, R_ACTUAL_ROLL, label="Actual Roll")
    plt.plot(num_pos, R_BLOCK_AREA, label="Block Area")

    plt.xlabel('Data number')
    plt.ylabel('Position(px), Angle(degree)')
    plt.title('Position Controller Data - X axis')
    plt.legend()
    plt.savefig('Position Controller Data - X axis.png')
    plt.show()

    # 位置控制器Y響應圖
    plt.plot(num_pos, R_BLOCK_Y_RAW, label="Y Position - Raw")
    plt.plot(num_pos, R_BLOCK_Y, label="Y Position")
    plt.plot(num_pos, R_TARGET_PITCH, label="Target Pitch")
    plt.plot(num_pos, R_ACTUAL_PITCH, label="Actual pitch")
    plt.plot(num_pos, R_BLOCK_AREA, label="Block Area")

    plt.xlabel('Data number')
    plt.ylabel('Position(px), Angle(degree)')
    plt.title('Position Controller Data - Y axis')
    plt.legend()
    plt.savefig('Position Controller Data - Y axis.png')
    plt.show()

    # 位置控制器軌跡響應圖
    plt.plot(R_BLOCK_X, R_BLOCK_Y, label="Block Position")
    plt.plot(R_TARGET_ROLL, R_TARGET_PITCH, label="Target Attitude")
    plt.plot(R_ACTUAL_ROLL, R_ACTUAL_PITCH, label="Actual Attitude")

    plt.xlabel('X-Position(px), Angle(degree)')
    plt.ylabel('Y-Position(px), Angle(degree)')
    plt.title('Position Controller Track Data')
    plt.legend()
    plt.savefig('Position Controller Track Data.png')
    plt.show()

    # ======航向控制器===========================================
    # 航向控制器響應圖
    num_hdg = []
    for i in range(1, len(R_HDG_ERROR) + 1):
        num_hdg.append(i)

    plt.plot(num_hdg, R_HDG_ERROR, label="Heading Error")
    plt.plot(num_hdg, R_TARGET_YAW_RATE, label="Target Yaw Rate")
    plt.plot(num_hdg, R_HDG_TARGET_ROLL, label="Target Roll")

    plt.xlabel('Data number')
    plt.ylabel('Angle(degree), Angular Rate(degree/s)')
    plt.title('Heading Controller Data')
    plt.legend()
    plt.savefig('Heading Controller Data - .png')
    plt.show()

    # 航向控制器PID響應圖
    plt.plot(num_hdg, R_HDG_ERROR, label="Heading Error")
    plt.plot(num_hdg, R_HDG_GP, label="Gain P")
    plt.plot(num_hdg, R_HDG_GI, label="Gain I")
    plt.plot(num_hdg, R_HDG_GD, label="Gain D")

    plt.xlabel('Data number')
    plt.ylabel('Angle(degree), Angular Rate(degree/s)')
    plt.title('Heading Controller PID Data')
    plt.legend()
    plt.savefig('Heading Controller PID Data.png')
    plt.show()

    # ======高度感測器================================
    # 高度感測器數值處理圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT, label="Actual Height")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR, label="Sonar Data")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_KF, label="Barometer Data")

    plt.xlabel('Data number')
    plt.ylabel('Height(m)')
    plt.title('Altitude Sensor Data')
    plt.legend()
    plt.savefig('Altitude Sensor Data.png')
    plt.show()

    # 超音波高度計數值處理圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT_SONAR_KF) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_RAW, label="Raw Data")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_OFFSET, label="Angle & Height Offset")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_AVE, label="Average Filter")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_KF, label="Kalman Filter")

    plt.xlabel('Data number')
    plt.ylabel('Height(m)')
    plt.title('Altitude Data - Sonar')
    plt.legend()
    plt.savefig('Altitude Data - Sonar.png')
    plt.show()

    # 氣壓高度計數值處理圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT_BAROM_RAW) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_RAW, label="Raw Data")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_OFFSET, label="Height Offset")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_AVE, label="Average Filter")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_KF, label="Kalman Filter")

    plt.xlabel('Data number')
    plt.ylabel('Height(m)')
    plt.title('Altitude Data - Barometer')
    plt.legend()
    plt.savefig('Altitude Data - Barometer.png')
    plt.show()
    THREAD_OCCUPY.release()


# ========== GUI介面區塊 ================================================
# 圖形化介面建構
flight_monitor = tk.Tk()
flight_monitor.title('TDK 24th 飛行組 C08 台灣大艦隊 飛行控制介面')
flight_monitor.geometry('1920x1030')
flight_monitor.configure(background='#eff8fd')

# 1 標題區
row_1_frame = tk.Frame(flight_monitor, height=80, width=1920, bg='#eff8fd')
row_1_frame.pack_propagate(0)
row_1_frame.pack(side=tk.TOP)
head_label = tk.Label(row_1_frame, font=('', 30), fg='#004085', bg='#eff8fd',
                      text='\n----- TDK 24th 飛行組 C08 台灣大艦隊 飛行控制介面 -----\n')
head_label.pack(side=tk.TOP)

# 2 系統狀態列
row_2_frame = tk.Frame(flight_monitor, height=210, width=1920, bg='#eff8fd')
row_2_frame.pack_propagate(0)
row_2_frame.pack(side=tk.TOP)

# 系統與控制器狀態
state_frame = tk.Frame(row_2_frame, bg='#eff8fd')
state_frame.pack(side=tk.TOP)
state_label = tk.Label(state_frame, font=('', 25), bg='#eff8fd',
                       fg='#007dd1', text='----- 系統與控制器狀態 -----')
state_label.pack(side=tk.TOP)

# 系統控制器
state_sys_frame = tk.Frame(state_frame, height=150, width=130, bg='#eff8fd')
state_sys_frame.pack_propagate(0)
state_sys_frame.pack(side=tk.LEFT)
state_sys_label = tk.Label(state_sys_frame, font=('', 25), text='SYS\n系統', bg='#eff8fd')
state_sys_label.pack(side=tk.TOP)
state_sys = tk.Label(state_sys_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_sys.pack(side=tk.TOP)

# 模式控制器
state_mode_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_mode_frame.pack_propagate(0)
state_mode_frame.pack(side=tk.LEFT)
state_mode_label = tk.Label(state_mode_frame, font=('', 25), text='MODE\n模式', bg='#eff8fd')
state_mode_label.pack(side=tk.TOP)
state_mode = tk.Label(state_mode_frame, fg='red', font=('', 35), text='[無]', bg='#eff8fd')
state_mode.pack(side=tk.TOP)

# 線程控制器
state_thr_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_thr_frame.pack_propagate(0)
state_thr_frame.pack(side=tk.LEFT)
state_thr_label = tk.Label(state_thr_frame, font=('', 25), text='THR\n多線程', bg='#eff8fd')
state_thr_label.pack(side=tk.TOP)
state_thr = tk.Label(state_thr_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_thr.pack(side=tk.TOP)

# 高度控制器
state_alt_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_alt_frame.pack_propagate(0)
state_alt_frame.pack(side=tk.LEFT)
state_alt_label = tk.Label(state_alt_frame, font=('', 25), text='ALT\n高度', bg='#eff8fd')
state_alt_label.pack(side=tk.TOP)
state_alt = tk.Label(state_alt_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_alt.pack(side=tk.TOP)

# 位置控制器
state_pos_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_pos_frame.pack_propagate(0)
state_pos_frame.pack(side=tk.LEFT)
state_pos_label = tk.Label(state_pos_frame, font=('', 25), text='POS\n位置', bg='#eff8fd')
state_pos_label.pack(side=tk.TOP)
state_pos = tk.Label(state_pos_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_pos.pack(side=tk.TOP)

# 航向控制器
state_hdg_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_hdg_frame.pack_propagate(0)
state_hdg_frame.pack(side=tk.LEFT)
state_hdg_label = tk.Label(state_hdg_frame, font=('', 25), text='HDG\n航向', bg='#eff8fd')
state_hdg_label.pack(side=tk.TOP)
state_hdg = tk.Label(state_hdg_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_hdg.pack(side=tk.TOP)

# 命令控制器
state_cmd_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_cmd_frame.pack_propagate(0)
state_cmd_frame.pack(side=tk.LEFT)
state_cmd_label = tk.Label(state_cmd_frame, font=('', 25), text='CMD\n命令', bg='#eff8fd')
state_cmd_label.pack(side=tk.TOP)
state_cmd = tk.Label(state_cmd_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_cmd.pack(side=tk.TOP)

# 相機控制器
state_cam_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_cam_frame.pack_propagate(0)
state_cam_frame.pack(side=tk.LEFT)
state_cam_label = tk.Label(state_cam_frame, font=('', 25), text='CAM\n相機', bg='#eff8fd')
state_cam_label.pack(side=tk.TOP)
state_cam = tk.Label(state_cam_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_cam.pack(side=tk.TOP)

# 高度感測器
state_alsen_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_alsen_frame.pack_propagate(0)
state_alsen_frame.pack(side=tk.LEFT)
state_alsen_label = tk.Label(state_alsen_frame, font=('', 25), text='ALSEN\n高度計', bg='#eff8fd')
state_alsen_label.pack(side=tk.TOP)
state_alsen = tk.Label(state_alsen_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_alsen.pack(side=tk.TOP)

# 超音波感測器
state_sonar_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_sonar_frame.pack_propagate(0)
state_sonar_frame.pack(side=tk.LEFT)
state_sonar_label = tk.Label(state_sonar_frame, font=('', 25), text='SONAR\n超音波', bg='#eff8fd')
state_sonar_label.pack(side=tk.TOP)
state_sonar = tk.Label(state_sonar_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_sonar.pack(side=tk.TOP)

# 沙包盒艙門
state_box_frame = tk.Frame(state_frame, height=150, width=150, bg='#eff8fd')
state_box_frame.pack_propagate(0)
state_box_frame.pack(side=tk.LEFT)
state_box_label = tk.Label(state_box_frame, font=('', 25), text='BOX\n艙門', bg='#eff8fd')
state_box_label.pack(side=tk.TOP)
state_box = tk.Label(state_box_frame, fg='red', font=('', 35), text='[OFF]', bg='#eff8fd')
state_box.pack(side=tk.TOP)

# 3 即時飛行資料
row_3_frame = tk.Frame(flight_monitor, height=210, width=1920, bg='#eff8fd')
row_3_frame.pack_propagate(0)
row_3_frame.pack(side=tk.TOP)

# 即時飛行資料
info_frame = tk.Frame(row_3_frame, bg='#eff8fd')
info_frame.pack(side=tk.LEFT)
info_label = tk.Label(info_frame, font=('', 25), text='----- 即時飛行資料 -----', bg='#eff8fd', fg='#007dd1')
info_label.pack(side=tk.TOP)

# 滾轉
info_roll_frame = tk.Frame(info_frame, height=150, width=150, bg='#eff8fd')
info_roll_frame.pack_propagate(0)
info_roll_frame.pack(side=tk.LEFT)
info_roll_label = tk.Label(info_roll_frame, font=('', 25), text='Roll', bg='#eff8fd')
info_roll_label.pack(side=tk.TOP)
info_roll = tk.Label(info_roll_frame, font=('', 35), bg='#eff8fd')
info_roll.pack(side=tk.TOP)
info_roll_unit = tk.Label(info_roll_frame, font=('', 15), text='Degree', bg='#eff8fd')
info_roll_unit.pack(side=tk.TOP)

# 俯仰
info_pitch_frame = tk.Frame(info_frame, height=150, width=150, bg='#eff8fd')
info_pitch_frame.pack_propagate(0)
info_pitch_frame.pack(side=tk.LEFT)
info_pitch_label = tk.Label(info_pitch_frame, font=('', 25), text='Pitch', bg='#eff8fd')
info_pitch_label.pack(side=tk.TOP)
info_pitch = tk.Label(info_pitch_frame, font=('', 35), bg='#eff8fd')
info_pitch.pack(side=tk.TOP)
info_pitch_unit = tk.Label(info_pitch_frame, font=('', 15), text='Degree', bg='#eff8fd')
info_pitch_unit.pack(side=tk.TOP)

# 偏航
info_yaw_frame = tk.Frame(info_frame, height=150, width=150, bg='#eff8fd')
info_yaw_frame.pack_propagate(0)
info_yaw_frame.pack(side=tk.LEFT)
info_yaw_label = tk.Label(info_yaw_frame, font=('', 25), text='Yaw', bg='#eff8fd')
info_yaw_label.pack(side=tk.TOP)
info_yaw = tk.Label(info_yaw_frame, font=('', 35), bg='#eff8fd')
info_yaw.pack(side=tk.TOP)
info_yaw_unit = tk.Label(info_yaw_frame, font=('', 15), text='Degree', bg='#eff8fd')
info_yaw_unit.pack(side=tk.TOP)

# 航向
info_hdg_frame = tk.Frame(info_frame, height=150, width=150, bg='#eff8fd')
info_hdg_frame.pack_propagate(0)
info_hdg_frame.pack(side=tk.LEFT)
info_hdg_label = tk.Label(info_hdg_frame, font=('', 25), text='HDG', bg='#eff8fd')
info_hdg_label.pack(side=tk.TOP)
info_hdg = tk.Label(info_hdg_frame, font=('', 35), bg='#eff8fd')
info_hdg.pack(side=tk.TOP)
info_hdg_unit = tk.Label(info_hdg_frame, font=('', 15), text='Degree', bg='#eff8fd')
info_hdg_unit.pack(side=tk.TOP)

# 高度
info_alt_frame = tk.Frame(info_frame, height=150, width=150, bg='#eff8fd')
info_alt_frame.pack_propagate(0)
info_alt_frame.pack(side=tk.LEFT)
info_alt_label = tk.Label(info_alt_frame, font=('', 25), text='ALT', bg='#eff8fd')
info_alt_label.pack(side=tk.TOP)
info_alt = tk.Label(info_alt_frame, font=('', 35), bg='#eff8fd')
info_alt.pack(side=tk.TOP)
info_alt_unit = tk.Label(info_alt_frame, font=('', 15), text='Meter', bg='#eff8fd')
info_alt_unit.pack(side=tk.TOP)

# 超音波高度計
info_alt_sonar_frame = tk.Frame(info_frame, height=150, width=150, bg='#eff8fd')
info_alt_sonar_frame.pack_propagate(0)
info_alt_sonar_frame.pack(side=tk.LEFT)
info_alt_sonar_label = tk.Label(info_alt_sonar_frame, font=('', 25), text='Sonar', bg='#eff8fd')
info_alt_sonar_label.pack(side=tk.TOP)
info_alt_sonar = tk.Label(info_alt_sonar_frame, font=('', 35), bg='#eff8fd')
info_alt_sonar.pack(side=tk.TOP)
info_alt_sonar_unit = tk.Label(info_alt_sonar_frame, font=('', 15), text='Meter', bg='#eff8fd')
info_alt_sonar_unit.pack(side=tk.TOP)

# 氣壓高度計
info_alt_barom_frame = tk.Frame(info_frame, height=150, width=150, bg='#eff8fd')
info_alt_barom_frame.pack_propagate(0)
info_alt_barom_frame.pack(side=tk.LEFT)
info_alt_barom_label = tk.Label(info_alt_barom_frame, font=('', 25), text='Barom', bg='#eff8fd')
info_alt_barom_label.pack(side=tk.TOP)
info_alt_barom = tk.Label(info_alt_barom_frame, font=('', 35), bg='#eff8fd')
info_alt_barom.pack(side=tk.TOP)
info_alt_barom_unit = tk.Label(info_alt_barom_frame, font=('', 15), text='Meter', bg='#eff8fd')
info_alt_barom_unit.pack(side=tk.TOP)

# 起飛開關
button_frame = tk.Frame(row_3_frame, bg='#eff8fd')
button_frame.pack(side=tk.LEFT)
button_label = tk.Label(button_frame, font=('', 25), text='----- 起飛開關 -----',
                        fg='#007dd1', bg='#eff8fd')
button_label.pack(side=tk.TOP)

# 多線程啟動
button_thread_frame = tk.Frame(button_frame, bg='#eff8fd')
button_thread_frame.pack(side=tk.LEFT)
button_thread_label = tk.Label(button_thread_frame, font=('', 17), text='1\n  多線程程序  \n', bg='#eff8fd')
button_thread_label.pack(side=tk.TOP)
button_thread = tk.Button(button_thread_frame, font=('', 13),
                          text='啟 動', width=5, height=2, command=thread_procedure1)
button_thread.pack(side=tk.TOP)

# 任務啟動
button_mission_frame = tk.Frame(button_frame, bg='#eff8fd')
button_mission_frame.pack(side=tk.LEFT)
button_mission_label = tk.Label(button_mission_frame, font=('', 17), text='2\n  任務程序  \n', bg='#eff8fd')
button_mission_label.pack(side=tk.TOP)
button_mission = tk.Button(button_mission_frame, font=('', 13),
                           text='啟 動', width=5, height=2, command=thread_procedure2)
button_mission.pack(side=tk.TOP)

# 飛行資料
button_mission_frame = tk.Frame(button_frame, bg='#eff8fd')
button_mission_frame.pack(side=tk.RIGHT)
button_mission_label = tk.Label(button_mission_frame, font=('', 17), text='3\n  飛行資料  \n', bg='#eff8fd')
button_mission_label.pack(side=tk.TOP)
button_mission = tk.Button(button_mission_frame, font=('', 13),
                           text='取 得', width=5, height=2, command=thread_procedure3)
button_mission.pack(side=tk.TOP)

# 任務階段
info_mission_frame = tk.Frame(row_3_frame, height=210, width=400, bg='#eff8fd')
info_mission_frame.pack_propagate(0)
info_mission_frame.pack(side=tk.LEFT)

info_mission_label = tk.Label(info_mission_frame, font=('', 25), text='\n--- 任務階段 ---',
                              fg='#007dd1', bg='#eff8fd')
info_mission_label.pack(side=tk.TOP)
info_mission = tk.Label(info_mission_frame, font=('', 30), text='[飛行前檢查]', bg='#eff8fd')
info_mission.pack(side=tk.TOP)


# 相機處理框
cam_frame = tk.Frame(flight_monitor, width=500, height=500, bg='#eff8fd')
cam_frame.pack_propagate(0)
cam_frame.pack(side=tk.RIGHT, padx=5, pady=5)
cam_headline = tk.Label(cam_frame, font=('', 20), text='----- 影像處理畫面 -----',
                        fg='#007dd1', bg='#eff8fd')
cam_headline.pack(side=tk.TOP)
cam_image = tk.Label(cam_frame)
cam_image.pack(side=tk.TOP)

# 4 控制器資訊
row_4_frame = tk.Frame(flight_monitor, height=210, width=1920, bg='#eff8fd')
row_4_frame.pack_propagate(0)
row_4_frame.pack(side=tk.TOP)

# 高度控制器
alt_frame = tk.Frame(row_4_frame, bg='#eff8fd')
alt_frame.pack(side=tk.LEFT)
alt_label = tk.Label(alt_frame, font=('', 25), text='----- 高度控制器 -----', fg='#007dd1', bg='#eff8fd')
alt_label.pack(side=tk.TOP)

# 目標高
alt_target_frame = tk.Frame(alt_frame, height=150, width=150, bg='#eff8fd')
alt_target_frame.pack_propagate(0)
alt_target_frame.pack(side=tk.LEFT)
alt_target_label = tk.Label(alt_target_frame, font=('', 20), text='目標高度', bg='#eff8fd')
alt_target_label.pack(side=tk.TOP)
alt_target = tk.Label(alt_target_frame, font=('', 35), bg='#eff8fd')
alt_target.pack(side=tk.TOP)
alt_target_unit = tk.Label(alt_target_frame, font=('', 15), text='Meter', bg='#eff8fd')
alt_target_unit.pack(side=tk.TOP)

# 實際高度
alt_actual_frame = tk.Frame(alt_frame, height=150, width=150, bg='#eff8fd')
alt_actual_frame.pack_propagate(0)
alt_actual_frame.pack(side=tk.LEFT)
alt_actual_label = tk.Label(alt_actual_frame, font=('', 20), text='實際高度', bg='#eff8fd')
alt_actual_label.pack(side=tk.TOP)
alt_actual = tk.Label(alt_actual_frame, font=('', 35), bg='#eff8fd')
alt_actual.pack(side=tk.TOP)
alt_actual_unit = tk.Label(alt_actual_frame, font=('', 15), text='Meter', bg='#eff8fd')
alt_actual_unit.pack(side=tk.TOP)

# 輸出推力
alt_thrust_frame = tk.Frame(alt_frame, height=150, width=150, bg='#eff8fd')
alt_thrust_frame.pack_propagate(0)
alt_thrust_frame.pack(side=tk.LEFT)
alt_thrust_label = tk.Label(alt_thrust_frame, font=('', 20), text='升降速率', bg='#eff8fd')
alt_thrust_label.pack(side=tk.TOP)
alt_thrust = tk.Label(alt_thrust_frame, font=('', 35), bg='#eff8fd')
alt_thrust.pack(side=tk.TOP)
alt_thrust_unit = tk.Label(alt_thrust_frame, font=('', 15), text='%', bg='#eff8fd')
alt_thrust_unit.pack(side=tk.TOP)

# 位置控制器
pos_frame = tk.Frame(row_4_frame, bg='#eff8fd')
pos_frame.pack(side=tk.LEFT)
pos_label = tk.Label(pos_frame, font=('', 25), text='------------- 位置控制器 -------------',
                     fg='#007dd1', bg='#eff8fd')
pos_label.pack(side=tk.TOP)

# 
pos_mis_space1_frame = tk.Frame(pos_frame, height=150, width=70, bg='#eff8fd')
pos_mis_space1_frame.pack_propagate(0)
pos_mis_space1_frame.pack(side=tk.LEFT)
pos_mis_space1_label = tk.Label(pos_mis_space1_frame, font=('', 20), bg='#eff8fd')
pos_mis_space1_label.pack(side=tk.TOP)
pos_mis_space1 = tk.Label(pos_mis_space1_frame, font=('', 35), bg='#eff8fd')
pos_mis_space1.pack(side=tk.TOP)

# 任務顏色
pos_mis_color_frame = tk.Frame(pos_frame, height=150, width=150, bg='#eff8fd')
pos_mis_color_frame.pack_propagate(0)
pos_mis_color_frame.pack(side=tk.LEFT)
pos_mis_color_label = tk.Label(pos_mis_color_frame, font=('', 20), text='任務顏色', bg='#eff8fd')
pos_mis_color_label.pack(side=tk.TOP)
pos_mis_color = tk.Label(pos_mis_color_frame, font=('', 35), text=MISSION_COLOR, bg='#eff8fd')
pos_mis_color.pack(side=tk.TOP)

# 定位顏色
pos_color_frame = tk.Frame(pos_frame, height=150, width=150, bg='#eff8fd')
pos_color_frame.pack_propagate(0)
pos_color_frame.pack(side=tk.LEFT)
pos_color_label = tk.Label(pos_color_frame, font=('', 20), text='定位顏色', bg='#eff8fd')
pos_color_label.pack(side=tk.TOP)
pos_color = tk.Label(pos_color_frame, font=('', 35), text=POS_COLOR, bg='#eff8fd')
pos_color.pack(side=tk.TOP)

# 型心位置
pos_actual_frame = tk.Frame(pos_frame, height=150, width=250, bg='#eff8fd')
pos_actual_frame.pack_propagate(0)
pos_actual_frame.pack(side=tk.LEFT)
pos_actual_label = tk.Label(pos_actual_frame, font=('', 20), text='型心位置', bg='#eff8fd')
pos_actual_label.pack(side=tk.TOP)
pos_actual = tk.Label(pos_actual_frame, font=('', 35), text=('(%d,%d)' % (ACTUAL_POS[0], ACTUAL_POS[1])), bg='#eff8fd')
pos_actual.pack(side=tk.TOP)
pos_actual_unit = tk.Label(pos_actual_frame, font=('', 15), text='( X , Y )', bg='#eff8fd')
pos_actual_unit.pack(side=tk.TOP)

# 滾轉
pos_roll_frame = tk.Frame(pos_frame, height=150, width=150, bg='#eff8fd')
pos_roll_frame.pack_propagate(0)
pos_roll_frame.pack(side=tk.LEFT)
pos_roll_label = tk.Label(pos_roll_frame, font=('', 20), text='Tar-Roll', bg='#eff8fd')
pos_roll_label.pack(side=tk.TOP)
pos_roll = tk.Label(pos_roll_frame, font=('', 35), text=TARGET_ROLL, bg='#eff8fd')
pos_roll.pack(side=tk.TOP)
pos_roll_unit = tk.Label(pos_roll_frame, font=('', 15), text='Degree', bg='#eff8fd')
pos_roll_unit.pack(side=tk.TOP)

# 俯仰
pos_pitch_frame = tk.Frame(pos_frame, height=150, width=150, bg='#eff8fd')
pos_pitch_frame.pack_propagate(0)
pos_pitch_frame.pack(side=tk.LEFT)
pos_pitch_label = tk.Label(pos_pitch_frame, font=('', 20), text='Tar-Pitch', bg='#eff8fd')
pos_pitch_label.pack(side=tk.TOP)
pos_pitch = tk.Label(pos_pitch_frame, font=('', 35), text=TARGET_PITCH, bg='#eff8fd')
pos_pitch.pack(side=tk.TOP)
pos_pitch_unit = tk.Label(pos_pitch_frame, font=('', 15), text='Degree', bg='#eff8fd')
pos_pitch_unit.pack(side=tk.TOP)

# 
pos_mis_space2_frame = tk.Frame(pos_frame, height=150, width=70, bg='#eff8fd')
pos_mis_space2_frame.pack_propagate(0)
pos_mis_space2_frame.pack(side=tk.LEFT)
pos_mis_space2_label = tk.Label(pos_mis_space2_frame, font=('', 20), bg='#eff8fd')
pos_mis_space2_label.pack(side=tk.TOP)
pos_mis_space2 = tk.Label(pos_mis_space2_frame, font=('', 35), bg='#eff8fd')
pos_mis_space2.pack(side=tk.TOP)

# 5 控制器資訊
row_5_frame = tk.Frame(flight_monitor, height=250, width=1920, bg='#eff8fd')
row_5_frame.pack_propagate(0)
row_5_frame.pack(side=tk.TOP)

# 航向控制器
hdg_frame = tk.Frame(row_5_frame, bg='#eff8fd')
hdg_frame.pack(side=tk.LEFT)
hdg_label = tk.Label(hdg_frame, font=('', 25), text='----- 航向控制器 -----',
                     fg='#007dd1', bg='#eff8fd')
hdg_label.pack(side=tk.TOP)

# 角度差
hdg_error_frame = tk.Frame(hdg_frame, height=150, width=150, bg='#eff8fd')
hdg_error_frame.pack_propagate(0)
hdg_error_frame.pack(side=tk.LEFT)
hdg_error_label = tk.Label(hdg_error_frame, font=('', 20), text='角度差', bg='#eff8fd')
hdg_error_label.pack(side=tk.TOP)
hdg_error = tk.Label(hdg_error_frame, font=('', 35), text=ACTUAL_HDG_ERR, bg='#eff8fd')
hdg_error.pack(side=tk.TOP)
hdg_error_unit = tk.Label(hdg_error_frame, font=('', 15), text='Degree', bg='#eff8fd')
hdg_error_unit.pack(side=tk.TOP)

# 角速度
hdg_yaw_rate_frame = tk.Frame(hdg_frame, height=150, width=150, bg='#eff8fd')
hdg_yaw_rate_frame.pack_propagate(0)
hdg_yaw_rate_frame.pack(side=tk.LEFT)
hdg_yaw_rate_label = tk.Label(hdg_yaw_rate_frame, font=('', 20), text='Tar-Y Rate', bg='#eff8fd')
hdg_yaw_rate_label.pack(side=tk.TOP)
hdg_yaw_rate = tk.Label(hdg_yaw_rate_frame, font=('', 35), text=TARGET_YAW_RATE, bg='#eff8fd')
hdg_yaw_rate.pack(side=tk.TOP)
hdg_yaw_rate_unit = tk.Label(hdg_yaw_rate_frame, font=('', 15), text='Degree/s', bg='#eff8fd')
hdg_yaw_rate_unit.pack(side=tk.TOP)

# 滾轉角
hdg_roll_frame = tk.Frame(hdg_frame, height=150, width=150, bg='#eff8fd')
hdg_roll_frame.pack_propagate(0)
hdg_roll_frame.pack(side=tk.LEFT)
hdg_roll_label = tk.Label(hdg_roll_frame, font=('', 20), text='Tar-Roll', bg='#eff8fd')
hdg_roll_label.pack(side=tk.TOP)
hdg_roll = tk.Label(hdg_roll_frame, font=('', 35), text=TARGET_ROLL, bg='#eff8fd')
hdg_roll.pack(side=tk.TOP)
hdg_roll_unit = tk.Label(hdg_roll_frame, font=('', 15), text='Degree', bg='#eff8fd')
hdg_roll_unit.pack(side=tk.TOP)

# 提示窗
msg_frame = tk.Frame(row_5_frame, bg='#eff8fd')
msg_frame.pack(side=tk.LEFT)
msg_label = tk.Label(msg_frame, font=('', 20), text='----- 系統狀態提示 -----\nSystem Information',
                     fg='#007dd1', bg='#eff8fd')
msg_label.pack(side=tk.TOP)
msg_window = tk.Text(msg_frame, font=('', 15), width=100, height='5')
msg_window.pack_propagate(0)
msg_window.pack(side=tk.TOP)
msg_window_scroll = tk.Scrollbar(msg_window, width=10)
msg_window.config(yscrollcommand=msg_window_scroll.set)
msg_window_scroll.pack(side=tk.RIGHT, fill=tk.Y)
msg_window_scroll.config(command=msg_window.yview)

# 參數更新線程啟動
iu = threading.Thread(target=info_update)
iu.start()

# ========== 程式執行區 ==================================================
flight_monitor.mainloop()
