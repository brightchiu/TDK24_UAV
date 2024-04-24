# 定位飛行程式
# 包含高度、定位控制器
# 預定0910測試

# 0909修改
# 新增來自相機定位、定高控制修正

# 0910測試結論
# 需做速度控制(慣性抵銷)
# 0911已更改，加入級距式補償(輸出問題未驗證)

# 0917定位控制改成PID控制器

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# 載入模組
from __future__ import print_function
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math
import cv2

# ======設定全域變數======
print('<啟動程序> 載入全域變數設定')

# UAV 通訊參數
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# ======旗標與變數======================================================
# 旗標
STATE_SYSTEM = True
STATE_ALT = False
STATE_POS = False
STATE_CMD = False
STATE_CAM = False
STATE_SENSOR_SONAR = False
STATE_SENSOR = False
MODE = ''                # 模式旗標

# 多線程優先執行
THREAD_OCCUPY = threading.Lock()

# 目標值(命令值)
TARGET_ALT = 0           # 目標高度，主程式輸入於高度控制器(m)
TARGET_ROLL = 0          # 目標滾轉角(degree)
TARGET_PITCH = 0         # 目標俯仰角(degree)
TARGET_YAW = 0           # 目標偏航角(degree)
TARGET_YAW_OFFSET = 0    # 目標偏航角補正值(degree)
TARGET_THRUST = 0        # 目標推力，表示升降速度(0.5為中點)
TARGET_ROLL_RATE = 0     # 滾轉角速度(degree/s)，一般不使用
TARGET_PITCH_RATE = 0    # 俯仰角速度(degree/s)，一般不使用
TARGET_YAW_RATE = 0      # 偏航角速度(degree/s)，需搭配控制器控制角度

# 實際值(量測值)
ACTUAL_ALT = 0           # 實際高度
ACTUAL_ALT_SONAR = 0     # 超音波高度計
ACTUAL_ALT_BAROM = 0     # 氣壓高度計
ACTUAL_POS = [0.0, 0.0]  # 色塊實際位置

# ======控制器參數設定===================================================
# 迴圈頻率(5Hz有滑順表現，測試可到30Hz)
FREQ_ALT = 30            # 高度控制器頻率(不快於高度感測器頻率)
FREQ_POS = 30            # 位置控制器頻率(不快於影像禎率)
FREQ_SONAR = 30          # 超音波感測器頻率(5~50Hz，越高雜訊越多)
FREQ_SENSOR = 30         # 高度感測器頻率(不快於超音波感測器頻率)
FREQ_CMD = 30            # 命令傳輸頻率(不快於控制器頻率)
FREQ_CAM = 30            # 影像禎率(fps，依系統負載調整)

# 高度控制器參數
ALT_KP = 1               # P增益值
ALT_KI = 0               # I增益值
ALT_KD = 0.1             # D增益值
ALT_DZ = 0.0             # 油門平飛安定區間(單向)

# 位置控制器參數
POS_COLOR = ''                      # 定位的色塊顏色
POS_SAMPLE_NUM_AVE = 10             # 色塊座標均值濾波取樣次數
POS_DZ_IN_X = 10                    # 色塊定位作動忽略區間(單向，px)
POS_DZ_IN_Y = POS_DZ_IN_X*(3/4)     # 色塊定位作動忽略區間(單向，px)
POS_DZ_OUT_X = 200                  # 外側範圍，固定輸出下限(單向，px)
POS_DZ_OUT_Y = POS_DZ_OUT_X*(3/4)   # 外側範圍，固定輸出下限(單向，px)
POS_ANGLE_LIMIT = 5                 # 定位控制最大飛行角度(degree)

# 命令傳輸程式參數
CMD_MSG = 3              # 命令編碼模式(1:角度 2:角度補正 3:角速度)

# 起降程序參數
TAKEOFF_SETTLE = 0.95         # 安定高度百分比(%)
LANDING_CUTOFF = 0.05         # 油門關閉高度(m)

# ======影像參數設定=============================================================
# 相機設定參數
CAM_VIDEO = cv2.VideoCapture(0)     # 影像來源(預設來源為0)
CAM_IMAGE = 0                       # 儲存影像
CAM_IMAGE_WEIGHT = 480              # 影像寬度(px，4:3設定 )
CAM_IMAGE_HEIGHT = 360              # 影像高度(px)
VIDEO_FOURCC = cv2.VideoWriter_fourcc(*'XVID')
VIDEO_SAVE = cv2.VideoWriter('Position Controller Data.avi',
                             VIDEO_FOURCC, FREQ_CAM, (CAM_IMAGE_WEIGHT, CAM_IMAGE_HEIGHT))

# 影像計算參數
IMG_BLUE_L = np.array([85, 76, 90])        # 藍色色域-下限
IMG_BLUE_H = np.array([127, 255, 255])     # 藍色色域-上限
IMG_GREEN_L = np.array([44, 104, 27])      # 綠色色域-下限
IMG_GREEN_H = np.array([87, 255, 255])     # 綠色色域-上限
IMG_RED_L = np.array([160, 111, 56])       # 紅色色域-下限
IMG_RED_H = np.array([180, 255, 255])      # 紅色色域-上限
IMG_BLACK_L = np.array([0, 0, 0])          # 黑色色域-下限
IMG_BLACK_H = np.array([180, 255, 75])     # 黑色色域-上限
IMG_MORPHOLOGY_EX_KERNEL = (5, 5)          # 閉運算矩陣大小(越大降噪能力越強)
IMG_AREA_LIMIT = 3000                      # 取用面積下限值
BODY_REF_X = int(CAM_IMAGE_WEIGHT / 2)     # 畫面參考X座標中心
BODY_REF_Y = int(CAM_IMAGE_HEIGHT / 2)     # 畫面位置Y座標中心
BODY_OFFSET_X = 0                          # 機身中心X偏移量(px，左負右正)
BODY_OFFSET_Y = 0                          # 機身中心Y偏移量(px，上正下負)

# ======高度感測器參數=============================================================
# ->高度感測器整合參數
BAROM_SAMPLE_NUM_AVE = 10    # 均值濾波取樣次數*
WEIGHTED_SONAR = 1           # 超音波高度計權重*
WEIGHTED_BAROM = 0           # 氣壓高度計權重*

# ->超音波高度計參數
SONAR_AIR_TEMP = 28          # 氣溫(攝氏)*
SONAR_TRIG_PIN = 2           # TRIG輸出腳位
SONAR_ECHO_PIN = 3           # ECHO輸入腳位
SONAR_TIMEOUT = 0.015        # 無回應超時限制(秒，過低時遠距離偵測將會變成超時)*
SONAR_OFFSET = 0             # 高度校正偏移量(m，向上為正)
SONAR_SAMPLE_NUM_CAL = 10    # 高度校正取樣次數*
SONAR_SAMPLE_NUM_AVE = 10    # 均值濾波取樣次數*
SONAR_KF_INPUT_MAX = 3       # 卡爾曼濾波器輸入值限制(高度，m)*
SONAR_KF_Q = 5               # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*
SONAR_ERR_COUNT = 0          # 量測超時次數記錄
SONAR_FILTER_ALT = 0.2       # 突波濾波作用起始高度(m)
SONAR_MAX_CHANGE = 0.3       # 突波容許值(m)

# ->氣壓計參數
BAROM_OFFSET = 0             # 高度校正偏移量(m，向上為正)
BAROM_SAMPLE_NUM_CAL = 10    # 高度校正取樣次數*
BAROM_KF_INPUT_MAX = 3       # 卡爾曼濾波器輸入值限制(高度，m)*
BAROM_KF_Q = 5               # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*

# ======數值記錄=================================================================
# 高度控制器
R_TARGET_ALT = []                     # 記錄目標高
R_ACTUAL_ALT_CTRL = []                # 記錄實際高
R_TARGET_THRUST = []                  # 記錄目標推力
R_ALT_ERROR = []                      # 記錄高度誤差
R_ALT_GP = []                         # 記錄P增益
R_ALT_GI = []                         # 記錄I增益
R_ALT_GD = []                         # 記錄D增益

# 位置控制器
R_TARGET_ROLL = []                    # 記錄目標滾轉角
R_TARGET_PITCH = []                   # 記錄目標俯仰角
R_ACTUAL_ROLL = []                    # 記錄實際滾轉角
R_ACTUAL_PITCH = []                   # 記錄實際俯仰角
R_BLOCK_X_ROW = []                    # 記錄X座標原始值
R_BLOCK_Y_ROW = []                    # 記錄Y座標原始值
R_BLOCK_AREA = []                     # 記錄面積變化
R_RECORD_SCALE = 10000                # 面積縮放比
R_BLOCK_X = []                        # 記錄X座標
R_BLOCK_Y = []                        # 記錄Y座標

# 高度感測器
R_ACTUAL_ALT = []                     # 記錄加權值
R_ACTUAL_ALT_SONAR = []               # 記錄超音波高度計值
R_ACTUAL_ALT_BAROM = []               # 記錄氣壓高度計值

# 超音波高度計
R_ACTUAL_ALT_SONAR_ROW = []           # 記錄原始值
R_ACTUAL_ALT_SONAR_OFFSET = []        # 記錄補正值
R_ACTUAL_ALT_SONAR_AVE = []           # 記錄平均值
R_ACTUAL_ALT_SONAR_KF = []            # 記錄KF值

# 氣壓高度計
R_ACTUAL_ALT_BAROM_ROW = []           # 記錄原始值
R_ACTUAL_ALT_BAROM_OFFSET = []        # 記錄補正值
R_ACTUAL_ALT_BAROM_AVE = []           # 記錄平均值
R_ACTUAL_ALT_BAROM_KF = []            # 記錄KF值

print('<啟動程序> 載入全域變數設定完畢')

# 與UAV執行連線
print('<啟動程序> 與UAV連線中，請稍候...')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線')


# ======背景主函式函式區塊=============================================================
def altitude_controller():
    global TARGET_THRUST, STATE_ALT
    # 變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 等待感測器啟動
    if not STATE_SENSOR:
        print('<啟動程序> 高度控制器：等待高度感測器啟動')
    while not STATE_SENSOR:
        time.sleep(0.1)

    # 回報高度控制器啟用
    STATE_ALT = True
    print('<啟動程序> 高度控制器：啟動')

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
    print('<關閉程序> 高度控制器：關閉')


# 位置控制器
def position_controller():
    global TARGET_PITCH, TARGET_ROLL, ACTUAL_POS, STATE_POS
    # 均值濾波器設定
    x_register = []
    y_register = []

    # 等待相機啟動
    if not STATE_CAM:
        print('<啟動程序> 位置控制器：等待影像擷取程式啟動')
    while not STATE_CAM:
        time.sleep(0.1)

    # 回報定位控制器啟用
    STATE_POS = True
    print('<啟動程序> 位置控制器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        while MODE == 'POS' and STATE_SYSTEM:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 取得色塊XY座標
            x_row, y_row, block_area = img_calculate(POS_COLOR)
            R_BLOCK_X_ROW.append(x_row)
            R_BLOCK_Y_ROW.append(y_row)
            R_BLOCK_AREA.append(block_area / R_RECORD_SCALE)

            # X座標均值濾波器
            x_register.insert(0, x_row)
            if len(x_register) > POS_SAMPLE_NUM_AVE:
                x_register.pop()
            block_x = np.average(x_register)
            R_BLOCK_X.append(block_x)

            # Y座標均值濾波器
            y_register.insert(0, y_row)
            if len(y_register) > POS_SAMPLE_NUM_AVE:
                y_register.pop()
            block_y = np.average(y_register)
            R_BLOCK_Y.append(block_y)

            # 輸出座標至全域變數
            ACTUAL_POS = [block_x, block_y]

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

            # 傾角安全限制(內設定45度與飛控板同步)
            if TARGET_ROLL > 45:
                TARGET_ROLL = 45
            elif TARGET_ROLL < -45:
                TARGET_ROLL = -45
            if TARGET_PITCH > 45:
                TARGET_PITCH = 45
            elif TARGET_PITCH < -45:
                TARGET_PITCH = -45

            # 記錄目標輸出命令
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
                TARGET_PITCH = 0
                print('<飛行狀態> 位置控制器：暫停')

        # 重設暫存器
        x_register = []
        y_register = []

        # 記錄值
        R_BLOCK_X_ROW.append(0)
        R_BLOCK_Y_ROW.append(0)
        R_BLOCK_X.append(0)
        R_BLOCK_Y.append(0)
        R_TARGET_ROLL.append(TARGET_ROLL)
        R_TARGET_PITCH.append(TARGET_PITCH)
        R_BLOCK_AREA.append(0)
        R_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
        R_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

        # 迴圈執行間隔
        time.sleep(1 / FREQ_POS)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_SYSTEM:
            print('<飛行狀態> 位置控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_POS = False
    print('<關閉程序> 位置控制器：關閉')


# 高度感測器整合程式
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
        print('<啟動程序> 高度感測器：等待超音波高度計啟動')
    while not STATE_SENSOR_SONAR:
        time.sleep(0.1)

    # 回報高度感測器啟用
    STATE_SENSOR = True
    print('<啟動程序> 高度感測器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 讀取超音波高度計資料
        height_sonar = ACTUAL_ALT_SONAR
        R_ACTUAL_ALT_SONAR.append(height_sonar)     # 記錄超音波高度計值

        # 氣壓高度計數據處理
        # 卡爾曼濾波-預測
        x_p = x_o
        p_p = p_o + q

        # 讀值與補正
        R_ACTUAL_ALT_BAROM_ROW.append(uav.location.global_relative_frame.alt)  # 記錄氣壓高度計原始值
        height_row_barom = uav.location.global_relative_frame.alt + BAROM_OFFSET
        R_ACTUAL_ALT_BAROM_OFFSET.append(height_row_barom)     # 記錄補正值

        # 均值濾波器
        height_register_barom.insert(0, height_row_barom)
        if len(height_register_barom) > BAROM_SAMPLE_NUM_AVE:
            height_register_barom.pop()
        height_barom_ave = np.average(height_register_barom)
        R_ACTUAL_ALT_BAROM_AVE.append(height_barom_ave)      # 記錄均值濾波值

        # 卡爾曼濾波器輸入限制(防止突波)
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
        R_ACTUAL_ALT_BAROM_KF.append(height_barom)  # 記錄KF值

        # 卡爾曼濾波：傳遞值給下次迴圈使用
        x_o = x
        p_o = p
        z_o = z

        # 加權平均
        alt_num = (WEIGHTED_SONAR * height_sonar + WEIGHTED_BAROM * height_barom)
        alt_den = (WEIGHTED_SONAR + WEIGHTED_BAROM)
        alt_ave = alt_num / alt_den

        # 更新高度值至全域變數
        ACTUAL_ALT = alt_ave
        ACTUAL_ALT_BAROM = height_barom
        R_ACTUAL_ALT.append(ACTUAL_ALT)             # 記錄總高度值

        # 同捆執行終點
        THREAD_OCCUPY.release()

        time.sleep(1 / FREQ_SENSOR)

    # 當系統裝態為否時，跳出迴圈並關閉函式
    STATE_SENSOR = False
    print('<關閉程序> 高度感測器：關閉')


# 超音波高度計
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
    print('<啟動程序> 超音波高度計：啟動')

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
            # 輸出啟用脈波
            GPIO.output(SONAR_TRIG_PIN, True)
            time.sleep(0.00001)              # 發出10us脈波
            GPIO.output(SONAR_TRIG_PIN, False)
            timeout_start = time.time()      # 記錄超時基準時間

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

        # 結束優先執行
        THREAD_OCCUPY.release()

        # 計算高度值
        pulse_duration = pulse_end - pulse_start
        height_row = (pulse_duration * (331.3 + 0.606 * SONAR_AIR_TEMP)) / 2
        R_ACTUAL_ALT_SONAR_ROW.append(height_row)      # 記錄原始值

        # 高度角度補正
        roll_angle = uav.attitude.roll
        pitch_angle = uav.attitude.pitch
        height_row = (height_row * math.cos(roll_angle) * math.cos(pitch_angle)) + SONAR_OFFSET
        R_ACTUAL_ALT_SONAR_OFFSET.append(height_row)   # 記錄補正值

        # 上下降突波防止
        if ACTUAL_ALT > SONAR_FILTER_ALT:
            if math.fabs(height_row-z_o) > SONAR_MAX_CHANGE:
                height_row = height_ave

        # 均值濾波器(使線條滑順)
        height_register.insert(0, height_row)
        if len(height_register) > SONAR_SAMPLE_NUM_AVE:
            height_register.pop()
        height_ave = np.average(height_register)       # 依據取樣次數計算即時平均值(會產生響應延遲)
        R_ACTUAL_ALT_SONAR_AVE.append(height_ave)      # 記錄平均值

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
        R_ACTUAL_ALT_SONAR_KF.append(ACTUAL_ALT_SONAR)  # 記錄KF值

        # 傳遞值給下次迴圈使用
        x_o = x
        p_o = p
        z_o = z

        # 迴圈執行間隔
        time.sleep(1 / FREQ_SONAR)

    # 當系統裝態為否時，跳出迴圈並關閉函式
    STATE_SONAR = False
    print('<關閉程序> 超音波高度計：關閉；量測超時次數共 %d 次' % SONAR_ERR_COUNT)


# 命令傳輸程式
def command_transfer():
    global STATE_CMD
    # 執行偏航角校正
    yaw_calibrate()

    # 回報命令傳輸程式啟動
    STATE_CMD = True
    print('<啟動程序> 命令傳輸程式：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 命令編碼
        # 模式一：使用角度控制
        if CMD_MSG == 1:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # time boot
                0,  # target system
                0,  # target component
                0b00000111,  # type mask: bit 1 is ignore
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # quaternion
                math.radians(TARGET_ROLL_RATE),  # body roll rate in radian
                math.radians(TARGET_PITCH_RATE),  # body pitch rate in radian
                math.radians(TARGET_YAW_RATE),  # body yaw rate in radian
                TARGET_THRUST)  # thrust

        # 模式二：使用偏航角度補正
        elif CMD_MSG == 2:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # time boot
                0,  # target system
                0,  # target component
                0b00000111,  # type mask: bit 1 is ignore
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW + TARGET_YAW_OFFSET),  # quaternion
                math.radians(TARGET_ROLL_RATE),  # body roll rate in radian
                math.radians(TARGET_PITCH_RATE),  # body pitch rate in radian
                math.radians(TARGET_YAW_RATE),  # body yaw rate in radian
                TARGET_THRUST)  # thrust

        # 模式三：使用偏航角速度控制
        else:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # time boot
                0,  # target system
                0,  # target component
                0b00000011,  # type mask: bit 1 is ignore
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # quaternion
                math.radians(TARGET_ROLL_RATE),  # body roll rate in radian
                math.radians(TARGET_PITCH_RATE),  # body pitch rate in radian
                math.radians(TARGET_YAW_RATE),  # body yaw rate in radian (Activate)
                TARGET_THRUST)  # thrust

        # 傳輸命令
        uav.send_mavlink(msg)

        # 同捆執行終點
        THREAD_OCCUPY.release()

        # 迴圈執行間隔
        time.sleep(1 / FREQ_CMD)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_CMD = False
    print('<關閉程序> 命令傳輸程式：關閉')


# 影像擷取程式
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
            CAM_IMAGE = cv2.resize(image, (CAM_IMAGE_WEIGHT, CAM_IMAGE_HEIGHT))
            VIDEO_SAVE.write(CAM_IMAGE)

            # 回報影像擷取程式啟動
            if not STATE_CAM:
                STATE_CAM = True
                print('<啟動程序> 影像擷取程式：啟動')

            cv2.imshow("origin", CAM_IMAGE)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 手動關閉影像顯示
            if cv2.waitKey(int(1000/FREQ_CAM)) & 0xff == ord("q"):
                STATE_SYSTEM = False
                print('<關閉程序> 已手動關閉相機，請手動重啟程式')
                break

        # 擷取失敗
        else:
            print('<錯誤> 影像擷取程式：無法擷取影像')
            STATE_SYSTEM = False
            print('<嚴重錯誤> 請檢查相機並手動重啟程式!!!!')
            break

    # 當系統狀態為否時，跳出迴圈並關閉函式
    CAM_VIDEO.release()
    VIDEO_SAVE.release()
    cv2.destroyAllWindows()                # 關閉影像顯示
    STATE_CAM = False
    print('<關閉程序> 影像擷取程式：關閉')


# ======子函式區塊==============================================================
# 超音波高度計校正程序
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
    print('<啟動程序> 超音波高度計校正完畢，偏移 %.3f m' % SONAR_OFFSET)


# 氣壓高度計校正程序
def barometer_calibrate():
    global BAROM_OFFSET
    # 平均降噪設定
    height_register = []

    # 校正氣壓計與平面設定
    uav.send_calibrate_barometer()

    # 量測迴圈(線程優先執行)
    THREAD_OCCUPY.acquire()
    for i in range(0, BAROM_SAMPLE_NUM_CAL):
        # 讀取氣壓計高度值
        height = uav.location.global_relative_frame.alt

        # 均值濾波器
        height_register.append(height)

        # 等待10ms執行下次量測
        time.sleep(0.1)

    # 結束優先執行
    THREAD_OCCUPY.release()

    # 更新高度校正偏移量
    BAROM_OFFSET = -np.average(height_register)
    print('<啟動程序> 氣壓高度計校正完畢，偏移 %.3f m' % BAROM_OFFSET)


# 偏航角校正程序
def yaw_calibrate():
    global TARGET_YAW_OFFSET
    # 更新偏航角校正偏移量
    TARGET_YAW_OFFSET = -math.degrees(uav.attitude.yaw)
    print('<啟動程序> 偏航角校正完畢，偏移 %.3f degree' % TARGET_YAW_OFFSET)


# 轉換尤拉角到四元數
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


# 影像計算程序
def img_calculate(color):
    # 色域轉換BGR->HSV
    image_hsv = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2HSV)
    # 閉運算
    image_close = cv2.morphologyEx(image_hsv, cv2.MORPH_CLOSE, np.ones(IMG_MORPHOLOGY_EX_KERNEL, np.uint8))

    # 依照目標色抓該顏色輪廓
    # 紅色
    if color == 'red':
        image_mask = cv2.inRange(image_close, IMG_RED_L, IMG_RED_H)
    # 綠色
    elif color == 'green':
        image_mask = cv2.inRange(image_close, IMG_GREEN_L, IMG_GREEN_H)
    # 紅色
    elif color == 'blue':
        image_mask = cv2.inRange(image_close, IMG_BLUE_L, IMG_BLUE_H)
    # 黑色
    else:
        image_mask = cv2.inRange(image_close, IMG_BLACK_L, IMG_BLACK_H)

    # 取得影像、輪廓點與索引
    image_c, contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
                moment_x_row = int(moment["m10"] / moment["m00"])
                moment_y_row = int(moment["m01"] / moment["m00"])
            else:
                moment_x_row = 0
                moment_y_row = 0

            # 計算色塊位置(相對於畫面中心)
            block_x_offset = moment_x_row - BODY_REF_X - BODY_OFFSET_X
            block_y_offset = BODY_REF_Y - moment_y_row - BODY_OFFSET_Y

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


# ======飛行程序指令(條件與命令)======================================================
# 啟動前檢查
def pre_check():
    print('<啟動程序> 啟動前檢查程序：啟動')

    # 等待高度控制器啟動
    if not STATE_ALT:
        print('<啟動程序> 主程序：等待高度控制器啟動')
    while not STATE_ALT:
        time.sleep(0.1)

    # 等待位置控制器啟動
    if not STATE_POS:
        print('<啟動程序> 主程序：等待位置控制器啟動')
    while not STATE_POS:
        time.sleep(0.1)

    # 等待命令傳輸啟動
    if not STATE_CMD:
        print('<啟動程序> 主程序：等待命令傳輸程式啟動')
    while not STATE_CMD:
        time.sleep(0.1)

    print('<啟動程序> 啟動前檢查程序：完成')


# 飛行模式切換
def flight_mode(mode, color=''):
    global MODE, POS_COLOR
    # 模式切換
    MODE = mode
    POS_COLOR = color
    if MODE == 'POS':
        print('<飛行狀態> 切換至定位模式，顏色追蹤：', POS_COLOR)
    elif MODE == 'LINE':
        print('<飛行狀態> 切換至循線模式')


# 輸入任務顏色
def input_color():
    color = str(input('輸入追蹤顏色(rgb&d)=>'))
    if color == 'r':
        flight_mode('POS', 'red')  # 使用定位模式，紅色
    elif color == 'g':
        flight_mode('POS', 'green')  # 使用定位模式，綠色
    elif color == 'b':
        flight_mode('POS', 'blue')  # 使用定位模式，藍色
    else:
        flight_mode('POS', 'black')  # 使用定位模式，黑色


# 起飛程序
def take_off(target_alt):
    global TARGET_ALT
    # 切換至導航模式
    uav.mode = VehicleMode("GUIDED_NOGPS")
    while not uav.mode.name == 'GUIDED_NOGPS':
        time.sleep(0.5)
    print('<啟動程序> 無人機已切換至導航模式')

    # 解鎖無人機
    uav.armed = True
    print('<啟動程序> 無人機已解鎖')

    # 設定目標高度
    print('<飛行命令> 起飛至目標高度 %.2f m' % target_alt)
    TARGET_ALT = target_alt

    # 等待抵達目標高度
    while ACTUAL_ALT < TARGET_ALT * TAKEOFF_SETTLE:
        print('<飛行狀態> 起飛中，目前高度 %.2f m，距離目標高度尚有 %.2f m' % (ACTUAL_ALT, TARGET_ALT-ACTUAL_ALT))
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' % (ACTUAL_POS[0], ACTUAL_POS[1]))
        time.sleep(1)

    # 回報抵達目標高度
    print('<飛行狀態> 已抵達目標高度 %.2f m' % TARGET_ALT)


# 懸停程序
def hover(duration):
    # 設定飛行命令
    duration = int(duration)
    print('<飛行命令> 懸停於 %.2f m，維持 %d sec' % (TARGET_ALT, duration))

    # 等待時間到達
    for i in range(1, duration+1):
        print('<飛行狀態> 懸停中，目前高度 %.2f m，剩餘時間 %d sec' % (ACTUAL_ALT, duration + 1 - i))
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' % (ACTUAL_POS[0], ACTUAL_POS[1]))
        time.sleep(1)

    # 回報懸停結束
    print('<飛行狀態> 懸停狀態結束，目前高度 %.2f m' % ACTUAL_ALT)


# 降落程序
def landing():
    global TARGET_ALT
    # 設定目標高度
    print('<飛行命令> 降落至地面')
    TARGET_ALT = -0.1           # 負值以確保不會著地後彈升

    # 等待抵達目標高度
    while ACTUAL_ALT > LANDING_CUTOFF:
        print('<飛行狀態> 降落中，目前高度 %.2f m' % ACTUAL_ALT)
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' % (ACTUAL_POS[0], ACTUAL_POS[1]))
        time.sleep(1)

    # 回報降落成功
    print('<飛行狀態> 已降落地面')

    # 返回手動模式
    uav.mode = VehicleMode("STABILIZE")
    while not uav.mode.name == 'STABILIZE':
        time.sleep(0.5)
    print('<關閉程序> 無人機已切換至手動模式')

    # 結束與無人機連線
    uav.close()
    print('<關閉程序> 無人機連線已中斷')


# ======前景主函式區塊=================================================================
# 任務主函式
def main():
    global STATE_SYSTEM
    # 啟動前檢查，回報主程式啟動
    pre_check()
    print('<啟動程序> 主程序：啟動')

    # 飛行流程
    take_off(1)                 # 起飛至1公尺高
    hover(5)                    # 懸停5秒鐘
    landing()                   # 降落至地面

    # 關閉背景多線程
    STATE_SYSTEM = False
    print('<關閉程序> 主程序：關閉')


# 多線程函式
def thread_procedure():
    # 定義多線程
    sonar_sen = threading.Thread(target=sonar)
    alt_sen = threading.Thread(target=altitude_sensor)
    alt = threading.Thread(target=altitude_controller)
    cam = threading.Thread(target=camera)
    pos = threading.Thread(target=position_controller)
    cmd = threading.Thread(target=command_transfer)
    m = threading.Thread(target=main)

    # 執行多線程(依啟動順序排放)
    print('<啟動程序> 多線程程序：啟動')
    input_color()               # 與多線程開始前設定顏色，防止記錄錯誤
    sonar_sen.start()
    alt_sen.start()
    alt.start()
    cam.start()
    pos.start()
    cmd.start()
    m.start()

    # 等待多線程結束(依結束順序排放)
    m.join()
    cmd.join()
    pos.join()
    cam.join()
    alt.join()
    alt_sen.join()
    sonar_sen.join()
    print('<關閉程序> 多線程程序：關閉')


# ======數值紀錄區塊======
def show_data():
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

    plt.plot(num_pos, R_BLOCK_X_ROW, label="X Position - Row")
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
    plt.plot(num_pos, R_BLOCK_Y_ROW, label="Y Position - Row")
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

    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_ROW, label="Row Data")
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
    for i in range(1, len(R_ACTUAL_ALT_BAROM_ROW) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_ROW, label="Row Data")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_OFFSET, label="Height Offset")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_AVE, label="Average Filter")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_KF, label="Kalman Filter")

    plt.xlabel('Data number')
    plt.ylabel('Height(m)')
    plt.title('Altitude Data - Barometer')
    plt.legend()
    plt.savefig('Altitude Data - Barometer.png')
    plt.show()


# ======程式執行區======
thread_procedure()
show_data()