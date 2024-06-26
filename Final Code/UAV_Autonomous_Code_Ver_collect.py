# 無人機已驗證程式碼
# 高度感測器模組未驗證
# 高度控制器模組未驗證
# 定位控制器模組未驗證
# ================ 本檔案不使用

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
STATE_HDG = False
STATE_CMD = False
STATE_CAM = False
STATE_SENSOR_SONAR = False
STATE_BAROM = False
STATE_SENSOR = False
MODE = ''                # 模式旗標
MISSION_COLOR = ''       # 任務顏色

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

# ======控制器參數設定===================================================
# 控制器迴圈頻率(預設上限30Hz)
FREQ_ALT = 30            # 高度控制器頻率(不快於高度感測器頻率)
FREQ_POS = 30            # 位置控制器頻率(不快於影像禎率)
FREQ_HDG = 30            # 航向控制器頻率(不快於影像禎率)
FREQ_SONAR = 30          # 超音波感測器頻率(5~50Hz，越高雜訊越多)
FREQ_SENSOR = 30         # 高度感測器頻率(不快於超音波感測器頻率)
FREQ_CMD = 30            # 命令傳輸頻率(不快於控制器頻率)
FREQ_CAM = 30            # 影像禎率(fps，依系統負載調整)

# 高度控制器參數
ALT_KP = 1               # P增益值*
ALT_KI = 0               # I增益值
ALT_KD = 0               # D增益值
ALT_DZ = 0.05            # 油門平飛安定區間(單向)*

# 航向控制器參數
HDG_KP = 1               # P增益值*
HDG_KI = 0               # I增益值
HDG_KD = 0               # D增益值
HDG_DZ = 5               # 航向安定區間(單向)*
HDG_SAMPLE_NUM_AVE = 10  # 角度差均值濾波取樣次數*
HDG_YAW_RATE_LIMIT = 10  # 角速度限制*

# 命令傳述程式參數
CMD_MSG = 3              # 命令編碼模式(1:角度 2:角度補正 3:角速度)

# 位置控制器參數
POS_COLOR = ''              # 定位的色塊顏色
POS_SAMPLE_NUM_AVE = 10     # 色塊座標均值濾波取樣次數*
POS_DZ = 20                 # 色塊定位作動忽略區間(單向，px)*
POS_ANGLE_LIMIT = 5         # 定位控制最大飛行角度(degree)*

# 起降程序參數
TAKEOFF_SETTLE = 0.95         # 安定高度百分比(%)*
LANDING_CUTOFF = 0.05         # 油門關閉高度(m)*
LANDING_POS_LIMIT = 100       # 降落定位範圍限制(單向，px)*
DROP_POS_LIMIT = 100          # 空投定位範圍限制(單向，px)*

# ======影像參數設定=============================================================
# 相機設定參數
CAM_VIDEO = cv2.VideoCapture(0)     # 影像來源(預設來源為0)
CAM_IMAGE = 0                       # 儲存影像
CAM_IMAGE_WEIGHT = 480              # 影像寬度(px)
CAM_IMAGE_HEIGHT = 340              # 影像高度(px)

# 影像計算參數
IMG_BLUE_L = np.array([85, 76, 90])        # 各顏色上下限值，HSV色域
IMG_BLUE_H = np.array([127, 255, 255])
IMG_GREEN_L = np.array([44, 104, 27])
IMG_GREEN_H = np.array([87, 255, 255])
IMG_RED_L = np.array([160, 111, 56])
IMG_RED_H = np.array([180, 255, 255])
IMG_BLACK_L = np.array([0, 0, 0])
IMG_BLACK_H = np.array([180, 255, 46])
IMG_AREA_LIMIT = 6000                      # 偵測面積下限值*
BODY_REF_X = int(CAM_IMAGE_WEIGHT / 2)     # 機身位置X座標
BODY_REF_Y = int(CAM_IMAGE_HEIGHT / 2)     # 機身位置Y座標

# 線角度計算
ANG_THRESHOLD = 40              # 二值化門檻值(調高比較能看到線，但可能有雜訊)*
ANG_THRESHOLD_NEW = 255         # 取得黑白照填入最高值
ANG_GB_KERNEL_SIZE = (7, 7)     # 高斯模糊矩陣大小(越大越模糊)*
ANG_GB_SIGMA = 2                # 高斯模糊色域標準差(越大越模糊)*
ANG_FILLED_COLOR = 255          # 界線填充顏色
ANG_LINE_TRACK_AREA_L = 450     # 線追蹤型心計算面積下限*
ANG_LINE_TRACK_AREA_H = 4500    # 線追蹤型心計算面積上限*

# ======高度感測器參數=============================================================
# ->高度感測器整合參數
WEIGHTED_SONAR = 1            # 超音波高度計權重*
WEIGHTED_BAROM = 0            # 氣壓高度計權重*

# ->超音波高度計參數
SONAR_AIR_TEMP = 28           # 氣溫(攝氏)*
SONAR_TRIG_PIN = 2            # TRIG輸出腳位
SONAR_ECHO_PIN = 3            # ECHO輸入腳位
SONAR_TIMEOUT = 0.05          # 無回應超時限制(秒)*
SONAR_OFFSET = 0              # 高度校正偏移量(向上為正)
SONAR_SAMPLE_NUM_CAL = 10     # 高度校正取樣次數*
SONAR_SAMPLE_NUM_AVE = 10     # 均值率波取樣次數*
SONAR_KF_INPUT_MAX = 3        # 卡爾曼濾波器輸入值限制(高度，m)*
SONAR_KF_Q = 5                # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*

# ->氣壓計參數
BAROM_OFFSET = 0              # 高度校正偏移量(向上為正)
BAROM_SAMPLE_NUM_CAL = 10     # 高度校正取樣次數*
BAROM_SAMPLE_NUM_AVE = 10     # 氣壓高度計均值濾波取樣次數*

# 沙包盒參數
BOX_PWM_PIN = 4               # 沙包盒PWM輸出腳位
BOX_PWM_FREQ = 50             # PWM頻率
BOX_PWM_OPEN = 400            # 開門位置
BOX_PWM_CLOSE = 2350          # 關門位置

# 同步多線程設定
WAIT = threading.Lock()       # 多線程獨佔執行設定

# ======數值記錄=================================================================
# 高度控制器
R_TARGET_ALT = []             # 記錄目標高
R_ACTUAL_ALT_CTRL = []        # 記錄實際高
R_TARGET_THRUST = []          # 記錄目標推力
R_ALT_ERROR = []              # 記錄高度誤差
R_ALT_GP = []                 # 記錄P增益
R_ALT_GI = []                 # 記錄I增益
R_ALT_GD = []                 # 記錄D增益

# 位置控制器
R_TARGET_ROLL = []            # 記錄目標滾轉角
R_TARGET_PITCH = []           # 記錄目標俯仰角
R_ACTUAL_ROLL = []            # 記錄實際滾轉角
R_ACTUAL_PITCH = []           # 記錄實際俯仰角
R_BLOCK_X_ROW = []            # 記錄X座標原始值
R_BLOCK_Y_ROW = []            # 記錄Y座標原始值
R_BLOCK_X = []                # 記錄X座標
R_BLOCK_Y = []                # 記錄Y座標

# 航向控制器
R_HDG_ERROR_ROW = []
R_HDG_ERROR = []
R_TARGET_YAW_RATE = []
R_HDG_GP = []                 # 記錄P增益
R_HDG_GI = []                 # 記錄I增益
R_HDG_GD = []                 # 記錄D增益

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

print('<啟動程序> 載入全域變數設定完畢')


# 與UAV執行連線
print('<啟動程序> 與UAV連線中，請稍候...')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線')


# ======背景主函式函式區塊=============================================================
# 高度控制器
def altitude_controller():
    global TARGET_THRUST, STATE_ALT
    # 變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 等待感測器啟動
    if not STATE_SENSOR:
        print('<啟動程序> 高度控制器：等待高度感測器啟用')
    while not STATE_SENSOR:
        time.sleep(0.1)

    # 回報高度控制器啟用
    STATE_ALT = True
    print('<啟動程序> 高度控制器：啟動')

    while STATE_SYSTEM:
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

        # 迴圈執行間隔
        time.sleep(1 / FREQ_ALT)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 高度控制器：關閉')


# 位置控制器
def position_controller():
    global TARGET_PITCH, TARGET_ROLL, STATE_POS
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

    while STATE_SYSTEM:
        while MODE == 'POS' and STATE_SYSTEM:
            # 取得色塊XY座標
            x_row, y_row, img_area = img_calculate(POS_COLOR)
            R_BLOCK_X_ROW.append(x_row)
            R_BLOCK_Y_ROW.append(y_row)

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

            # 判斷與輸出姿態命令(待實際驗證方向)
            # X座標
            if block_x > POS_DZ:                 # Right
                TARGET_ROLL = POS_ANGLE_LIMIT
            elif block_x < -POS_DZ:              # Left
                TARGET_ROLL = -POS_ANGLE_LIMIT
            else:                                # 定位作動忽略區間
                TARGET_ROLL = 0
            # Y座標
            if block_y > POS_DZ:                 # Forward
                TARGET_PITCH = POS_ANGLE_LIMIT
            elif block_y < -POS_DZ:              # Backward
                TARGET_PITCH = -POS_ANGLE_LIMIT
            else:                                # 定位作動忽略區間
                TARGET_PITCH = 0

            # 記錄目標輸出命令
            R_TARGET_ROLL.append(TARGET_ROLL)
            R_TARGET_PITCH.append(TARGET_PITCH)

            # 記錄實際轉角
            R_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
            R_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

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
        R_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
        R_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

        # 迴圈執行間隔
        time.sleep(1 / FREQ_POS)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_SYSTEM:
            print('<飛行狀態> 位置控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 位置控制器：關閉')


# 航向控制器
def heading_controller():
    global TARGET_PITCH, TARGET_YAW_RATE, STATE_HDG
    # 變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 均值濾波器設定
    hdg_register = []

    # 等待相機啟動
    if not STATE_CAM:
        print('<啟動程序> 位置控制器：等待影像擷取程式啟動')
    while not STATE_CAM:
        time.sleep(0.1)

    # 回報定位控制器啟用
    STATE_HDG = True
    print('<啟動程序> 航向控制器：啟動')

    while STATE_SYSTEM:
        while MODE == 'LINE' and STATE_SYSTEM:
            # 取得現在角度誤差值
            error_row = angle_calculate()
            R_HDG_ERROR_ROW.append(error_row)

            # X座標均值濾波器
            hdg_register.insert(0, error_row)
            if len(hdg_register) > HDG_SAMPLE_NUM_AVE:
                hdg_register.pop()
            error = np.average(hdg_register)
            R_HDG_ERROR.append(error)

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
            R_HDG_GP.append(gain_p)
            R_HDG_GI.append(gain_i)
            R_HDG_GD.append(gain_d)

            # 航向固定安定區間
            if HDG_DZ > angular_rate > - HDG_DZ:
                angular_rate = 0

            # 角速度上下限制
            if angular_rate > HDG_YAW_RATE_LIMIT:
                angular_rate = HDG_YAW_RATE_LIMIT
            elif angular_rate < -HDG_YAW_RATE_LIMIT:
                angular_rate = -HDG_YAW_RATE_LIMIT

            # 更新角速度值至全域變數
            TARGET_YAW_RATE = angular_rate
            R_TARGET_YAW_RATE.append(TARGET_YAW_RATE)

            # 傳遞值給下次迴圈使用
            previous_error = error
            previous_time = current_time

            # 迴圈執行間隔
            time.sleep(1 / FREQ_HDG)

            # 當飛行模式不為循跡時，跳出迴圈並暫停函式
            if not MODE == 'LINE':
                # 歸零姿態數值
                TARGET_PITCH = 0
                TARGET_YAW_RATE = 0
                print('<飛行狀態> 航向控制器：暫停')

        # 重設暫存器
        hdg_register = []

        # 記錄值
        R_HDG_ERROR_ROW.append(0)
        R_HDG_ERROR.append(0)
        R_HDG_GP.append(0)
        R_HDG_GI.append(0)
        R_HDG_GD.append(0)
        R_TARGET_YAW_RATE.append(TARGET_YAW_RATE)

        # 迴圈執行間隔
        time.sleep(1 / FREQ_HDG)

        # 當飛行模式為循跡時，跳入迴圈並啟動函式
        if MODE == 'LINE' and STATE_SYSTEM:
            print('<飛行狀態> 航向控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 航向控制器：關閉')


# 高度感測器整合程式
def altitude_sensor():
    global ACTUAL_ALT, ACTUAL_ALT_BAROM, STATE_SENSOR
    # 平均降噪設定
    height_register_barom = []

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

    while STATE_SYSTEM:
        # 讀取感測器資料
        # 超音波高度計
        height_sonar = ACTUAL_ALT_SONAR
        R_ACTUAL_ALT_SONAR.append(height_sonar)     # 記錄超音波高度計值

        # 氣壓高度計(含均值濾波)
        height_row_barom = uav.location.global_relative_frame.alt + BAROM_OFFSET
        R_ACTUAL_ALT_BAROM_ROW.append(height_row_barom)  # 記錄氣壓高度計原始值
        height_register_barom.insert(0, height_row_barom)
        if len(height_register_barom) > BAROM_SAMPLE_NUM_AVE:
            height_register_barom.pop()
        height_barom = np.average(height_register_barom)
        R_ACTUAL_ALT_BAROM.append(height_barom)     # 記錄氣壓高度計值

        # 加權平均
        alt_num = (WEIGHTED_SONAR * height_sonar + WEIGHTED_BAROM * height_barom)
        alt_den = (WEIGHTED_SONAR + WEIGHTED_BAROM)
        alt_ave = alt_num / alt_den

        # 更新高度值至全域變數
        ACTUAL_ALT = alt_ave
        ACTUAL_ALT_BAROM = height_barom
        R_ACTUAL_ALT.append(ACTUAL_ALT)             # 記錄總高度值

        # 迴圈執行間隔
        time.sleep(1 / FREQ_SENSOR)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 高度感測器：關閉')


# 超音波高度計
def sonar():
    global ACTUAL_ALT_SONAR, STATE_SENSOR_SONAR
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

    # 執行超音波高度計校正程序
    sonar_calibrate()

    # 回報超音波高度計啟用
    STATE_SONAR = True
    print('<啟動程序> 超音波高度計：啟動')

    while STATE_SYSTEM:
        # 設定超時旗標
        timeout = True

        # 卡爾曼濾波-預測
        x_p = x_o
        p_p = p_o + q

        # 量測迴圈(超時防止)
        WAIT.acquire()
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

            # 若發生超時則顯示於訊息
            if timeout:
                pass
        WAIT.release()

        # 計算高度值
        pulse_duration = pulse_end - pulse_start
        height_row = (pulse_duration * (331.3 + 0.606 * SONAR_AIR_TEMP)) / 2
        R_ACTUAL_ALT_SONAR_ROW.append(height_row)      # 記錄原始值

        # 高度角度補正
        roll_angel = uav.attitude.roll
        pitch_angel = uav.attitude.pitch
        height_row = (height_row * math.cos(roll_angel) * math.cos(pitch_angel)) + SONAR_OFFSET
        R_ACTUAL_ALT_SONAR_OFFSET.append(height_row)   # 記錄補正值

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

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 超音波高度計：關閉')


# 命令傳輸程式
def command_transfer():
    global STATE_CMD
    # 執行偏航角校正
    yaw_calibrate()

    # 回報命令傳輸程式啟動
    STATE_CMD = True
    print('<啟動程序> 命令傳輸程式：啟動')

    while STATE_SYSTEM:
        # 命令編碼(預設採行模式三)
        # 模式一：使用角度控制
        if CMD_MSG == 1:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # time boot
                0,  # target system
                0,  # target component
                0b00000111,  # type mask: bit 1 is ignore
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # quaternion
                math.radians(TARGET_ROLL_RATE),  # body roll rate in rad/s (Not Use)
                math.radians(TARGET_PITCH_RATE),  # body pitch rate in rad/s (Not Use)
                math.radians(TARGET_YAW_RATE),  # body yaw rate in rad/s (Not Use)
                TARGET_THRUST)  # thrust

        # 模式二：使用偏航角度補正
        elif CMD_MSG == 2:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # time boot
                0,  # target system
                0,  # target component
                0b00000111,  # type mask: bit 1 is ignore
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW + TARGET_YAW_OFFSET),  # quaternion
                math.radians(TARGET_ROLL_RATE),  # body roll rate in rad/s (Not Use)
                math.radians(TARGET_PITCH_RATE),  # body pitch rate in rad/s (Not Use)
                math.radians(TARGET_YAW_RATE),  # body yaw rate in rad/s (Not Use)
                TARGET_THRUST)  # thrust

        # 模式三：使用偏航角速度控制
        else:
            msg = uav.message_factory.set_attitude_target_encode(
                0,  # time boot
                0,  # target system
                0,  # target component
                0b00000011,  # type mask: bit 1 is ignore
                to_quaternion(TARGET_ROLL, TARGET_PITCH, TARGET_YAW),  # quaternion
                math.radians(TARGET_ROLL_RATE),  # body roll rate in rad/s (Not Use)
                math.radians(TARGET_PITCH_RATE),  # body pitch rate in rad/s (Not Use)
                math.radians(TARGET_YAW_RATE),  # body yaw rate in rad/s (Activate)
                TARGET_THRUST)  # thrust

        # 傳輸命令
        uav.send_mavlink(msg)

        # 迴圈執行間隔
        time.sleep(1 / FREQ_CMD)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 命令傳輸程式：關閉')


# 影像擷取程式
def camera():
    global CAM_IMAGE, STATE_CAM, STATE_SYSTEM
    while CAM_VIDEO.isOpened() and STATE_SYSTEM:
        # 確認影像擷取狀態
        state_grab = CAM_VIDEO.grab()
        # 擷取成功
        if state_grab:
            # 擷取影像並儲存
            state_retrieve, image = CAM_VIDEO.retrieve()
            CAM_IMAGE = cv2.resize(image, (CAM_IMAGE_WEIGHT, CAM_IMAGE_HEIGHT))

            # 回報影像擷取程式啟動
            if not STATE_CAM:
                STATE_CAM = True
                print('<啟動程序> 影像擷取程式：啟動')

            # 顯示影像
            cv2.imshow("origin", CAM_IMAGE)

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
    CAM_VIDEO.release()                  # 關閉影像顯示
    cv2.destroyAllWindows()
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

    # 量測迴圈
    WAIT.acquire()
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
                print('<錯誤> 超音波感測器：量測超時')

        # 計算高度值
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * SONAR_AIR_TEMP)) / 2

        # 均值濾波器
        height_register.append(height)

        # 等待10ms執行下次量測
        time.sleep(0.1)
    WAIT.release()

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

    # 量測迴圈
    WAIT.acquire()
    for i in range(0, BAROM_SAMPLE_NUM_CAL):
        # 讀取氣壓計高度值
        height = uav.location.global_relative_frame.alt

        # 均值濾波器
        height_register.append(height)

        # 等待10ms執行下次量測
        time.sleep(0.1)
    WAIT.release()

    # 更新高度校正偏移量
    BAROM_OFFSET = -np.average(height_register)
    print('<啟動程序> 氣壓高度計校正完畢，偏移 %.3f m' % BAROM_OFFSET)


# 偏航角校正程序
def yaw_calibrate():
    global TARGET_YAW_OFFSET
    # 更新偏航角校正偏移量
    TARGET_YAW_OFFSET = -math.degrees(uav.attitude.yaw)
    print('<啟動程序> 偏航角校正完畢，偏移 %.3f degree' % TARGET_YAW_OFFSET)


# 影像計算程序
def img_calculate(color):
    # 色域轉換BGR->HSV
    image_hsv = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2HSV)
    # 閉運算(還有黑線就把矩陣加大)
    image_hsv = cv2.morphologyEx(image_hsv, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    # 依照目標色抓該顏色輪廓
    # 紅色
    if color == 'red':
        mask = cv2.inRange(image_hsv, IMG_RED_L, IMG_RED_H)
    # 綠色
    elif color == 'green':
        mask = cv2.inRange(image_hsv, IMG_GREEN_L, IMG_GREEN_H)
    # 紅色
    elif color == 'blue':
        mask = cv2.inRange(image_hsv, IMG_BLUE_L, IMG_BLUE_H)
    # 黑色
    else:
        mask = cv2.inRange(image_hsv, IMG_BLACK_L, IMG_BLACK_H)

    # 取得影像、輪廓點與索引
    image_c, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 計算型心與面積
    for c in contours:
        # 計算色塊面積
        image_area = cv2.contourArea(c)
        if image_area >= IMG_AREA_LIMIT:
            # 取得型心位置
            moment = cv2.moments(c)
            moment_x = int(moment["m10"] / (moment["m00"] + 0.001))
            moment_y = int(moment["m01"] / (moment["m00"] + 0.001))

            # 計算色塊位置
            block_x = moment_x - BODY_REF_X  # 色塊座標-畫面中央x座標 (右正左負)
            block_y = BODY_REF_Y - moment_y  # 畫面中央y座標-色塊座標 (上正下負)

            # 回傳XY座標、色塊面積
            return block_x, block_y, image_area

    return 0, 0, 0


# 角度計算
def angle_calculate():
    # 色域轉換BGR->GRAY
    img_gray = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    img_blur = cv2.GaussianBlur(img_gray, ANG_GB_KERNEL_SIZE, ANG_GB_SIGMA)
    # 二值化
    ret, img_th = cv2.threshold(img_blur, ANG_THRESHOLD, ANG_THRESHOLD_NEW, cv2.THRESH_BINARY_INV)

    # 切割區域
    img_shape = CAM_IMAGE.shape
    block = np.array(                                 # 設定切割界線
        [[(img_shape[1] * 0.1, img_shape[0] * 0.1),
          (img_shape[1] * 0.1, img_shape[0] * 0.5),
          (img_shape[1] * 0.9, img_shape[0] * 0.5),
          (img_shape[1] * 0.9, img_shape[0] * 0.1)]],
        dtype=np.int32)
    mask = np.zeros_like(img_th)                      # 零陣列遮罩
    cv2.fillPoly(mask, block, ANG_FILLED_COLOR)      # 多邊形填充切割區
    masked_edges = cv2.bitwise_and(img_th, mask)      # 二值與(and)判斷

    # 取得影像、輪廓點與索引
    image_c, contours, hierarchy = cv2.findContours(masked_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 設定型心列表
    moment_x_list = []
    moment_y_list = []

    for c in contours:
        moment = cv2.moments(c)
        moment_x = int(moment["m10"] / (moment["m00"] + 0.001))
        moment_y = int(moment["m01"] / (moment["m00"] + 0.001))
        area = cv2.contourArea(c)

        # 計算型心位置
        if ANG_LINE_TRACK_AREA_L < area <= ANG_LINE_TRACK_AREA_H:
            # 標出型心位置
            cv2.circle(masked_edges, (moment_x, moment_y), 5, (0, 0, 255), -1)
            # 儲存型心
            moment_x_list.append(moment_x)
            moment_y_list.append(moment_y)
        else:
            continue

    # 計算座標差
    x_different = (moment_x_list[-1] - BODY_REF_X + 0.01)
    y_different = moment_y_list[-1] - (BODY_REF_Y * 2)

    # 計算角度
    center = (len(moment_x_list) + len(moment_y_list)) / 2
    if 1 <= center <= 3:
        cv2.arrowedLine(CAM_IMAGE, (BODY_REF_X, BODY_REF_Y * 2), (moment_x_list[-1], moment_y_list[-1]),
                        (255, 255, 255), 3, 0, 0)
        angle = math.degrees(np.arctan(x_different / (y_different + 0.01)))

    else:
        cv2.arrowedLine(CAM_IMAGE, (BODY_REF_X, BODY_REF_Y * 2), (BODY_REF_X, BODY_REF_Y),
                        (255, 255, 255), 3, 0, 0)
        angle = 0

    return angle


# 顏色抵達偵測
def detect_arrive(color):
    # 初始化顏色面積
    img_area = 0

    # 色塊面積偵測
    while img_area < 6000:
        x_row, y_row, img_area = img_calculate(color)
        time.sleep(0.5)

    # 回報偵測到顏色
    print('<飛行狀態> 抵達目標色塊上方，顏色為', color)


# 顏色離開偵測
def detect_out(color):
    # 初始化顏色面積
    img_area = 100000

    # 色塊面積偵測
    while img_area > 6500:
        x_row, y_row, img_area = img_calculate(color)
        time.sleep(0.5)

    # 回報離開的顏色
    print('<飛行狀態> 已離開目標色塊上方，顏色為', color)


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


# 沙包盒關門
def box_close():
    # 初始化腳位
    GPIO.setmode(GPIO.BCM)                      # 設定為BCM模式
    GPIO.setwarnings(False)                     # 忽略警告
    GPIO.setup(BOX_PWM_PIN, GPIO.OUT)           # PWM腳位輸出設定
    box = GPIO.PWM(BOX_PWM_PIN, BOX_PWM_FREQ)   # 建立PWM控制物件

    # 啟動PWM控制，使門維持關閉
    dc = BOX_PWM_FREQ * (BOX_PWM_CLOSE / 1000000) * 100
    box.start(dc)
    print('<飛行狀態> 沙包盒艙門：關閉')


# 沙包盒開門
def box_open():
    # 初始化腳位
    GPIO.setmode(GPIO.BCM)                      # 設定為BCM模式
    GPIO.setwarnings(False)                     # 忽略警告
    GPIO.setup(BOX_PWM_PIN, GPIO.OUT)           # PWM腳位輸出設定
    box = GPIO.PWM(BOX_PWM_PIN, BOX_PWM_FREQ)   # 建立PWM控制物件

    # 啟動PWM控制，使門打開
    dc = BOX_PWM_FREQ * (BOX_PWM_OPEN / 1000000) * 100
    box.start(dc)
    print('<飛行狀態> 沙包盒艙門：開啟')


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
        print('<啟動程序> 主程序：等待命令傳輸啟動')
    while not STATE_CMD:
        time.sleep(0.1)

    # 初始化沙包盒，並關閉艙門
    box_close()
    print('<啟動程序> 沙包盒艙門關閉：完成')

    print('<啟動程序> 啟動前檢查程序：完成')


# 設定為定位模式
def pos_mode(color):
    global MODE, POS_COLOR
    MODE = 'POS'             # 模式旗標切為定位
    POS_COLOR = color        # 定位目標顏色設置
    # 回報模式啟用狀態
    print('<飛行狀態> 切換至定位模式，顏色追蹤：', POS_COLOR)


# 設定為循跡模式
def line_mode():
    global MODE, POS_COLOR
    MODE = 'LINE'            # 模式旗標切為循跡
    POS_COLOR = ''           # 清空顏色目標
    # 回報模式啟用狀態
    print('<飛行狀態> 切換至循跡模式')


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

    # 設定為定位模式(定位黑色)
    pos_mode('black')

    # 設定目標高度
    print('<飛行命令> 起飛至目標高度 %.2f m' % target_alt)
    TARGET_ALT = target_alt

    # 等待抵達目標高度
    while ACTUAL_ALT < TARGET_ALT * TAKEOFF_SETTLE:
        print('<飛行狀態> 起飛中，目前高度 %.2f m，距離目標高度尚有 %.2f m' % (ACTUAL_ALT, TARGET_ALT-ACTUAL_ALT))
        time.sleep(1)

    # 回報抵達目標高度
    print('<飛行狀態> 已抵達目標高度 %.2f m' % TARGET_ALT)


# 懸停程序(無定位)
def hover(duration):
    # 設定飛行命令
    duration = int(duration)
    print('<飛行命令> 懸停於 %.2f m，維持 %d sec' % (TARGET_ALT, duration))

    # 等待時間到達
    for i in range(1, duration+1):
        print('<飛行狀態> 懸停中，目前高度 %.2f m，剩餘時間 %d sec' % (ACTUAL_ALT, duration + 1 - i))
        time.sleep(1)

    # 回報懸停結束
    print('<飛行狀態> 懸停狀態結束，目前高度 %.2f m' % ACTUAL_ALT)


# 往前飛行
def flight_forward():
    global TARGET_PITCH
    line_mode()              # 切換至循跡模式
    TARGET_PITCH = 5         # 以固定傾角往前飛行
    print('<飛行狀態> 開始往前飛行')


# 轉向控制
def turn():
    global TARGET_YAW_RATE, MISSION_COLOR
    while 1:      # 藍右，綠左
        # 讀取顏色面積值做判斷
        block_x, block_y, image_area_blue = img_calculate('blue')
        block_x, block_y, image_area_green = img_calculate('green')

        # 讀取偏航角
        yaw_start = math.degrees(uav.attitude.yaw)
        yaw_current = math.degrees(uav.attitude.yaw)

        # 若偵測到藍色
        if image_area_blue >= 0:
            MISSION_COLOR = 'blue'
            print('<飛行狀態> 偵測到藍色為任務顏色')
            # 右轉
            print('<飛行命令> 往右旋轉90度')
            TARGET_YAW_RATE = 10
            while math.fabs(yaw_current - yaw_start) < 90 * 0.95:
                yaw_current = math.degrees(uav.attitude.yaw)
            TARGET_YAW_RATE = 0
            break

        # 若偵測到綠色
        elif image_area_green >= 0:
            MISSION_COLOR = 'green'
            print('<飛行狀態> 偵測到綠色為任務顏色')
            # 左轉
            print('<飛行命令> 往左旋轉90度')
            TARGET_YAW_RATE = -10
            while math.fabs(yaw_current - yaw_start) < 90 * 0.95:
                yaw_current = math.degrees(uav.attitude.yaw)
            TARGET_YAW_RATE = 0
            break
        time.sleep(1/FREQ_CAM)
    print('<飛行狀態> 轉向完成')


# 空投程序
def drop():
    # 切換定位模式
    pos_mode(MISSION_COLOR)

    # 等待移至降落點上方
    print('<飛行命令> 移動至空投點上方')
    while 1:
        block_x, block_y, image_area = img_calculate(MISSION_COLOR)
        if -DROP_POS_LIMIT < block_x < DROP_POS_LIMIT:
            if -DROP_POS_LIMIT < block_y < DROP_POS_LIMIT:
                time.sleep(1)
                break

    # 打開沙包盒空投
    print('<飛行命令> 位置已確認，執行空投')
    box_open()
    time.sleep(2)
    print('<飛行狀態> 沙包已成功空投')

    # 關閉沙包盒
    box_close()
    print('<飛行狀態> 沙包盒已復位')


# 降落程序
def landing():
    global TARGET_ALT
    # 切換定位模式
    pos_mode(MISSION_COLOR)

    # 等待移至降落點上方
    print('<飛行命令> 移動至降落點上方')
    while 1:
        block_x, block_y, image_area = img_calculate(MISSION_COLOR)
        if -LANDING_POS_LIMIT < block_x < LANDING_POS_LIMIT:
            if -LANDING_POS_LIMIT < block_y < LANDING_POS_LIMIT:
                time.sleep(1)
                break

    # 設定目標高度
    print('<飛行命令> 位置已確認，開始降落至地面')
    TARGET_ALT = -0.1           # 負值以確保不會著地後彈升

    # 等待抵達目標高度
    while ACTUAL_ALT > LANDING_CUTOFF:
        print('<飛行狀態> 降落中，目前高度 %.2f m' % ACTUAL_ALT)
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
    # 起飛與紅燈區
    take_off(1)                         # 起飛
    flight_forward()                    # 往前循跡飛行
    detect_out('red')                   # 偵測已離開起飛區
    detect_arrive('red')                # 偵測抵達紅燈區
    pos_mode('red')                     # 紅燈上定位

    # 循線A區
    turn()                              # 偵測到顏色後轉向
    flight_forward()                    # 往前循跡飛行
    detect_out(MISSION_COLOR)           # 偵測已離開號誌區

    # 投擲與循線B區
    detect_arrive(MISSION_COLOR)        # 偵測抵達投擲色塊上方
    drop()                              # 空投程序
    flight_forward()                    # 往前循跡飛行
    detect_out(MISSION_COLOR)           # 偵測已離開投擲區

    # T路口與降落區
    detect_arrive(MISSION_COLOR)        # 偵測抵達降落區
    landing()                           # 執行降落

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
    hdg = threading.Thread(target=heading_controller)
    cmd = threading.Thread(target=command_transfer)
    m = threading.Thread(target=main)

    # 執行多線程(依啟動順序排放)
    print('<啟動程序> 多線程程序：啟動')
    sonar_sen.start()
    alt_sen.start()
    alt.start()
    cam.start()
    pos.start()
    hdg.start()
    cmd.start()
    m.start()

    # 等待多線程結束(依結束順序排放)
    m.join()
    cmd.join()
    hdg.join()
    pos.join()
    cam.join()
    alt.join()
    alt_sen.join()
    sonar_sen.join()
    print('<關閉程序> 多線程程序：關閉')


# ======數值紀錄區塊===================================================
def show_data():
    # ======高度控制器================
    # 高度控制器響應圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT_CTRL) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT_CTRL, label="Actual Height")
    plt.plot(num_alt, R_TARGET_ALT, label="Target Altitude")
    plt.plot(num_alt, R_TARGET_THRUST, label="Target Thrust")

    # naming the x axis
    plt.ylabel('Height(m)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Altitude Controller Data')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Altitude Controller Data.png')
    # function to show the plot
    plt.show()

    # 高度控制器PID響應圖
    num_alt = []
    for i in range(1, len(R_ALT_ERROR) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ALT_ERROR, label="Error")
    plt.plot(num_alt, R_ALT_GP, label="Gain P")
    plt.plot(num_alt, R_ALT_GI, label="Gain I")
    plt.plot(num_alt, R_ALT_GD, label="Gain D")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Altitude Controller PID Data')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Altitude Controller PID Data.png')
    # function to show the plot
    plt.show()

    # ======位置控制器=========================
    # 位置控制器X響應圖
    num_pos = []
    for i in range(1, len(R_BLOCK_X) + 1):
        num_pos.append(i)

    plt.plot(num_pos, R_BLOCK_X_ROW, label="X Position - Row")
    plt.plot(num_pos, R_BLOCK_X, label="X Position")
    plt.plot(num_pos, R_TARGET_ROLL, label="Target Roll")
    plt.plot(num_pos, R_ACTUAL_ROLL, label="Actual Roll")

    # naming the x axis
    plt.ylabel('Position(px), Angle(degree)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Position Controller Data - X axis')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Position Controller Data - X axis.png')
    # function to show the plot
    plt.show()

    # 位置控制器Y響應圖
    num_pos = []
    for i in range(1, len(R_BLOCK_Y) + 1):
        num_pos.append(i)

    plt.plot(num_pos, R_BLOCK_Y_ROW, label="Y Position - Row")
    plt.plot(num_pos, R_BLOCK_Y, label="Y Position")
    plt.plot(num_pos, R_TARGET_PITCH, label="Target Pitch")
    plt.plot(num_pos, R_ACTUAL_PITCH, label="Actual Pitch")

    # naming the x axis
    plt.ylabel('Position(px), Angle(degree)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Position Controller Data - Y axis')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Position Controller Data - Y axis.png')
    # function to show the plot
    plt.show()

    # 位置控制器軌跡響應圖
    plt.plot(R_BLOCK_X, R_BLOCK_Y, label="Block Position")
    plt.plot(R_TARGET_ROLL, R_TARGET_PITCH, label="Target Attitude")
    plt.plot(R_ACTUAL_ROLL, R_ACTUAL_PITCH, label="Actual Attitude")

    # naming the x axis
    plt.ylabel('X-Position(px), Angle(degree)')
    # naming the y axis
    plt.xlabel('Y-Position(px), Angle(degree)')
    # giving a title to my graph
    plt.title('Position Controller Data')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Position Controller Data.png')
    # function to show the plot
    plt.show()

    # ======航向控制器===========================================
    # 航向控制器響應圖
    num_hdg = []
    for i in range(1, len(R_HDG_ERROR) + 1):
        num_hdg.append(i)

    plt.plot(num_hdg, R_HDG_ERROR, label="Heading Error")
    plt.plot(num_hdg, R_TARGET_YAW_RATE, label="Target Yaw Rate")

    # naming the x axis
    plt.ylabel('Angle(degree), Angular Rate(degree/s)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Heading Controller Data')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Heading Controller Data - .png')
    # function to show the plot
    plt.show()

    # 航向控制器PID響應圖
    num_hdg = []
    for i in range(1, len(R_HDG_ERROR) + 1):
        num_hdg.append(i)

    plt.plot(num_hdg, R_HDG_ERROR, label="Error")
    plt.plot(num_hdg, R_HDG_GP, label="Gain P")
    plt.plot(num_hdg, R_HDG_GI, label="Gain I")
    plt.plot(num_hdg, R_HDG_GD, label="Gain D")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Heading Controller PID Data')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Heading Controller PID Data.png')
    # function to show the plot
    plt.show()

    # ======高度感測器================================
    # 高度感測器數值處理圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT, label="Actual Height")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR, label="Sonar Data")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM, label="Barometer Data")

    # naming the x axis
    plt.ylabel('Height(m)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Altitude Data')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Altitude Data.png')
    # function to show the plot
    plt.show()

    # 超音波高度計數值處理圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT_SONAR_KF) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_ROW, label="Row Data")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_OFFSET, label="Angel & Height Offset")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_AVE, label="Average Filter")
    plt.plot(num_alt, R_ACTUAL_ALT_SONAR_KF, label="Kalman Filter")

    # naming the x axis
    plt.ylabel('Height(m)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Altitude Data - Sonar')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Altitude Data - Sonar.png')
    # function to show the plot
    plt.show()

    # 氣壓高度計數值處理圖
    num_alt = []
    for i in range(1, len(R_ACTUAL_ALT_BAROM_ROW) + 1):
        num_alt.append(i)

    plt.plot(num_alt, R_ACTUAL_ALT_BAROM_ROW, label="Row Data")
    plt.plot(num_alt, R_ACTUAL_ALT_BAROM, label="Average Filter")

    # naming the x axis
    plt.ylabel('Height(m)')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Altitude Data - Barometer')
    # show a legend on the plot
    plt.legend()
    # function to save the plot
    plt.savefig('Altitude Data - Barometer.png')
    # function to show the plot
    plt.show()


# ======程式執行區======
thread_procedure()
show_data()
