

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# 載入模組
from __future__ import print_function
from dronekit import connect
import matplotlib.pyplot as plt
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
R_BLOCK_AREA = [0.0]                  # 記錄面積變化
R_RECORD_SCALE = 1000                 # 面積縮放比
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
# 位置控制器(原始版)
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
            R_BLOCK_AREA.append(block_area/R_RECORD_SCALE)

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

            # 判斷與輸出姿態命令
            # X座標
            if block_x > POS_DZ_X:                 # 向右飛行
                TARGET_ROLL = POS_ANGLE_LIMIT
            elif block_x < -POS_DZ_X:              # 向左飛行
                TARGET_ROLL = -POS_ANGLE_LIMIT
            else:                                # 定位作動忽略區間
                TARGET_ROLL = 0
            # Y座標
            if block_y > POS_DZ_Y:                 # 向前飛行
                TARGET_PITCH = -POS_ANGLE_LIMIT
            elif block_y < -POS_DZ_Y:              # 向後飛行
                TARGET_PITCH = POS_ANGLE_LIMIT
            else:                                # 定位作動忽略區間
                TARGET_PITCH = 0

            # 記錄目標輸出命令
            R_TARGET_ROLL.append(TARGET_ROLL)
            R_TARGET_PITCH.append(TARGET_PITCH)

            # 記錄實際轉角
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
        R_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
        R_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

        # 迴圈執行間隔
        time.sleep(1 / FREQ_POS)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_SYSTEM:
            print('<飛行狀態> 位置控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 位置控制器：關閉')


# 位置控制器(改良版)
# 更動部分：無訊號時重複前一動，採行比例式級距，設定小範圍切斷區
def position_controller_2():
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
            R_BLOCK_AREA.append(block_area/R_RECORD_SCALE)

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

            # 判斷與輸出姿態命令(更換成級距式)
            # X座標
            if block_x > POS_DZ_OUT_X:                      # 向右最快飛行
                TARGET_ROLL = POS_ANGLE_LIMIT

            elif block_x < -POS_DZ_OUT_X:                   # 向左最快飛行
                TARGET_ROLL = -POS_ANGLE_LIMIT

            elif POS_DZ_IN_X <= block_x <= POS_DZ_OUT_X:    # 向右比例飛行
                percent_x = POS_DZ_OUT_X - block_x / POS_DZ_OUT_X - POS_DZ_IN_X
                TARGET_ROLL = POS_ANGLE_LIMIT * percent_x

            elif -POS_DZ_IN_X >= block_x >= -POS_DZ_OUT_X:  # 向左比例飛行
                percent_x = POS_DZ_OUT_X + block_x / POS_DZ_OUT_X - POS_DZ_IN_X
                TARGET_ROLL = -POS_ANGLE_LIMIT * percent_x

            else:                                           # 定位作動忽略區間
                TARGET_ROLL = 0

            # X座標
            if block_y > POS_DZ_OUT_Y:  # 向右最快飛行
                TARGET_ROLL = POS_ANGLE_LIMIT

            elif block_y < -POS_DZ_OUT_Y:  # 向左最快飛行
                TARGET_ROLL = -POS_ANGLE_LIMIT

            elif POS_DZ_IN_Y <= block_y <= POS_DZ_OUT_Y:  # 向右比例飛行
                percent_y = POS_DZ_OUT_Y - block_y / POS_DZ_OUT_Y - POS_DZ_IN_Y
                TARGET_ROLL = POS_ANGLE_LIMIT * percent_y

            elif -POS_DZ_IN_Y >= block_y >= -POS_DZ_OUT_Y:  # 向左比例飛行
                percent_y = POS_DZ_OUT_Y + block_y / POS_DZ_OUT_Y - POS_DZ_IN_Y
                TARGET_ROLL = -POS_ANGLE_LIMIT * percent_y

            else:  # 定位作動忽略區間
                TARGET_ROLL = 0

            # 記錄目標輸出命令
            R_TARGET_ROLL.append(TARGET_ROLL)
            R_TARGET_PITCH.append(TARGET_PITCH)

            # 記錄實際轉角
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
        R_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
        R_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

        # 迴圈執行間隔
        time.sleep(1 / FREQ_POS)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_SYSTEM:
            print('<飛行狀態> 位置控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 位置控制器：關閉')


# ======子函式區塊==============================================================
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
