# 影像循跡測試
# 包含影像擷取、航向控制器

# 0909 測試通過

# 調整新式資料流程：分拆角度計算成兩個函式

# 0911增加滾轉輔助轉向

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# 載入模組
from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import threading
import math
import time
import cv2

# ======設定全域變數======
print('<啟動程序> 載入全域變數設定')

# ======旗標與變數======================================================
# 旗標
STATE_SYSTEM = True
STATE_HDG = False
STATE_CAM = False
MODE = ''                # 模式旗標

# 多線程優先執行
THREAD_OCCUPY = threading.Lock()

# 目標值(命令值)
TARGET_PITCH = 0         # 目標俯仰角(degree)
TARGET_ROLL = 0          # 目標滾轉角(degree)
TARGET_YAW_RATE = 0      # 偏航角速度(degree/s)，需搭配控制器控制角度

# 實際值(量測值)
ACTUAL_HDG_ERR = 0              # 中線夾角

# ======控制器參數設定===================================================
# 控制器迴圈頻率
FREQ_HDG = 30                   # 航向控制器頻率(不快於影像禎率)
FREQ_CAM = 30                   # 影像禎率(fps，依系統負載調整)

# 航向控制器參數
HDG_KP = 1                      # P增益值
HDG_KI = 0                      # I增益值
HDG_KD = 0                      # D增益值
HDG_DZ = 5                      # 航向安定區間(度，單向)
HDG_SAMPLE_NUM_AVE = 10         # 角度差均值濾波取樣次數
HDG_YAW_RATE_LIMIT = 10         # 角速度限制
HDG_YAW_ROLL_RATIO = 0.3        # 滾轉輔助轉向比率
HDG_ROLL_LIMIT = 4              # 滾轉角限制(degree)

# ======影像參數設定=============================================================
# 相機設定參數
CAM_VIDEO = cv2.VideoCapture(0)     # 影像來源(預設來源為0)
CAM_IMAGE = 0                       # 儲存影像
CAM_IMAGE_WEIGHT = 480              # 影像寬度(px，4:3設定 )
CAM_IMAGE_HEIGHT = 360              # 影像高度(px)
VIDEO_FOURCC = cv2.VideoWriter_fourcc(*'XVID')
VIDEO_SAVE = cv2.VideoWriter('Heading Controller Data.avi',
                             VIDEO_FOURCC, FREQ_CAM, (CAM_IMAGE_WEIGHT, CAM_IMAGE_HEIGHT))

# 影像計算參數
BODY_REF_X = int(CAM_IMAGE_WEIGHT / 2)     # 畫面參考X座標中心
BODY_REF_Y = int(CAM_IMAGE_HEIGHT / 2)     # 畫面位置Y座標中心
BODY_OFFSET_X = 0                          # 機身中心X偏移量(px，左負右正)
BODY_OFFSET_Y = 0                          # 機身中心Y偏移量(px，上正下負)

# 線角度計算
ANG_THRESHOLD = 40              # 二值化門檻值(調高比較能看到線，但可能有雜訊)
ANG_THRESHOLD_NEW = 255         # 取得黑白照填入最高值
ANG_GB_KERNEL_SIZE = (7, 7)     # 高斯模糊矩陣大小(越大越模糊)
ANG_GB_SIGMA = 2                # 高斯模糊色域標準差(越大越模糊)
ANG_FILLED_COLOR = 255          # 界線填充顏色
ANG_LINE_TRACK_AREA_L = 450     # 線追蹤型心計算面積下限

# ======數值記錄=================================================================
# 航向控制器
R_HDG_ERROR_ROW = []
R_HDG_ERROR = []
R_TARGET_YAW_RATE = []
R_HDG_TARGET_ROLL = []
R_HDG_GP = []                 # 記錄P增益
R_HDG_GI = []                 # 記錄I增益
R_HDG_GD = []                 # 記錄D增益

print('<啟動程序> 載入全域變數設定完畢')


# ======背景主函式函式區塊=============================================================
# 航向控制器
def heading_controller():
    global ACTUAL_HDG_ERR, TARGET_ROLL, TARGET_PITCH, TARGET_YAW_RATE, STATE_HDG
    # 變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 均值濾波器設定
    block_x_register = []
    block_y_register = []
    hdg_register = []

    # 等待相機啟動
    if not STATE_CAM:
        print('<啟動程序> 位置控制器：等待影像擷取程式啟動')
    while not STATE_CAM:
        time.sleep(0.1)

    # 回報定位控制器啟用
    STATE_HDG = True
    print('<啟動程序> 航向控制器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_SYSTEM:
        while MODE == 'LINE' and STATE_SYSTEM:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 取得型心位置
            block_x_row, block_y_row = img_moment()

            # 型心位置均值濾波器
            # X座標均值濾波器
            block_x_register.insert(0, block_x_row)
            if len(block_x_register) > HDG_SAMPLE_NUM_AVE:
                block_x_register.pop()
            block_x = np.average(block_x_register)

            # Y座標均值濾波器
            block_y_register.insert(0, block_y_row)
            if len(block_y_register) > HDG_SAMPLE_NUM_AVE:
                block_y_register.pop()
            block_y = np.average(block_y_register)

            # 取得現在角度誤差值(中心為目標值，右斜為負，左斜為正)
            error_row = angle_calculate(block_x, block_y)
            R_HDG_ERROR_ROW.append(error_row)

            # 誤差角均值濾波器
            hdg_register.insert(0, error_row)
            if len(hdg_register) > HDG_SAMPLE_NUM_AVE:
                hdg_register.pop()
            error = np.average(hdg_register)
            ACTUAL_HDG_ERR = error
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

            # 輸出滾轉做轉向輔助(未驗證)
            roll = angular_rate * HDG_YAW_ROLL_RATIO
            if roll > HDG_ROLL_LIMIT:
                roll = HDG_ROLL_LIMIT
            elif roll < -HDG_ROLL_LIMIT:
                roll = -HDG_ROLL_LIMIT
            TARGET_ROLL = roll
            R_HDG_TARGET_ROLL.append(TARGET_ROLL)

            # 傳遞值給下次迴圈使用
            previous_error = error
            previous_time = current_time

            # 同捆執行終點
            THREAD_OCCUPY.release()

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
        R_HDG_TARGET_ROLL.append(0)

        # 迴圈執行間隔
        time.sleep(1 / FREQ_HDG)

        # 當飛行模式為循跡時，跳入迴圈並啟動函式
        if MODE == 'LINE' and STATE_SYSTEM:
            print('<飛行狀態> 航向控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 航向控制器：關閉')


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

            # 顯示影像
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
    print('<關閉程序> 影像擷取程式：關閉')


# ======子函式區塊==============================================================
# 二值化後型心計算
def img_moment():
    # 色域轉換BGR->GRAY
    img_gray = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    img_blur = cv2.GaussianBlur(img_gray, ANG_GB_KERNEL_SIZE, ANG_GB_SIGMA)
    # 二值化
    ret, img_th = cv2.threshold(img_blur, ANG_THRESHOLD, ANG_THRESHOLD_NEW, cv2.THRESH_BINARY_INV)
    # 取得影像、輪廓點與索引
    image_c, contours, hierarchy = cv2.findContours(img_th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
                moment_x_row = int(moment["m10"] / moment["m00"])
                moment_y_row = int(moment["m01"] / moment["m00"])
            else:
                moment_x_row = 0
                moment_y_row = 0

            # 計算色塊位置(相對於畫面下方中心)
            block_x_offset = moment_x_row - BODY_REF_X - BODY_OFFSET_X
            block_y_offset = 2 * BODY_REF_Y - moment_y_row - BODY_OFFSET_Y

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


# 角度計算
def angle_calculate(block_x, block_y):
    # 計算與中心線夾角
    if block_y == 0 and block_x == 0:
        angle = 0
    elif block_y == 0 and block_x > 0:
        angle = 90
    elif block_y == 0 and block_x < 0:
        angle = -90
    else:
        angle = math.degrees(math.atan(block_x/block_y))

    return angle


# ======飛行程序指令(條件與命令)======================================================
# 啟動前檢查
def pre_check():
    print('<啟動程序> 啟動前檢查程序：啟動')

    # 等待位置控制器啟動
    if not STATE_HDG:
        print('<啟動程序> 主程序：等待位置控制器啟動')
    while not STATE_HDG:
        time.sleep(0.1)

    print('<啟動程序> 啟動前檢查程序：完成')


# 飛行模式切換
def flight_mode(mode):
    global MODE
    # 模式切換
    MODE = mode
    print('<飛行狀態> 切換至循線模式')


# ======前景主函式區塊=================================================================
# 任務主函式
def main():
    global STATE_SYSTEM
    # 啟動前檢查，回報主程式啟動
    print('<啟動程序> 主程序：啟動')

    # 飛行流程
    flight_mode('LINE')
    for i in range(0, 20):
        print('差角%.2f度，角速度%.2f度/秒' % (ACTUAL_HDG_ERR, TARGET_YAW_RATE))
        time.sleep(1)

    # 關閉背景多線程
    STATE_SYSTEM = False
    print('<關閉程序> 主程序：關閉')


# 多線程函式
def thread_procedure():
    # 定義多線程
    cam = threading.Thread(target=camera)
    hdg = threading.Thread(target=heading_controller)
    m = threading.Thread(target=main)

    # 執行多線程(依啟動順序排放)
    print('<啟動程序> 多線程程序：啟動')
    cam.start()
    hdg.start()
    m.start()

    # 等待多線程結束(依結束順序排放)
    m.join()
    hdg.join()
    cam.join()
    print('<關閉程序> 多線程程序：關閉')


# ======數值紀錄區塊===================================================
def show_data():
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


# ======程式執行區======
thread_procedure()
show_data()
