# 影像定位測試
# 包含影像擷取、位置控制器
# 預定0910測試，0911更改條件(輸出問題未驗證)

# 0909新增
# 設定同捆執行與即時回報位置
# 型心計算新增多輪廓計算

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# 載入模組
from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
import cv2

# ======設定全域變數======
print('<啟動程序> 載入全域變數設定')

# ======旗標與變數======================================================
# 旗標
STATE_SYSTEM = True
STATE_POS = False
STATE_CAM = False
MODE = ''                    # 模式旗標

# 多線程優先執行
THREAD_OCCUPY = threading.Lock()

# 目標值(命令值)
TARGET_ROLL = 0             # 目標滾轉角(degree)
TARGET_PITCH = 0            # 目標俯仰角(degree)

# 實際值(量測值)
ACTUAL_POS = [0.0, 0.0]     # 色塊實際位置

# ======控制器參數設定===================================================
# 迴圈頻率
FREQ_POS = 30               # 高度控制器頻率(不快於影像禎率)
FREQ_CAM = 30               # 影像禎率(fps，依系統負載調整)

# 位置控制器參數
POS_COLOR = 'black'                 # 定位的色塊顏色(預設黑色)
POS_SAMPLE_NUM_AVE = 10             # 色塊座標均值濾波取樣次數
POS_DZ_IN_X = 10                    # 色塊定位作動忽略區間(單向，px)
POS_DZ_IN_Y = POS_DZ_IN_X*(3/4)     # 色塊定位作動忽略區間(單向，px)
POS_DZ_OUT_X = 200                  # 外側範圍，固定輸出下限(單向，px)
POS_DZ_OUT_Y = POS_DZ_OUT_X*(3/4)   # 外側範圍，固定輸出下限(單向，px)
POS_ANGLE_LIMIT = 5                 # 定位控制最大飛行角度(degree)


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
IMG_AREA_LIMIT = 6000                      # 取用面積下限值
BODY_REF_X = int(CAM_IMAGE_WEIGHT / 2)     # 畫面參考X座標中心
BODY_REF_Y = int(CAM_IMAGE_HEIGHT / 2)     # 畫面位置Y座標中心
BODY_OFFSET_X = 0                          # 機身中心X偏移量(px，左負右正)
BODY_OFFSET_Y = 0                          # 機身中心Y偏移量(px，上正下負)

# ======數值記錄=================================================================
# 位置控制器
R_TARGET_ROLL = []            # 記錄目標滾轉角
R_TARGET_PITCH = []           # 記錄目標俯仰角
R_BLOCK_X_ROW = []            # 記錄X座標原始值
R_BLOCK_Y_ROW = []            # 記錄Y座標原始值
R_BLOCK_AREA = []             # 記錄面積變化
R_RECORD_SCALE = 1000         # 面積縮放比
R_BLOCK_X = []                # 記錄X座標
R_BLOCK_Y = []                # 記錄Y座標

print('<啟動程序> 載入全域變數設定完畢')


# ======背景主函式函式區塊=============================================================
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

        # 迴圈執行間隔
        time.sleep(1 / FREQ_POS)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_SYSTEM:
            print('<飛行狀態> 位置控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    print('<關閉程序> 位置控制器：關閉')


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

    # 等待位置控制器啟動
    if not STATE_POS:
        print('<啟動程序> 主程序：等待位置控制器啟動')
    while not STATE_POS:
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


# ======前景主函式區塊=================================================================
# 任務主函式
def main():
    global STATE_SYSTEM
    # 啟動前檢查，回報主程式啟動
    pre_check()
    print('<啟動程序> 主程序：啟動')

    # 程式流程
    for i in range(20):
        # 回報座標位置
        print('中心座標 = (%.2f , %.2f)px' % (ACTUAL_POS[0], ACTUAL_POS[1]))
        time.sleep(0.5)

    # 關閉背景多線程
    STATE_SYSTEM = False
    print('<關閉程序> 主程序：關閉')


# 多線程函式
def thread_procedure():
    # 定義多線程
    cam = threading.Thread(target=camera)
    pos = threading.Thread(target=position_controller)
    m = threading.Thread(target=main)

    # 執行多線程(依啟動順序排放)
    print('<啟動程序> 多線程程序：啟動')
    input_color()                  # 與多線程開始前設定顏色，防止記錄錯誤
    cam.start()
    pos.start()
    m.start()

    # 等待多線程結束(依結束順序排放)
    m.join()
    pos.join()
    cam.join()
    print('<關閉程序> 多線程程序：關閉')


# ======數值紀錄區塊======
def show_data():
    # ======位置控制器================
    # 位置控制器X響應圖
    num_pos = []
    for i in range(1, len(R_BLOCK_X) + 1):
        num_pos.append(i)

    plt.plot(num_pos, R_BLOCK_X_ROW, label="X Position - Row")
    plt.plot(num_pos, R_BLOCK_X, label="X Position")
    plt.plot(num_pos, R_TARGET_ROLL, label="Target Roll")
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

    plt.xlabel('X-Position(px), Angle(degree)')
    plt.ylabel('Y-Position(px), Angle(degree)')
    plt.title('Position Controller Track Data')
    plt.legend()
    plt.savefig('Position Controller Track Data.png')
    plt.show()


# ======程式執行區======
thread_procedure()
show_data()
