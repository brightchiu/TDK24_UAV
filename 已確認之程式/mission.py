# 任務程式

# 載入模組
from __future__ import print_function
from dronekit import connect, VehicleMode
from openpyxl import Workbook
# import dronekit_sitl
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math
import cv2

# ========== 旗標與變數 ================================================
# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_UAV = True                # 無人機連線狀態
STATE_ALT = False               # 高度控制器旗標
STATE_POS = False               # 位置控制器旗標
STATE_VEL = False               # 位置-速度控制器旗標
STATE_HDG = False               # 航向控制器旗標
STATE_SPD = False               # 飛行速度控制器旗標
STATE_SENSOR_SPD = False        # 飛行速度感測器旗標
STATE_SENSOR_ALT = False        # 高度感測器旗標
STATE_SENSOR_SONAR = False      # 超音波高度計旗標
STATE_SENSOR_CAM = False        # 影像感測器旗標
STATE_CMD = False               # 命令傳輸程式旗標
STATE_CAM = False               # 影像擷取程式旗標
STATE_BOX = False               # 貨物艙門狀態旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)
MISSION_COLOR = ''              # 任務顏色(blue or green)
POS_COLOR = ''                  # 定位的色塊顏色

# ========== UAV通訊參數 ==============================================
# 樹莓派Serial連線參數
CONNECT_PORT = '/dev/serial0'   # 傳輸口位址
CONNECT_BAUD = 921600           # 傳輸鮑率

# ========== 與UAV執行連線 =============================================
print('<啟動程序> 與UAV連線中，請稍候...')
# 樹莓派連線指令
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)


# ====================================================================
# ========== 飛行程序指令(條件與命令) ====================================
# ====================================================================
# 任務中會使用的區段飛行命令、模式切換命令、觸發條件

# ========== 啟動前檢查 ================================================
# 本程式為飛行前的安全檢查項目，除確認控制器是否上線外，也提醒控制員檢查安全程序
# 項目有：安全裝置移除(槳片銷)、起飛區淨空，完成後即交付起飛控制
def pre_check():
    # 執行起飛前檢查
    print('<啟動程序> 啟動前檢查程序：啟動')

    # 階段一：控制器檢查
    # 等待高度控制器啟動
    if not STATE_ALT:
        print('<啟動程序> 主程序：等待高度控制器啟動')
    while not STATE_ALT:
        time.sleep(0.1)

    # 等待命令傳輸啟動
    if not STATE_CMD:
        print('<啟動程序> 主程序：等待命令傳輸啟動')
    while not STATE_CMD:
        time.sleep(0.1)

    box_initialize()

    # 階段三：安全檢查程序
    # 確認安全裝置解除
    safe_pin = 'N'
    while safe_pin != 'Y':
        safe_pin = input('<啟動程序> 請確認安全裝置已移除(Y/N) => ')
    print('<啟動程序> 安全裝置已移除')
    time.sleep(1)

    # 確認起飛區已淨空
    lunch_clear = 'N'
    while lunch_clear != 'Y':
        lunch_clear = input('<啟動程序> 請確認起飛區已淨空(Y/N) => ')
    print('<啟動程序> 起飛區已淨空')

    # 回報起飛前檢點完成
    print('<啟動程序> 啟動前檢查程序：完成')
    time.sleep(1)

    # 任務開始確認
    go_or_nogo = 'N'
    while go_or_nogo != 'Y':
        go_or_nogo = input('<啟動程序> 請求任務開始(Y/N) => ')

    # 回報任務執行開始
    print('<啟動程序> TDK飛行任務，開始!')
    time.sleep(1)


# ========== 起飛程序 =================================================
# 執行模式切換、無人機解鎖，並由高度控制器將無人機送至目標飛行高度
def take_off(target_alt):
    global TARGET_ALT, MODE
    # 切換至無衛星導航模式
    uav.mode = VehicleMode("GUIDED_NOGPS")
    while not uav.mode.name == 'GUIDED_NOGPS':
        time.sleep(0.5)
    print('<啟動程序> 無人機已切換至無衛星導航模式')

    # 解鎖無人機
    uav.arm(wait=True)
    print('<啟動程序> 無人機已解鎖')

    # 設定飛行模式
    pos_mode('black')

    # 設定目標高度
    print('<飛行命令> 起飛至目標高度 %.2f m' % target_alt)
    TARGET_ALT = target_alt

    # 等待抵達目標高度
    while ACTUAL_ALT < TARGET_ALT * (TAKEOFF_SETTLE / 100):
        # 顯示飛行資訊
        print('<飛行狀態> 起飛中，目前高度 %.2f m，距離目標高度尚有 %.2f m' %
              (ACTUAL_ALT, TARGET_ALT - ACTUAL_ALT))
        print('<高度感測器狀態> 超音波：%.2f m，IMU：%.2f m' % (ACTUAL_ALT_SONAR, ACTUAL_ALT_IMU))
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px，色塊面積：%d px^2' %
              (ACTUAL_POS[0], ACTUAL_POS[1], ACTUAL_AREA))
        print('<飛行狀態> 定位中，相對速度(%.2f , %.2f)px/s\n' %
              (ACTUAL_VEL[0], ACTUAL_VEL[1]))
        time.sleep(0.5)

        if not STATE_THREAD:
            break

    # 回報抵達目標高度
    print('<飛行狀態> 已抵達目標高度 %.2f m' % TARGET_ALT)


# ========== 懸停程序 =================================================
def hover(duration):
    # 設定飛行命令
    duration = int(duration)
    print('<飛行命令> 懸停於 %.2f m，維持 %d sec' % (TARGET_ALT, duration))

    # 等待時間到達
    for i in range(duration):
        # 顯示飛行資訊
        print('<飛行狀態> 懸停中，目前高度 %.2f m，剩餘時間 %d sec' % (ACTUAL_ALT, duration - i))
        print('<高度感測器狀態> 超音波：%.2f m，IMU：%.2f m' % (ACTUAL_ALT_SONAR, ACTUAL_ALT_IMU))
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px，色塊面積：%d px^2' %
              (ACTUAL_POS[0], ACTUAL_POS[1], ACTUAL_AREA))
        print('<飛行狀態> 定位中，相對速度(%.2f , %.2f)px/s\n' %
              (ACTUAL_VEL[0], ACTUAL_VEL[1]))
        time.sleep(0.5)

    # 回報懸停結束
    print('<飛行狀態> 懸停狀態結束，目前高度 %.2f m' % ACTUAL_ALT)


# ========== 降落程序 =================================================
def landing():
    global TARGET_ALT, MODE
    # 設定目標高度
    print('<飛行命令> 位置已確認，開始降落至地面')
    TARGET_ALT = -0.05  # 負值以確保不會著地後彈升

    # 等待抵達目標高度
    while ACTUAL_ALT > LANDING_CUTOFF:
        print('<飛行狀態> 降落中，目前高度 %.2f m' % ACTUAL_ALT)
        print('<高度感測器狀態> 超音波：%.2f m，IMU：%.2f m' % (ACTUAL_ALT_SONAR, ACTUAL_ALT_IMU))
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px，色塊面積：%d px^2' %
              (ACTUAL_POS[0], ACTUAL_POS[1], ACTUAL_AREA))
        print('<飛行狀態> 定位中，相對速度(%.2f , %.2f)px/s\n' %
              (ACTUAL_VEL[0], ACTUAL_VEL[1]))
        time.sleep(1)

    # 回報降落成功
    print('<飛行狀態> 已降落地面')

    # 返回手動模式
    uav.mode = VehicleMode("STABILIZE")
    while not uav.mode.name == 'STABILIZE':
        time.sleep(0.5)
    print('<關閉程序> 無人機已切換至手動模式')


# ========== 設定為定位模式 ============================================
# 切換至定位模式，並輸入目標顏色來做目標色塊
def pos_mode(color):
    global MODE, POS_COLOR
    MODE = 'POS'                # 模式旗標切為定位
    POS_COLOR = color           # 定位目標顏色設置
    # 回報模式啟用狀態
    print('<飛行狀態> 切換至定位模式，顏色追蹤：%s\n' % POS_COLOR)


# ========== 設定為循跡模式 ===========================================
def line_mode():
    global MODE, POS_COLOR, TARGET_SPD
    MODE = 'LINE'               # 模式旗標切為循跡
    POS_COLOR = ''              # 清空顏色目標
    TARGET_SPD = FLIGHT_SPEED   # 設定飛行速度
    # 回報模式啟用狀態
    print('<飛行狀態> 切換至循跡模式，目標飛行速度 %.2f m/s，開始往前飛行\n' % TARGET_SPD)


# ========= 顏色抵達偵測 ==============================================
# 條件式，佔用目前程序，直到偵測抵達目標顏色為止，需在循跡模式中使用
def wait_arrive(color):
    # 初始化顏色面積
    img_area = 0

    # 色塊面積偵測
    while img_area < IMG_AREA_LIMIT:
        x_raw, y_raw, img_area = image_moment(color)
        print('<飛行狀態> 循跡中，差角%.2f度，角速度%.2f度/秒' % (ACTUAL_HDG, TARGET_YAW_RATE))
        print('<飛行狀態> 循跡中，飛行速度%.2fm/s，俯仰角%.2f度\n' %
              (ACTUAL_SPD, math.degrees(uav.attitude.pitch)))
        time.sleep(1)

    # 回報偵測到顏色
    print('<飛行狀態> 抵達%s色塊上方\n' % color)


# ========== 顏色離開偵測 =============================================
# 條件式，佔用目前程序，直到偵測到離開目標色塊為止，需在循跡模式中使用
# 為避免發生離開後瞬間偵測抵達情形發生，故將面積閥值拉高作預防
def wait_leave(color):
    # 初始化顏色面積
    img_area = 100000

    # 色塊面積偵測
    while img_area > IMG_AREA_LIMIT * 1.1:
        x_raw, y_raw, img_area = image_moment(color)
        print('<飛行狀態> 循跡中，差角%.2f度，角速度%.2f度/秒' % (ACTUAL_HDG, TARGET_YAW_RATE))
        print('<飛行狀態> 循跡中，飛行速度%.2fm/s，俯仰角%.2f度\n' %
              (ACTUAL_SPD, math.degrees(uav.attitude.pitch)))
        time.sleep(1)

    # 回報離開的顏色
    print('<飛行狀態> 已離開%s色塊上方\n' % color)


# ========== 號誌轉向控制 =============================================
# 抵達號誌區後，進行燈號等待，並於偵測到顏色變化後做紀錄，並作相對應轉向
def turn():
    global TARGET_YAW_RATE, MISSION_COLOR, POS_COLOR
    # 轉彎等待與執行迴圈
    while 1:      # 藍右，綠左
        # 讀取顏色面積值做判斷
        block_x, block_y, image_area_blue = image_moment('blue')
        block_x, block_y, image_area_green = image_moment('green')
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px' % (ACTUAL_POS[0], ACTUAL_POS[1]))
        print('<飛行狀態> 定位中，相對速度(%.2f , %.2f)px/s\n' %
              (ACTUAL_VEL[0], ACTUAL_VEL[1]))

        # 讀取偏航角
        yaw_start = math.degrees(uav.attitude.yaw)
        yaw_current = math.degrees(uav.attitude.yaw)

        # 若偵測到藍色
        if image_area_blue >= IMG_AREA_LIMIT:
            # 記錄任務顏色並更改定位顏色
            MISSION_COLOR = 'blue'
            POS_COLOR = MISSION_COLOR
            print('<飛行狀態> 偵測到藍色為任務顏色')

            # 右轉
            print('<飛行命令> 往右旋轉90度\n')
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
            print('<飛行狀態> 偵測到綠色為任務顏色')

            # 左轉
            print('<飛行命令> 往左旋轉90度\n')
            TARGET_YAW_RATE = -TURN_YAW_RATE
            while math.fabs(yaw_current - yaw_start) < 90 * (TURN_SETTLE / 100):
                yaw_current = math.degrees(uav.attitude.yaw)
            TARGET_YAW_RATE = 0
            break

        time.sleep(0.5)

    # 回報轉向完成
    print('<飛行狀態> 轉向完成\n')


# ========== 空投程序 ================================================
# 於投擲區上方做定位穩定，並執行空投程序
def drop():
    # 切換定位模式
    pos_mode(MISSION_COLOR)

    # 定位迴圈，等待移至空投點上方
    print('<飛行命令> 移動至空投點上方')
    while 1:
        # 取得目前型心位置
        block_x, block_y, image_area = image_moment(MISSION_COLOR)
        print('<飛行狀態> 定位中，相對位置(%.2f , %.2f)px\n' % (ACTUAL_POS[0], ACTUAL_POS[1]))

        # 判斷是否已抵達中心
        if -DROP_POS_LIMIT_X < block_x < DROP_POS_LIMIT_X:
            if -DROP_POS_LIMIT_Y < block_y < DROP_POS_LIMIT_Y:
                time.sleep(1)
                break

        time.sleep(0.5)

    # 打開沙包盒空投
    print('<飛行命令> 位置已確認，執行空投\n')
    box_open()
    time.sleep(2)

    # 回報空投成功
    print('<飛行狀態> 沙包已成功空投\n')

    # 關閉沙包盒
    box_close()


# ====================================================================
# ========== 前景主函式區塊 ============================================
# ====================================================================
# 任務主函式
def mission():
    global STATE_THREAD, STATE_UAV
    print('<啟動程序> 任務程序：啟動')
    # 啟動多線程
    thread_activate()

    # 飛行任務
    pre_check()                         # 執行起飛前檢查
    take_off(1.2)                       # 起飛至任務高度
    line_mode()                         # 循跡模式
    wait_arrive('red')                  # 等待抵達號誌區
    pos_mode('red')                     # 紅燈定位保持
    turn()                              # 轉彎等待模式
    drop()                              # 空投測試
    landing()                           # 執行降落與關閉程序

    # 關閉多線程
    STATE_THREAD = False
    print('<關閉程序> 主程序：關閉')

    # 清空GPIO設定
    GPIO.cleanup()
    print('<關閉程序> GPIO已關閉')

    # 結束與無人機連線
    uav.close()
    STATE_UAV = False
    print('<關閉程序> 無人機連線：關閉')

    # 輸出圖表
    data_sheet()
    alt_ctrl_data()
    alsen_data()
    pos_ctrl_data()
    hdg_ctrl_data()
    spd_ctrl_data()


# 執行程式
mission()
