# 高度感測器測試(包含超音波高度計與氣壓高度計)
# 0907測試
# 結果：多線程時雜訊過多

# 0908修正
# 新增高度氣壓計的卡爾曼濾波器(因應高度計更新次數過慢)
# 並於超音波高度計量測時增加線程執行鎖定

# 0909新增超過理論速度位移防止(上下降突波防止)

# !/usr/bin/env python
# -*- coding: utf-8 -*-

# 載入模組
from __future__ import print_function
from dronekit import connect
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math

# ======設定全域變數======
print('<啟動程序> 載入全域變數設定')

# UAV 通訊參數
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# ======旗標與變數======================================================
# 旗標
STATE_SYSTEM = True
STATE_SENSOR_SONAR = False
STATE_SENSOR = False

# 多線程優先執行
THREAD_OCCUPY = threading.Lock()

# 迴圈頻率
FREQ_SONAR = 30          # 超音波感測器頻率(5~50Hz，越高雜訊越多)*
FREQ_SENSOR = 30         # 高度感測器頻率(不快於超音波感測器頻率)*

# 實際值(量測值)
ACTUAL_ALT = 0           # 實際高度
ACTUAL_ALT_SONAR = 0     # 超音波高度計
ACTUAL_ALT_BAROM = 0     # 氣壓高度計

# ======高度感測器參數=============================================================
# ->高度感測器整合參數
BAROM_SAMPLE_NUM_AVE = 10    # 均值濾波取樣次數*
WEIGHTED_SONAR = 1           # 超音波高度計權重*
WEIGHTED_BAROM = 0           # 氣壓高度計權重*

# ->超音波高度計參數
SONAR_AIR_TEMP = 28          # 氣溫(攝氏)*
SONAR_TRIG_PIN = 2           # TRIG輸出腳位
SONAR_ECHO_PIN = 3           # ECHO輸入腳位
SONAR_TIMEOUT = 0.05         # 無回應超時限制(秒，過低時遠距離偵測將會變成超時)*
SONAR_OFFSET = 0             # 高度校正偏移量(m，向上為正)
SONAR_SAMPLE_NUM_CAL = 10    # 高度校正取樣次數*
SONAR_SAMPLE_NUM_AVE = 10    # 均值濾波取樣次數*
SONAR_KF_INPUT_MAX = 3       # 卡爾曼濾波器輸入值限制(高度，m)*
SONAR_KF_Q = 5               # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*
SONAR_ERR_COUNT = 0          # 量測超時次數記錄
SONAR_FILTER_ALT = 0.2       # 突波濾波作用起始高度(m)
SONAR_MAX_CHANGE = 0.25      # 突波容許值(m)

# ->氣壓計參數
BAROM_OFFSET = 0             # 高度校正偏移量(m，向上為正)
BAROM_SAMPLE_NUM_CAL = 10    # 高度校正取樣次數*
BAROM_KF_INPUT_MAX = 3       # 卡爾曼濾波器輸入值限制(高度，m)*
BAROM_KF_Q = 5               # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*

# ======數值記錄=================================================================
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
print('<啟動程序> 與UAV執行連線中，請稍候')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線')


# ======背景主函式函式區塊======
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

    while STATE_SYSTEM:
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

        time.sleep(1 / FREQ_SENSOR)

    # 當系統裝態為否時，跳出迴圈並關閉函式
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
                print('<錯誤> 超音波高度計：量測超時')

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
    print('<關閉程序> 超音波高度計：關閉；量測超時次數共 %d 次' % SONAR_ERR_COUNT)


# ======子函式區塊======
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
                print('<錯誤> 超音波感測器：量測超時')

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


# 啟動前檢查
def pre_check():
    print('<啟動程序> 啟動前檢查程序：啟動')

    # 等待高度控制器啟動
    if not STATE_SENSOR:
        print('<啟動程序> 主程序：等待高度控制器啟動')
    while not STATE_SENSOR:
        time.sleep(0.1)

    # 回報已完成檢查程序
    print('<啟動程序> 啟動前檢查程序：完成')


# ======前景主函式區塊=================================================================
# 任務主函式
def main():
    global STATE_SYSTEM
    # 啟動前檢查，回報主程式啟動
    pre_check()
    print('<啟動程序> 主程序：啟動')

    # 任務迴圈
    for i in range(10):
        THREAD_OCCUPY.acquire()
        print('現在高度：總高 %.3f m' % ACTUAL_ALT)
        print('超音波： %.3f m，氣壓計： %.3f m' % (ACTUAL_ALT_SONAR, ACTUAL_ALT_BAROM))
        THREAD_OCCUPY.release()

        time.sleep(1)

    # 關閉背景多線程
    STATE_SYSTEM = False
    print('<關閉程序> 主程序：關閉')

    # 結束與無人機連線
    uav.close()
    print('<關閉程序> 無人機連線已中斷')


# 多線程函式
def thread_procedure():
    # 定義多線程
    sonar_sen = threading.Thread(target=sonar)
    alt_sen = threading.Thread(target=altitude_sensor)
    m = threading.Thread(target=main)

    # 執行多線程(依啟動順序排放)
    print('<啟動程序> 多線程程序：啟動')
    sonar_sen.start()
    alt_sen.start()
    m.start()

    # 等待多線程結束(依結束順序排放)
    m.join()
    alt_sen.join()
    sonar_sen.join()
    print('<關閉程序> 多線程程序：關閉')


# ======數值紀錄區塊======
def show_data():
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
