# 高度感測器 確認版
# 包含二主二副程式

# 載入模組
from dronekit import connect
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math

# ========== UAV通訊參數 ==============================================
# 樹莓派Serial連線參數
CONNECT_PORT = '/dev/serial0'   # 傳輸口位址
CONNECT_BAUD = 921600           # 傳輸鮑率

# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_SENSOR_ALT = False        # 高度感測器旗標
STATE_SENSOR_SONAR = False      # 超音波高度計旗標

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->實際值(量測值)
ACTUAL_ALT = 0                  # 實際高度(m)
ACTUAL_ALT_SONAR = 0            # 超音波高度計(m)
ACTUAL_ALT_IMU = 0              # IMU氣壓高度計(m)

# ->迴圈頻率(預設20Hz)
FREQ_SENSOR_ALT = 20            # 高度感測器頻率(不快於超音波感測器頻率)
FREQ_SENSOR_SONAR = 20          # 超音波感測器頻率(5~50Hz，越高雜訊越多)

# ========== 高度感測器參數 =============================================
# ->高度感測器整合參數
IMU_SAMPLE_NUM_AVE = 10         # 均值濾波取樣次數*
WEIGHTED_SONAR = 5              # 超音波高度計權重*
WEIGHTED_IMU = 5                # IMU高度計權重*

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
SONAR_FILTER_ALT = 1            # 突波濾波作用起始高度(m)
SONAR_MAX_CHANGE = 0.5          # 突波容許值(m)

# ->IMU高度計參數
IMU_OFFSET = 0                  # 高度校正偏移量(m，向上為正)
IMU_SAMPLE_NUM_CAL = 10         # 高度校正取樣次數*
IMU_KF_INPUT_MAX = 3            # 卡爾曼濾波器輸入值限制(高度，m)*
IMU_KF_Q = 5                    # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*

# ========== 數值記錄 ==================================================
# ->高度感測器
R_ACTUAL_ALT = []                       # 記錄加權值
R_ACTUAL_ALT_SONAR = []                 # 記錄超音波高度計值
R_ACTUAL_ALT_BAROM = []                 # 記錄氣壓高度計值

# ->超音波高度計
R_ACTUAL_ALT_SONAR_RAW = []             # 記錄原始值
R_ACTUAL_ALT_SONAR_OFFSET = []          # 記錄補正值
R_ACTUAL_ALT_SONAR_AVE = []             # 記錄平均值
R_ACTUAL_ALT_SONAR_KF = []              # 記錄KF值

# ->氣壓高度計
R_ACTUAL_ALT_IMU_RAW = []               # 記錄原始值
R_ACTUAL_ALT_IMU_OFFSET = []            # 記錄補正值
R_ACTUAL_ALT_IMU_AVE = []               # 記錄平均值
R_ACTUAL_ALT_IMU_KF = []                # 記錄KF值

# ========== 與UAV執行連線 =============================================
print('<啟動程序> 與UAV連線中，請稍候...')
# 樹莓派連線指令
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
# 電腦連線指令
# uav = connect(CONNECT_PORT, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線')


# =========== 高度感測器整合程式 =========================================
# 說明：
# 負責整合超音波高度計、飛控氣壓計(可能為IMU來源)，做訊號整理給控制器使用
# 各訊號先經均值濾波後，再用卡爾曼濾波處理，最後再針對所有來源做加權平均
# 本函式處理氣壓計之濾波、高度感測器整合濾波
def altitude_sensor():
    global ACTUAL_ALT, ACTUAL_ALT_IMU, STATE_SENSOR_ALT
    # 平均降噪設定
    height_register_imu = []

    # 卡爾曼濾波器初始化(IMU高度計用)
    x_o = 0                     # 前一次狀態
    p_o = 0                     # 前一次斜方差
    z_o = 0                     # 前一次量測值
    q = math.exp(-IMU_KF_Q)     # 斜方差噪音值
    r = 2.92 * math.exp(-3)     # 斜方差量測噪音值(定值不更改)

    # 執行IMU高度計校正程序
    imu_calibrate()

    # 等待超音波高度計啟動
    if not STATE_SENSOR_SONAR:
        print('<啟動程序> 高度感測器：等待超音波高度計啟動\n')
    while not STATE_SENSOR_SONAR:
        time.sleep(0.1)

    # 回報高度感測器啟動
    STATE_SENSOR_ALT = True
    print('<啟動程序> 高度感測器：啟動\n')

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 讀取超音波高度計資料
        height_sonar = ACTUAL_ALT_SONAR

        # IMU高度計數據處理
        # 卡爾曼濾波-預測
        x_p = x_o
        p_p = p_o + q

        # 讀值與補正
        height_raw_imu = uav.location.global_relative_frame.alt + IMU_OFFSET

        # 均值濾波器
        height_register_imu.insert(0, height_raw_imu)
        if len(height_register_imu) > IMU_SAMPLE_NUM_AVE:
            height_register_imu.pop()
        height_imu_ave = np.average(height_register_imu)

        # 卡爾曼濾波器輸入限制(防止上衝突波)
        if height_imu_ave <= IMU_KF_INPUT_MAX:
            z = height_imu_ave          # 若低於限制則使用新值
        else:
            z = z_o                     # 若超出限制則使用舊值

        # 卡爾曼濾波-更新
        k = p_p / (p_p + r)             # 卡爾曼增益
        x = x_p + k * (z - x_p)         # 狀態(濾波輸出值)
        p = (1 - k) * p_p               # 斜方差

        # 更新高度值至變數
        height_imu = x
        ACTUAL_ALT_IMU = height_imu

        # 卡爾曼濾波：傳遞值給下次迴圈使用
        x_o = x
        p_o = p
        z_o = z

        # 加權平均(可調整加權值)
        alt_num = (WEIGHTED_SONAR * height_sonar + WEIGHTED_IMU * height_imu)
        alt_den = (WEIGHTED_SONAR + WEIGHTED_IMU)
        alt_ave = alt_num / alt_den

        # 更新高度值至全域變數
        ACTUAL_ALT = alt_ave

        # 記錄數值
        R_ACTUAL_ALT_SONAR.append(height_sonar)         # 記錄超音波高度計值
        R_ACTUAL_ALT_IMU_RAW.append(uav.location.global_relative_frame.alt)  # 記錄氣壓高度計原始值
        R_ACTUAL_ALT_IMU_OFFSET.append(height_raw_imu)  # 記錄補正值
        R_ACTUAL_ALT_IMU_AVE.append(height_imu_ave)     # 記錄均值濾波值
        R_ACTUAL_ALT_IMU_KF.append(height_imu)          # 記錄KF值
        R_ACTUAL_ALT.append(ACTUAL_ALT)                 # 記錄總高度值

        # 同捆執行終點
        THREAD_OCCUPY.release()

        time.sleep(1 / FREQ_SENSOR_ALT)

    # 當多線程狀態為否時，跳出迴圈並關閉函式
    STATE_SENSOR_ALT = False
    print('<關閉程序> 高度感測器：關閉\n')


# ========== 超音波高度計 ==============================================
# 說明：
# 使用HC-SR04為超音波感測模組，發出一脈波訊號使模組執行量測後傳回經過時間長的脈波
# 實際使用上雜訊相對多(可能受飛行時噪音影響)，故施加多重濾波條件：
# 超時防止(防止過長的無效等待)、角度補正、突波保護、均值濾波、卡爾曼濾波
# 經處理後的值堪用度大幅提高，對吸音面也能有效處理，使高度控制器能穩定運作
def altitude_sensor_sonar():
    global ACTUAL_ALT_SONAR, STATE_SENSOR_SONAR, SONAR_ERR_COUNT
    # 初始化腳位
    GPIO.setmode(GPIO.BCM)                  # 設定為BCM模式
    GPIO.setwarnings(False)                 # 忽略警告
    GPIO.setup(SONAR_TRIG_PIN, GPIO.OUT)    # TRIG腳位輸出設定
    GPIO.setup(SONAR_ECHO_PIN, GPIO.IN)     # ECHO腳位輸入設定
    GPIO.output(SONAR_TRIG_PIN, False)      # 預設輸出為0

    # 脈波變化時間值變數
    pulse_start = time.time()               # 脈波起始時間
    pulse_end = time.time()                 # 脈波結束時間

    # 卡爾曼濾波器初始化
    x_o = 0                                 # 前一次狀態
    p_o = 0                                 # 前一次斜方差
    z_o = 0                                 # 前一次量測值
    q = math.exp(-SONAR_KF_Q)               # 斜方差噪音值
    r = 2.92 * math.exp(-3)                 # 斜方差量測噪音值(定值不更改)

    # 平均降噪設定
    height_register = []
    height_ave = 0

    # 執行超音波高度計校正程序
    sonar_calibrate()

    # 回報超音波高度計啟動
    STATE_SENSOR_SONAR = True
    print('<啟動程序> 超音波高度計：啟動\n')

    # 程式迴圈
    while STATE_THREAD:
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
                pulse_start = time.time()           # 記錄脈波起始時間
                timeout = False                     # 成功記錄脈波時間，切換旗標
                if pulse_start - timeout_start > SONAR_TIMEOUT:
                    timeout = True                  # 等待接收時發生超時，跳出迴圈
                    break

            # 於高電位迴圈等待，跳出時記錄脈波結束時間(若發生超時則不執行本迴圈)
            while GPIO.input(SONAR_ECHO_PIN) and not timeout:
                pulse_end = time.time()             # 記錄脈波結束時間
                timeout = False                     # 成功記錄脈波時間，切換旗標
                if pulse_end - pulse_start > SONAR_TIMEOUT:
                    timeout = True                  # 接收時長發生超時，跳出迴圈
                    break

            # 若發生超時則顯示於訊息(可停用)
            if timeout:
                SONAR_ERR_COUNT += 1                # 超時次數計(統計用)
                # print('<錯誤> 超音波高度計：量測超時')

        # 計算高度值
        pulse_duration = pulse_end - pulse_start
        height_raw = (pulse_duration * (331.3 + 0.606 * SONAR_AIR_TEMP)) / 2

        # 高度角度補正
        roll_angle = uav.attitude.roll
        pitch_angle = uav.attitude.pitch
        height_offset = (height_raw * math.cos(roll_angle) * math.cos(pitch_angle)) + SONAR_OFFSET
        height_offset_r = height_offset             # 記錄用值

        # 上下降突波防止
        if ACTUAL_ALT > SONAR_FILTER_ALT:
            if math.fabs(height_offset - z_o) > SONAR_MAX_CHANGE:
                height_offset = height_ave

        # 均值濾波器(使線條滑順)
        height_register.insert(0, height_offset)
        if len(height_register) > SONAR_SAMPLE_NUM_AVE:
            height_register.pop()
        height_ave = np.average(height_register)    # 依據取樣次數計算即時平均值(會產生響應延遲)

        # 卡爾曼濾波器輸入限制(防止突波)
        if height_ave <= SONAR_KF_INPUT_MAX:
            z = height_ave                          # 若低於限制則使用新值
        else:
            z = z_o                                 # 若超出限制則使用舊值

        # 卡爾曼濾波-更新
        k = p_p / (p_p + r)                         # 卡爾曼增益
        x = x_p + k * (z - x_p)                     # 狀態(濾波輸出值)
        p = (1 - k) * p_p                           # 斜方差

        # 更新高度值至全域變數
        ACTUAL_ALT_SONAR = x

        # 傳遞值給下次迴圈使用
        x_o = x
        p_o = p
        z_o = z

        # 記錄數值
        R_ACTUAL_ALT_SONAR_RAW.append(height_raw)               # 記錄原始值
        R_ACTUAL_ALT_SONAR_OFFSET.append(height_offset_r)       # 記錄補正值(突波防止前)
        R_ACTUAL_ALT_SONAR_AVE.append(height_ave)               # 記錄平均值
        R_ACTUAL_ALT_SONAR_KF.append(ACTUAL_ALT_SONAR)          # 記錄KF值

        # 結束優先執行
        THREAD_OCCUPY.release()

        # 迴圈執行間隔
        time.sleep(1 / FREQ_SENSOR_SONAR)

    # 當多線程狀態為否時，跳出迴圈並關閉函式
    STATE_SENSOR_SONAR = False
    print('<關閉程序> 超音波高度計：關閉；量測超時次數共 %d 次\n' % SONAR_ERR_COUNT)


# ====================================================================
# ========== 子函式區塊 ================================================
# ====================================================================
# 本區放置各主函式下會使用到的功能函式，分有校正類、計算類、裝備控制類
# 將常使用的重複功能函式化以節省程式大小(僅限無傳遞值類的函式)

# ========== 超音波高度計校正程序 ========================================
# 於啟動前自動校正高度，為避免單次量測不準確，故以多次取樣後平均為基準來補正
def sonar_calibrate():
    global SONAR_OFFSET
    # 平均降噪設定
    height_register = []

    # 脈波變化時間值變數
    pulse_start = time.time()           # 脈波起始時間
    pulse_end = time.time()             # 脈波結束時間

    # 量測迴圈(線程優先執行)
    THREAD_OCCUPY.acquire()
    for i in range(0, SONAR_SAMPLE_NUM_CAL):
        # 設定超時旗標
        timeout = True

        # 量測迴圈(超時防止)
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
                pulse_start = time.time()           # 記錄脈波起始時間
                timeout = False                     # 成功記錄脈波時間，切換旗標
                if pulse_start - timeout_start > SONAR_TIMEOUT:
                    timeout = True                  # 等待接收時發生超時，跳出迴圈
                    break

            # 於高電位迴圈等待，跳出時記錄脈波結束時間(若發生超時則不執行本迴圈)
            while GPIO.input(SONAR_ECHO_PIN) and not timeout:
                pulse_end = time.time()             # 記錄脈波結束時間
                timeout = False                     # 成功記錄脈波時間，切換旗標
                if pulse_end - pulse_start > SONAR_TIMEOUT:
                    timeout = True                  # 接收時長發生超時，跳出迴圈
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
    print('<啟動程序> 超音波高度計校正完畢，偏移 %.3f m\n' % SONAR_OFFSET)


# ========== IMU高度計校正程序 =========================================
# 因IMU高度計會有波動，需做補正
def imu_calibrate():
    global IMU_OFFSET
    # 平均降噪設定
    height_register = []

    # 校正氣壓計
    uav.send_calibrate_barometer()
    uav.send_calibrate_vehicle_level()
    time.sleep(1)

    # 量測迴圈(線程優先執行)
    THREAD_OCCUPY.acquire()
    for i in range(0, IMU_SAMPLE_NUM_CAL):
        # 讀取氣壓計高度值
        height = uav.location.global_relative_frame.alt

        # 均值濾波器
        height_register.append(height)

        # 等待200ms執行下次量測
        time.sleep(0.2)

    # 結束優先執行
    THREAD_OCCUPY.release()

    # 更新高度校正偏移量
    IMU_OFFSET = -np.average(height_register)
    print('<啟動程序> 氣壓高度計校正完畢，偏移 %.3f m\n' % IMU_OFFSET)
