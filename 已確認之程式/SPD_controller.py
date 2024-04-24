# 飛行速度控制器(調整中)

# 載入模組
from dronekit import connect
import threading
import time
import math

# ========== UAV通訊參數 ==============================================
# 樹莓派Serial連線參數
CONNECT_PORT = '/dev/serial0'   # 傳輸口位址
CONNECT_BAUD = 921600           # 傳輸鮑率

# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_SPD = False               # 飛行速度控制器旗標
STATE_SENSOR_SPD = False        # 飛行速度感測器旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->目標值(命令值)
TARGET_PITCH = 0                # 目標俯仰角(degree)
TARGET_SPD = 0                  # 目標飛行速度

# ->實際值(量測值)
ACTUAL_SPD = 0                  # 實際飛行速度

# ========== 控制器參數設定 =============================================
# ->迴圈頻率(預設20Hz)
FREQ_SPD = 20                   # 飛行速度控制器頻率

# ->飛行速度控制器參數
SPD_KP = 0.1                    # P增益值*
SPD_KI = 0                      # I增益值*
SPD_KD = 0                      # D增益值*
SPD_ANGLE_CUTOFF = 1            # 飛行速度控制器-最小俯仰角度(degree)
SPD_ANGLE_LIMIT = 3             # 飛行速度控制器-最大俯仰角度(degree)
SPD_LIMIT = 0.5                 # 飛行速度控制器-最大飛行速度(m/s)
SPD_OFF_ANGLE = 5               # 切換跳出角度(degree)，慣性抵銷用

# ->飛行速度控制器
R_SPD_TARGET = []                       # 記錄目標速度
R_SPD_ACTUAL = []                       # 記錄實際速度
R_SPD_TARGET_PITCH = []                 # 記錄目標俯仰角
R_SPD_ACTUAL_PITCH = []                 # 記錄實際俯仰角
R_SPD_ERROR = []                        # 記錄速度誤差
R_SPD_GP = []                           # 記錄P增益
R_SPD_GI = []                           # 記錄I增益
R_SPD_GD = []                           # 記錄D增益

# ========== 與UAV執行連線 =============================================
print('<啟動程序> 與UAV連線中，請稍候...')
# 樹莓派連線指令
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)


# =========== 飛行速度控制器 ============================================
# 說明：
# 於循線期間控制往前飛行的速度
# 不一定會使用(視感測器響應情形而定)
def flight_speed_controller():
    global TARGET_PITCH, STATE_SPD
    # PID變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 等待飛行速度感測器啟動
    if not STATE_SENSOR_SPD:
        print('<啟動程序> 飛行速度控制器：等待飛行速度感測器啟動')
    while not STATE_SENSOR_SPD:
        time.sleep(0.1)

    # 回報飛行速度控制器啟用
    STATE_SPD = True
    print('<啟動程序> 飛行速度控制器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        while MODE == 'LINE' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 讀取目標值
            spd_target = TARGET_SPD

            # 目標輸入限制
            if spd_target > SPD_LIMIT:
                spd_target = SPD_LIMIT
            if spd_target < -SPD_LIMIT:
                spd_target = -SPD_LIMIT

            # 計算誤差值
            error = spd_target - ACTUAL_SPD

            # 誤差值微積分計算
            current_time = time.time()
            delta_time = current_time - previous_time

            delta_error = error - previous_error
            int_error += (error * delta_time)
            derivative = delta_error / delta_time

            # 誤差增益處理
            gain_p = error * SPD_KP
            gain_i = int_error * SPD_KI
            gain_d = derivative * SPD_KD
            pitch_out = gain_p + gain_i + gain_d

            # 角度限制(包含截止區設定)
            if pitch_out > SPD_ANGLE_LIMIT:
                pitch_out = SPD_ANGLE_LIMIT
            elif pitch_out < -SPD_ANGLE_LIMIT:
                pitch_out = -SPD_ANGLE_LIMIT
            elif -SPD_ANGLE_CUTOFF < pitch_out < SPD_ANGLE_CUTOFF:
                pitch_out = 0

            # 輸出至全域變數
            TARGET_PITCH = -pitch_out

            # 傳遞值給下次迴圈使用
            previous_error = error
            previous_time = current_time

            # 記錄數值
            R_SPD_TARGET.append(spd_target)
            R_SPD_ACTUAL.append(ACTUAL_SPD)
            R_SPD_TARGET_PITCH.append(TARGET_PITCH)
            R_SPD_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

            R_SPD_ERROR.append(error)
            R_SPD_GP.append(gain_p)
            R_SPD_GI.append(gain_i)
            R_SPD_GD.append(gain_d)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_SPD)

            # 當飛行模式不為循跡時，跳出迴圈並暫停函式
            if not MODE == 'LINE':
                # 切換跳出姿態數值(慣性抵銷用)
                TARGET_PITCH = SPD_OFF_ANGLE
                STATE_SPD = False
                print('<飛行狀態> 飛行速度控制器：暫停')

        # 無循跡作動區間
        # 回報迴圈休眠狀態
        if MODE == 'POS' and STATE_SPD:
            STATE_SPD = False
            print('<飛行狀態> 飛行速度控制器：暫停')

        # 重設暫存器
        int_error = 0
        previous_error = 0
        previous_time = time.time()

        # 記錄值
        R_SPD_TARGET.append(0)
        R_SPD_ACTUAL.append(0)
        R_SPD_TARGET_PITCH.append(0)
        R_SPD_ACTUAL_PITCH.append(0)

        R_SPD_ERROR.append(0)
        R_SPD_GP.append(0)
        R_SPD_GI.append(0)
        R_SPD_GD.append(0)

        # 迴圈執行間隔
        time.sleep(1 / FREQ_SPD)

        # 當飛行模式為循跡時，跳入迴圈並啟動函式
        if MODE == 'LINE' and STATE_THREAD:
            print('<飛行狀態> 飛行速度控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_SPD = False
    print('<關閉程序> 飛行速度控制器：關閉')
