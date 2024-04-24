# 高度控制器 確認版

# 載入模組
import threading
import time

# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_ALT = False               # 高度控制器旗標
STATE_SENSOR_ALT = False        # 高度感測器旗標

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->迴圈頻率(預設20Hz)
FREQ_ALT = 20                   # 高度控制器頻率(不快於高度感測器頻率)

# ->目標值(命令值)
TARGET_ALT = 0                  # 目標高度，主程式輸入於高度控制器(m)
TARGET_THRUST = 0               # 目標推力，表示升降速度(0.5為中點)

# ->實際值(量測值)
ACTUAL_ALT = 0                  # 實際高度(m)

# ->高度控制器參數
ALT_KP = 0.1                    # P增益值*
ALT_KI = 0                      # I增益值*
ALT_KD = 0.1                    # D增益值*
ALT_SPEED = 0.3                 # 昇降速率(m/s，與飛控參數一致)*

# ->高度控制器
R_TARGET_ALT = []                       # 記錄目標高
R_ACTUAL_ALT_CTRL = []                  # 記錄實際高
R_TARGET_THRUST = []                    # 記錄目標推力
R_ALT_ERROR = []                        # 記錄高度誤差
R_ALT_GP = []                           # 記錄P增益
R_ALT_GI = []                           # 記錄I增益
R_ALT_GD = []                           # 記錄D增益


# ========== 高度控制器 ================================================
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
    if not STATE_SENSOR_ALT:
        print('<啟動程序> 高度控制器：等待高度感測器啟動')
    while not STATE_SENSOR_ALT:
        time.sleep(0.1)

    # 回報高度控制器啟動
    STATE_ALT = True
    print('<啟動程序> 高度控制器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
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
        speed_out = gain_p + gain_i + gain_d

        # 將速度以比例輸出
        thrust_out = speed_out / ALT_SPEED

        # 補正至中點
        thrust_out += 0.5

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

    # 當多線程狀態為否時，跳出迴圈並關閉函式
    STATE_ALT = False
    print('<關閉程序> 高度控制器：關閉')
