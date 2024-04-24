# 位置控制器(需再優化)
# 包含位置與速度控制器
# 增加目標位置供彈性使用(預設為0,0)
# 對角度輸出增加限幅截止處理

# 載入模組
from dronekit import connect
import numpy as np
import threading
import time
import math

# ========== UAV通訊參數 ==============================================
# 樹莓派Serial連線參數
CONNECT_PORT = '/dev/serial0'   # 傳輸口位址
CONNECT_BAUD = 921600           # 傳輸鮑率

# ========== 旗標與變數 ================================================
# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_POS = False               # 位置控制器旗標
STATE_VEL = False               # 位置-速度控制器旗標
STATE_SENSOR_CAM = False        # 影像感測器旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)
MISSION_COLOR = ''              # 任務顏色(blue or green)
POS_COLOR = ''                  # 定位的色塊顏色

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->目標值(命令值)
TARGET_ROLL = 0                 # 目標滾轉角(degree)
TARGET_PITCH = 0                # 目標俯仰角(degree)
TARGET_VEL = [0, 0]             # 目標移動速度(px/s，相機座標)
TARGET_POS = [0, 0]             # 目標型心位置(px，相機座標)

# ->實際值(量測值)
ACTUAL_ALT = 0                  # 實際高度(m)
ACTUAL_POS = [0, 0]             # 色塊實際位置(px)
ACTUAL_VEL = [0, 0]             # 色塊實際速度(px/s)
ACTUAL_AREA = 0                 # 色塊面積(px^2)

# ========== 控制器參數設定 =============================================
# ->迴圈頻率(預設20Hz)
FREQ_POS = 10                   # 位置控制器頻率(不快於位置-速度控制器頻率)
FREQ_VEL = 20                   # 位置-速度控制器頻率(不快於影像禎率)

# ->位置控制器參數
POS_SAMPLE_NUM_AVE = 10         # 位置控制器均值濾波取樣次數*
POS_X_KP = 0.13                 # 位置控制器X軸-P增益值
POS_X_KI = 0                    # 位置控制器X軸-I增益值
POS_X_KD = 0                    # 位置控制器X軸-D增益值
POS_Y_KP = 0.18                 # 位置控制器Y軸-P增益值
POS_Y_KI = 0                    # 位置控制器Y軸-I增益值
POS_Y_KD = 0                    # 位置控制器Y軸-D增益值
POS_VEL_LIMIT = 20              # 位置控制器-速度輸出限制(px/s，1.2m高時1px=3.02mm，建議低於100)
POS_DZ = 50                     # 位置控制器截止區參數(於截止區內使速度控制器控制零速度，防震盪)
# 特殊附註：截止區調校方式=>區域內阻力可以抵消慣性、控制器解析程度內(控制器截止區)

# ->速度控制器參數
POS_VEL_X_KP = 0.1              # 速度控制器X軸-P增益值
POS_VEL_X_KI = 0                # 速度控制器X軸-I增益值
POS_VEL_X_KD = 0                # 速度控制器X軸-D增益值
POS_VEL_Y_KP = 0.12             # 速度控制器Y軸-P增益值
POS_VEL_Y_KI = 0                # 速度控制器Y軸-I增益值
POS_VEL_Y_KD = 0                # 速度控制器Y軸-D增益值
POS_ANGLE_LIMIT = 3             # 速度控制器-最大飛行角度(degree)
POS_ANGLE_CUTOFF = 1            # 速度控制器-最小飛行角度(degree)
POS_VEL_X_FEEDBACK_GAIN = 0.13  # X軸速度回授增益
POS_VEL_Y_FEEDBACK_GAIN = 0.18  # Y軸速度回授增益
POS_VEL_CUTOFF_ALT = 0.5        # 控制器最低作動高度

# ========== 數值記錄 ==================================================
# ->位置控制器
R_POS_BLOCK_X = []                      # 記錄X座標
R_POS_BLOCK_Y = []                      # 記錄Y座標
R_POS_BLOCK_AREA = []                   # 記錄面積變化
R_POS_RECORD_SCALE = 1000               # 面積縮放比
R_POS_X_ERROR = []                      # 記錄誤差值
R_POS_Y_ERROR = []                      # 記錄誤差值
R_POS_X_GP = []                         # 記錄P增益
R_POS_X_GI = []                         # 記錄I增益
R_POS_X_GD = []                         # 記錄D增益
R_POS_Y_GP = []                         # 記錄P增益
R_POS_Y_GI = []                         # 記錄I增益
R_POS_Y_GD = []                         # 記錄D增益
R_POS_X_TARGET_VEL = []                 # 記錄輸出的目標速度值
R_POS_Y_TARGET_VEL = []                 # 記錄輸出的目標速度值

# ->速度控制器
R_POS_TARGET_ROLL = []                  # 記錄目標滾轉角
R_POS_TARGET_PITCH = []                 # 記錄目標俯仰角
R_POS_ACTUAL_ROLL = []                  # 記錄實際滾轉角
R_POS_ACTUAL_PITCH = []                 # 記錄實際俯仰角
R_POS_VEL_X_TAR = []                    # 記錄X目標速度
R_POS_VEL_Y_TAR = []                    # 記錄Y目標速度
R_POS_VEL_X_ACT = []                    # 記錄X實際速度
R_POS_VEL_Y_ACT = []                    # 記錄Y實際速度
R_POS_VEL_X_ERROR = []                  # 記錄誤差值
R_POS_VEL_Y_ERROR = []                  # 記錄誤差值
R_POS_VEL_X_GP = []                     # 記錄P增益
R_POS_VEL_X_GI = []                     # 記錄I增益
R_POS_VEL_X_GD = []                     # 記錄D增益
R_POS_VEL_Y_GP = []                     # 記錄P增益
R_POS_VEL_Y_GI = []                     # 記錄I增益
R_POS_VEL_Y_GD = []                     # 記錄D增益

# ========== 與UAV執行連線 =============================================
print('<啟動程序> 與UAV連線中，請稍候...')
# 樹莓派連線指令
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
# 電腦連線指令
# uav = connect(CONNECT_PORT, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線')


# ========== 位置控制器 ================================================
# 說明：
# 外迴圈的位置控制
# 先以影像取得的色塊座標計算速度增益值，在輸出至速度控制器做角度-加速度控制
def position_controller():
    global TARGET_VEL, STATE_POS
    # PID變數設定
    pos_int_error_x = 0
    pos_int_error_y = 0
    pos_previous_error_x = 0
    pos_previous_error_y = 0
    previous_time = time.time()

    # 均值濾波設定
    vel_x_cmd_register = []
    vel_y_cmd_register = []

    # 等待影像感測器啟動
    if not STATE_SENSOR_CAM:
        print('<啟動程序> 位置控制器：等待影像感測器啟動')
    while not STATE_SENSOR_CAM:
        time.sleep(0.1)

    # 回報位置控制器啟用
    STATE_POS = True
    print('<啟動程序> 位置控制器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        while MODE == 'POS' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 讀取回授值
            pos_actual_x = ACTUAL_POS[0]
            pos_actual_y = ACTUAL_POS[1]

            # 回授值截止區修正
            if TARGET_POS[0] - POS_DZ < pos_actual_x < TARGET_POS[0] + POS_DZ:
                pos_actual_x = TARGET_POS[0]
            if TARGET_POS[1] - POS_DZ < pos_actual_y < TARGET_POS[1] + POS_DZ:
                pos_actual_y = TARGET_POS[1]

            # 計算誤差值
            pos_error_x = TARGET_POS[0] - pos_actual_x
            pos_error_y = TARGET_POS[1] - pos_actual_y

            # 誤差值微積分計算
            current_time = time.time()
            delta_time = current_time - previous_time

            pos_delta_error_x = pos_error_x - pos_previous_error_x
            pos_int_error_x += (pos_error_x * delta_time)
            pos_derivative_x = pos_delta_error_x / delta_time

            pos_delta_error_y = pos_error_y - pos_previous_error_y
            pos_int_error_y += (pos_error_y * delta_time)
            pos_derivative_y = pos_delta_error_y / delta_time

            # X軸誤差增益處理
            pos_gain_p_x = pos_error_x * POS_X_KP
            pos_gain_i_x = pos_int_error_x * POS_X_KI
            pos_gain_d_x = pos_derivative_x * POS_X_KD
            vel_x_out = pos_gain_p_x + pos_gain_i_x + pos_gain_d_x

            # Y軸誤差增益處理
            pos_gain_p_y = pos_error_y * POS_Y_KP
            pos_gain_i_y = pos_int_error_y * POS_Y_KI
            pos_gain_d_y = pos_derivative_y * POS_Y_KD
            vel_y_out = pos_gain_p_y + pos_gain_i_y + pos_gain_d_y

            # X速度命令均值濾波器
            vel_x_cmd_register.insert(0, vel_x_out)
            if len(vel_x_cmd_register) > POS_SAMPLE_NUM_AVE:
                vel_x_cmd_register.pop()
            vel_x_cmd = np.average(vel_x_cmd_register)

            # Y速度命令均值濾波器
            vel_y_cmd_register.insert(0, vel_y_out)
            if len(vel_y_cmd_register) > POS_SAMPLE_NUM_AVE:
                vel_y_cmd_register.pop()
            vel_y_cmd = np.average(vel_y_cmd_register)

            # 速度輸出限制(原則上不使用)
            if vel_x_cmd > POS_VEL_LIMIT:
                vel_x_cmd = POS_VEL_LIMIT
            elif vel_x_cmd < -POS_VEL_LIMIT:
                vel_x_cmd = -POS_VEL_LIMIT

            if vel_y_cmd > POS_VEL_LIMIT:
                vel_y_cmd = POS_VEL_LIMIT
            elif vel_y_cmd < -POS_VEL_LIMIT:
                vel_y_cmd = -POS_VEL_LIMIT

            # 更新速度命令值至全域變數
            TARGET_VEL = [vel_x_cmd, vel_y_cmd]

            # 傳遞值給下次迴圈使用
            pos_previous_error_x = pos_error_x
            pos_previous_error_y = pos_error_y
            previous_time = current_time

            # 記錄數值
            R_POS_BLOCK_X.append(ACTUAL_POS[0])
            R_POS_BLOCK_Y.append(ACTUAL_POS[1])
            R_POS_BLOCK_AREA.append(ACTUAL_AREA / R_POS_RECORD_SCALE)

            R_POS_X_ERROR.append(pos_error_x)
            R_POS_Y_ERROR.append(pos_error_y)
            R_POS_X_GP.append(pos_gain_p_x)
            R_POS_X_GI.append(pos_gain_i_x)
            R_POS_X_GD.append(pos_gain_d_x)
            R_POS_Y_GP.append(pos_gain_p_y)
            R_POS_Y_GI.append(pos_gain_i_y)
            R_POS_Y_GD.append(pos_gain_d_y)

            R_POS_X_TARGET_VEL.append(vel_x_cmd)
            R_POS_Y_TARGET_VEL.append(vel_y_cmd)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_POS)

            # 當飛行模式不為定位時，跳出迴圈並暫停函式
            if not MODE == 'POS':
                # 歸零速度命令數值
                TARGET_VEL = [0, 0]
                print('<飛行狀態> 位置控制器：暫停')

        # 重設暫存器
        vel_x_cmd_register = []
        vel_y_cmd_register = []
        pos_int_error_x = 0
        pos_int_error_y = 0
        pos_previous_error_x = 0
        pos_previous_error_y = 0
        previous_time = time.time()

        # 記錄數值
        R_POS_BLOCK_X.append(0)
        R_POS_BLOCK_Y.append(0)
        R_POS_BLOCK_AREA.append(0)

        R_POS_X_ERROR.append(0)
        R_POS_Y_ERROR.append(0)
        R_POS_X_GP.append(0)
        R_POS_X_GI.append(0)
        R_POS_X_GD.append(0)
        R_POS_Y_GP.append(0)
        R_POS_Y_GI.append(0)
        R_POS_Y_GD.append(0)

        R_POS_X_TARGET_VEL.append(0)
        R_POS_Y_TARGET_VEL.append(0)

        # 迴圈執行間隔
        time.sleep(1 / FREQ_POS)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_THREAD:
            print('<飛行狀態> 位置控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_POS = False
    print('<關閉程序> 位置控制器：關閉')


# ========== 位置-速度控制器 ============================================
# 說明：
# 位置控制器的內迴圈，負責控制與色塊速度
def position_velocity_controller():
    global TARGET_PITCH, TARGET_ROLL, TARGET_VEL, ACTUAL_POS, ACTUAL_AREA, STATE_VEL
    # PID變數設定
    vel_int_error_x = 0
    vel_int_error_y = 0
    vel_previous_error_x = 0
    vel_previous_error_y = 0
    previous_time = time.time()

    # 等待位置感測器啟動
    if not STATE_POS:
        print('<啟動程序> 速度控制器：等待位置控制器啟動')
    while not STATE_POS:
        time.sleep(0.1)

    # 回報速度控制器啟用
    STATE_VEL = True
    print('<啟動程序> 速度控制器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        while MODE == 'POS' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 誤差增益
            vel_actual_x = ACTUAL_VEL[0] * POS_VEL_X_FEEDBACK_GAIN
            vel_actual_y = ACTUAL_VEL[1] * POS_VEL_Y_FEEDBACK_GAIN

            # 計算誤差值
            vel_error_x = TARGET_VEL[0] - vel_actual_x
            vel_error_y = TARGET_VEL[1] - vel_actual_y

            # 誤差值微積分計算
            current_time = time.time()
            delta_time = current_time - previous_time

            vel_delta_error_x = vel_error_x - vel_previous_error_x
            vel_int_error_x += (vel_error_x * delta_time)
            vel_derivative_x = vel_delta_error_x / delta_time

            vel_delta_error_y = vel_error_y - vel_previous_error_y
            vel_int_error_y += (vel_error_y * delta_time)
            vel_derivative_y = vel_delta_error_y / delta_time

            # X軸誤差增益處理
            vel_gain_p_x = vel_error_x * POS_VEL_X_KP
            vel_gain_i_x = vel_int_error_x * POS_VEL_X_KI
            vel_gain_d_x = vel_derivative_x * POS_VEL_X_KD
            roll_out = vel_gain_p_x + vel_gain_i_x + vel_gain_d_x

            # Y軸誤差增益處理
            vel_gain_p_y = vel_error_y * POS_VEL_Y_KP
            vel_gain_i_y = vel_int_error_y * POS_VEL_Y_KI
            vel_gain_d_y = vel_derivative_y * POS_VEL_Y_KD
            pitch_out = vel_gain_p_y + vel_gain_i_y + vel_gain_d_y

            # 角度限制(飽和區限幅與截止區處理)
            if roll_out > POS_ANGLE_LIMIT:
                roll_out = POS_ANGLE_LIMIT
            elif roll_out < -POS_ANGLE_LIMIT:
                roll_out = -POS_ANGLE_LIMIT
            elif -POS_ANGLE_CUTOFF < roll_out < POS_ANGLE_CUTOFF:
                roll_out = 0

            if pitch_out > POS_ANGLE_LIMIT:
                pitch_out = POS_ANGLE_LIMIT
            elif pitch_out < -POS_ANGLE_LIMIT:
                pitch_out = -POS_ANGLE_LIMIT
            elif -POS_ANGLE_CUTOFF < pitch_out < POS_ANGLE_CUTOFF:
                pitch_out = 0

            # 輸出至全域變數
            if ACTUAL_ALT < POS_VEL_CUTOFF_ALT:
                TARGET_ROLL = 0
                TARGET_PITCH = 0
            else:
                TARGET_ROLL = -roll_out
                TARGET_PITCH = pitch_out

            # 傳遞值給下次迴圈使用
            vel_previous_error_x = vel_error_x
            vel_previous_error_y = vel_error_y
            previous_time = current_time

            # 記錄數值
            R_POS_VEL_X_TAR.append(TARGET_VEL[0])
            R_POS_VEL_Y_TAR.append(TARGET_VEL[1])
            R_POS_VEL_X_ACT.append(vel_actual_x)
            R_POS_VEL_Y_ACT.append(vel_actual_y)

            R_POS_VEL_X_ERROR.append(vel_error_x)
            R_POS_VEL_Y_ERROR.append(vel_error_y)
            R_POS_VEL_X_GP.append(vel_gain_p_x)
            R_POS_VEL_X_GI.append(vel_gain_i_x)
            R_POS_VEL_X_GD.append(vel_gain_d_x)
            R_POS_VEL_Y_GP.append(vel_gain_p_y)
            R_POS_VEL_Y_GI.append(vel_gain_i_y)
            R_POS_VEL_Y_GD.append(vel_gain_d_y)

            R_POS_TARGET_ROLL.append(TARGET_ROLL)
            R_POS_TARGET_PITCH.append(TARGET_PITCH)
            R_POS_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
            R_POS_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_VEL)

            # 當飛行模式不為定位時，跳出迴圈並暫停函式
            if not MODE == 'POS':
                # 歸零姿態數值
                TARGET_ROLL = 0
                TARGET_PITCH = 0
                print('<飛行狀態> 速度控制器：暫停')

        # 重設暫存器
        vel_int_error_x = 0
        vel_int_error_y = 0
        vel_previous_error_x = 0
        vel_previous_error_y = 0
        previous_time = time.time()

        # 記錄值
        R_POS_VEL_X_TAR.append(0)
        R_POS_VEL_Y_TAR.append(0)
        R_POS_VEL_X_ACT.append(0)
        R_POS_VEL_Y_ACT.append(0)

        R_POS_VEL_X_ERROR.append(0)
        R_POS_VEL_Y_ERROR.append(0)
        R_POS_VEL_X_GP.append(0)
        R_POS_VEL_X_GI.append(0)
        R_POS_VEL_X_GD.append(0)
        R_POS_VEL_Y_GP.append(0)
        R_POS_VEL_Y_GI.append(0)
        R_POS_VEL_Y_GD.append(0)

        R_POS_TARGET_ROLL.append(TARGET_ROLL)
        R_POS_TARGET_PITCH.append(TARGET_PITCH)
        R_POS_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
        R_POS_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

        # 迴圈執行間隔
        time.sleep(1 / FREQ_VEL)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_THREAD:
            print('<飛行狀態> 速度控制器：啟動')

        # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_VEL = False
    print('<關閉程序> 速度控制器：關閉')
