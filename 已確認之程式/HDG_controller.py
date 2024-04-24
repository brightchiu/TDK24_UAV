# 航向控制器(待驗證)
# 包含三種模式：Roll/Yaw/Roll&Yaw

# 載入模組
import threading
import time

# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_HDG = False               # 航向控制器旗標
STATE_SENSOR_CAM = False        # 影像感測器旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->目標值(命令值)
TARGET_ROLL = 0                 # 目標滾轉角(degree)
TARGET_YAW_RATE = 0             # 偏航角速度(degree/s)，需搭配控制器控制角度

# ->實際值(量測值)
ACTUAL_HDG = 0                  # 與線夾角(degree)

# ========== 控制器參數設定 =============================================
# ->迴圈頻率(預設20Hz)
FREQ_HDG = 20                   # 航向控制器頻率(不快於影像禎率)

# ->航向控制器參數
HDG_MODE = ''                   # 航向控制器模式(R:Roll/Y:Yaw/RY:Roll&Yaw-預設)
HDG_KP = 0.1                    # P增益值*
HDG_KI = 0                      # I增益值*
HDG_KD = 0                      # D增益值*
HDG_DZ = 5                      # 截止區角度(degree)*
HDG_ROLL_ANGLE_LIMIT = 3        # 角度輸出限制(degree)*
HDG_ROLL_ANGLE_CUTOFF = 1       # 角度截止限制(degree)*
HDG_YAW_RATE_LIMIT = 10         # 角速度輸出限制(degree/s)*
HDG_ROLL_YAW_RATIO = 5          # 模式RY下滾轉偏航比(大於一以滾轉為主，小於一以偏航為主)

# ->航向控制器
R_HDG_ACTUAL = []                       # 記錄實際角度
R_HDG_TARGET_ROLL = []                  # 記錄滾轉輸出
R_HDG_TARGET_YAW_RATE = []              # 記錄目標偏航速率
R_HDG_ERROR = []                        # 記錄高度誤差
R_HDG_GP = []                           # 記錄P增益
R_HDG_GI = []                           # 記錄I增益
R_HDG_GD = []                           # 記錄D增益


# =========== 航向控制器 ===============================================
# 說明：
# 採用PID控制器型態，每一迴圈讀取一次影像計算線條型心，並將型心值做均值綠波
# 濾波完畢後再以該值計算出型心至畫面下方中心之夾角，並以此值為控制器的誤差值
# 以該值來做PID增益處理，輸出角速度來追尋角度，此控制器可處理平行、轉角、Ｔ路口轉向
# 為不可逆型循跡控制器，參數可調最大角速度輸出、切斷區間、濾波等級、面積門檻值
# 使用Global-Frame參照，以地面標線為參照座標，預測為PD控制器(前進速度控制分離)
def heading_controller():
    global TARGET_ROLL, TARGET_YAW_RATE, STATE_HDG
    # PID變數設定
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    # 等待相機啟動
    if not STATE_SENSOR_CAM:
        print('<啟動程序> 位置控制器：等待影像感測器啟動')
    while not STATE_SENSOR_CAM:
        time.sleep(0.1)

    # 回報定位控制器啟用
    STATE_HDG = True
    print('<啟動程序> 航向控制器：啟動')

    # 程式迴圈
    while STATE_THREAD:
        # 定位作動迴圈(同捆執行模式，於循跡模式作動)
        while MODE == 'LINE' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 讀取回授值
            hdg_actual = ACTUAL_HDG

            # 回授值截止區修正
            if HDG_DZ > hdg_actual > - HDG_DZ:
                hdg_actual = 0

            # 計算誤差值
            error = -hdg_actual

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
            gain_out = gain_p + gain_i + gain_d

            # 輸出模式
            if HDG_MODE == 'R':
                roll_out = gain_out
                angular_rate = 0
            elif HDG_MODE == 'Y':
                roll_out = 0
                angular_rate = gain_out
            else:
                roll_out = gain_out * HDG_ROLL_YAW_RATIO
                angular_rate = gain_out

            # 角度上下限制(包含截止區設定)
            if roll_out > HDG_ROLL_ANGLE_LIMIT:
                roll_out = HDG_ROLL_ANGLE_LIMIT
            elif roll_out < -HDG_ROLL_ANGLE_LIMIT:
                roll_out = -HDG_ROLL_ANGLE_LIMIT
            elif -HDG_ROLL_ANGLE_CUTOFF < roll_out < HDG_ROLL_ANGLE_CUTOFF:
                roll_out = 0

            # 角速度上下限制
            if angular_rate > HDG_YAW_RATE_LIMIT:
                angular_rate = HDG_YAW_RATE_LIMIT
            elif angular_rate < -HDG_YAW_RATE_LIMIT:
                angular_rate = -HDG_YAW_RATE_LIMIT

            # 更新目標值至全域變數
            TARGET_ROLL = roll_out
            TARGET_YAW_RATE = angular_rate

            # 傳遞值給下次迴圈使用
            previous_error = error
            previous_time = current_time

            # 記錄數值
            R_HDG_ACTUAL.append(hdg_actual)
            R_HDG_TARGET_ROLL.append(TARGET_ROLL)
            R_HDG_TARGET_YAW_RATE.append(TARGET_YAW_RATE)

            R_HDG_ERROR.append(error)
            R_HDG_GP.append(gain_p)
            R_HDG_GI.append(gain_i)
            R_HDG_GD.append(gain_d)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_HDG)

            # 當飛行模式不為循跡時，跳出迴圈並暫停函式
            if not MODE == 'LINE':
                # 歸零姿態數值
                TARGET_ROLL = 0
                TARGET_YAW_RATE = 0
                STATE_HDG = False
                print('<飛行狀態> 航向控制器：暫停')

        # 無循跡作動區間
        # 回報迴圈休眠狀態
        if MODE == 'POS' and STATE_HDG:
            STATE_HDG = False
            print('<飛行狀態> 航向控制器：暫停')

        # 重設暫存器
        int_error = 0
        previous_error = 0
        previous_time = time.time()

        # 記錄數值
        R_HDG_ACTUAL.append(0)
        R_HDG_TARGET_ROLL.append(0)
        R_HDG_TARGET_YAW_RATE.append(0)

        R_HDG_ERROR.append(0)
        R_HDG_GP.append(0)
        R_HDG_GI.append(0)
        R_HDG_GD.append(0)

        # 迴圈執行間隔
        time.sleep(1 / FREQ_HDG)

        # 當飛行模式為循跡時，跳入迴圈並啟動函式
        if MODE == 'LINE' and STATE_THREAD:
            STATE_HDG = True
            print('<飛行狀態> 航向控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_HDG = False
    print('<關閉程序> 航向控制器：關閉')
