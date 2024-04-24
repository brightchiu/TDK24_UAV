# 飛行速度感測器

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

# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_UAV = True                # 無人機連線狀態
STATE_SENSOR_SPD = False        # 飛行速度感測器旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->實際值(量測值)
ACTUAL_SPD = 0                  # 實際飛行速度

# ========== 控制器參數設定 =============================================
# ->迴圈頻率(預設20Hz)
FREQ_SENSOR_SPD = 20            # 飛行速度感測器頻率

SPD_SAMPLE_NUM_AVE = 5          # 飛行速度感測器均值濾波取樣次數*

# ========== 與UAV執行連線 =============================================
print('<啟動程序> 與UAV連線中，請稍候...')
# 樹莓派連線指令
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)


# =========== 飛行速度感測器 ============================================
# 說明：
# 計算循跡時的飛行速度，利用IMU的速度值計算合速度
def speed_sensor():
    global ACTUAL_SPD, STATE_SENSOR_SPD
    # 均值濾波器設定
    speed_register = []

    # 等待無人機連線
    if not STATE_UAV:
        print('<啟動程序> 飛行速度感測器：等待無人機連線')
    while not STATE_UAV:
        time.sleep(0.1)

    # 回報飛行速度控制器啟用
    STATE_SENSOR_SPD = True
    print('<啟動程序> 飛行速度感測器：啟動')

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        # 讀取速度值
        vx = uav.velocity[0]
        vy = uav.velocity[1]

        # 計算和速度
        if MODE == 'POS':
            speed_raw = 0
        else:
            speed_raw = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))

        # 均值濾波處理
        speed_register.insert(0, speed_raw)
        if len(speed_register) > SPD_SAMPLE_NUM_AVE:
            speed_register.pop()
        speed = np.average(speed_register)

        # 方向判斷
        hdg = uav.heading
        if vx > 0 and (hdg < 90 or hdg > 270):
            speed = speed
        elif vx > 0 and (90 < hdg < 270):
            speed = -speed
        elif vx < 0 and (hdg < 90 or hdg > 270):
            speed = -speed
        elif vx < 0 and (90 < hdg < 270):
            speed = speed
        elif vx == 0:
            if vy > 0 and hdg == 90:
                speed = speed
            elif vy > 0 and hdg == 270:
                speed = -speed
            elif vy < 0 and hdg == 270:
                speed = speed
            elif vy < 0 and hdg == 90:
                speed = -speed

        # 輸出至全域變數
        ACTUAL_SPD = speed

        # 迴圈執行間隔
        time.sleep(1 / FREQ_SENSOR_SPD)

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_SENSOR_SPD = False
    print('<關閉程序> 飛行速度感測器：關閉')
