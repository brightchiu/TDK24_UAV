import VL53L1X
import time
import numpy as np
import threading

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

LASER_OFFSET = 0
LASER_SAMPLE_NUM_CAL = 10


# ========== 雷射測距儀校正程序 =========================================
def laser_initialize():
    global LASER_OFFSET
    # 建立雷射量測物件
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    tof.open()

    # 設定量測模式-中距模式
    tof.start_ranging(2)

    # 平均降噪設定
    height_register = []

    # 量測迴圈(線程優先執行)
    THREAD_OCCUPY.acquire()
    for i in range(0, LASER_SAMPLE_NUM_CAL):
        # 量測迴圈
        height_raw = tof.get_distance()

        # 均值濾波器
        height_register.append(height_raw)

        # 等待50ms執行下次量測
        time.sleep(0.05)

    # 關閉量測物件(防止衝突)
    tof.stop_ranging()

    # 結束優先執行
    THREAD_OCCUPY.release()

    # 更新高度校正偏移量
    LASER_OFFSET = -np.average(height_register)
    print('<啟動程序> 雷射測距儀校正完畢，偏移 %.3f m\n' % LASER_OFFSET)


# 初始化雷射測距儀
laser_initialize()

# 重新建立雷射量測物件
laser = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
laser.open()
laser.start_ranging(2)

for i in range(10):
    # 雷射測距儀讀值與補正
    height_raw_laser = laser.get_distance() + LASER_OFFSET
    print(height_raw_laser)
    print(laser.get_timing())

    time.sleep(0.5)
