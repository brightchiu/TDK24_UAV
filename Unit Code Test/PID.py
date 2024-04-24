# 載入模組
from __future__ import print_function
from dronekit import connect
import time
import math

# ======設定全域變數======
print('<啟動程序> 載入全域變數設定')

# UAV 通訊參數
CONNECT_PORT = '/dev/serial0'
CONNECT_BAUD = 921600

# 與UAV執行連線
print('<啟動程序> 與UAV執行連線中，請稍候')
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線')


def get_data():
    uav.send_calibrate_barometer()
    while 1:
        hdg = uav.heading
        roll = math.degrees(uav.attitude.roll)
        pitch = math.degrees(uav.attitude.pitch)
        yaw = math.degrees(uav.attitude.yaw)
        alt = uav.location.global_relative_frame.alt
        vz = uav.velocity[2]
        print('Roll: %.3f    Pitch: %.3f    Yaw: %.3f' % (roll, pitch, yaw))
        print('HDG: %d   ALT: %.3f m   Vz: %.3f m/s\n' % (hdg, alt, vz))
        time.sleep(0.5)


get_data()
