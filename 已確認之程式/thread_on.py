
from dronekit import connect
import threading
import time

# ========== UAV通訊參數 ==============================================
# 樹莓派Serial連線參數
CONNECT_PORT = '/dev/serial0'   # 傳輸口位址
CONNECT_BAUD = 921600           # 傳輸鮑率

# ========== 與UAV執行連線 =============================================
print('<啟動程序> 與UAV連線中，請稍候...')
# 樹莓派連線指令
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)


# ========== 多線程啟動函式 ============================================
# 供主程式啟動多線程用，若連線中斷時，自動重新連線
def thread_activate():
    global STATE_THREAD, STATE_UAV, uav
    # 若未連線，重新執行連線
    if not STATE_UAV:
        print('<啟動程序> 與UAV連線中，請稍候...')
        # 樹莓派連線指令
        uav = connect(CONNECT_PORT, baud=CONNECT_BAUD)
        # 電腦連線指令
        # uav = connect(CONNECT_PORT, baud=CONNECT_BAUD)
        STATE_UAV = True
        print('<啟動程序> UAV已連線')

    # 定義多線程
    cam = threading.Thread(target=camera)
    cam_sen = threading.Thread(target=camera_sensor)
    sonar_sen = threading.Thread(target=altitude_sensor_sonar)
    alt_sen = threading.Thread(target=altitude_sensor)
    spd_sen = threading.Thread(target=speed_sensor)
    alt_ctrl = threading.Thread(target=altitude_controller)
    pos_ctrl = threading.Thread(target=position_controller)
    pos_vel_ctrl = threading.Thread(target=position_velocity_controller)
    hdg_ctrl = threading.Thread(target=heading_controller)
    spd_ctrl = threading.Thread(target=flight_speed_controller)
    cmd = threading.Thread(target=command_transfer)

    # 啟動多線程旗標
    STATE_THREAD = True

    # 執行多線程(依啟動順序放置)
    cam.start()
    cam_sen.start()
    sonar_sen.start()
    alt_sen.start()
    spd_sen.start()
    alt_ctrl.start()
    pos_ctrl.start()
    pos_vel_ctrl.start()
    hdg_ctrl.start()
    spd_ctrl.start()
    cmd.start()

    # 回報多線程啟動
    time.sleep(5)
    print('<啟動程序> 多線程程序：啟動')
