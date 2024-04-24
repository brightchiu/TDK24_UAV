import serial
import time

STATE_THREAD = True
ACTUAL_ALT_LASER = 0
ACTUAL_AX = 0
ACTUAL_AY = 0
ACTUAL_AZ = 0

# ========== Arduino連線參數 ==========================================
ARDUINO_CONNECT_PORT = '/dev/cu.usbmodem14501'       # 傳輸口位址
ARDUINO_CONNECT_BAUD = 9600                 # 傳輸鮑率

# ========== 與Arduino執行連線 =============================================
# print('<啟動程序> 與UAV連線中，請稍候...')

# 連線指令
t_start = time.time()
nano = serial.Serial(port=ARDUINO_CONNECT_PORT, baudrate=ARDUINO_CONNECT_BAUD, timeout=0.1)
# nano.open()
t_end = time.time()
print('<啟動程序> Arduino已連線，花費%.2d秒' % (t_end-t_start))


# ========== 感測接收程式 ==============================================
def get_sensor_data():
    global ACTUAL_ALT_LASER, ACTUAL_AX, ACTUAL_AY, ACTUAL_AZ
    alt_laser = 0
    ax = 0
    ay = 0
    az = 0
    while STATE_THREAD:
        # 等待非完整資料清空
        wait = str(nano.readline())
        # 讀取資料直到換行
        sen_raw_data = str(nano.readline())

        # 切出資料
        sen_data = sen_raw_data.split(sep='_' or '\'')
        # 抓取資料
        for x in range(len(sen_data)):
            if sen_data[x] == 'altlaser':
                alt_laser = sen_data[x+1]
            elif sen_data[x] == 'ax':
                ax = sen_data[x+1]
            elif sen_data[x] == 'ay':
                ay = sen_data[x+1]
            elif sen_data[x] == 'az':
                az = sen_data[x+1]
        # 寫入至全域變數
        ACTUAL_ALT_LASER = float(alt_laser)/1000
        ACTUAL_AX = float(ax)/1000
        ACTUAL_AY = float(ay)/1000
        ACTUAL_AZ = float(az)/1000
        print('\naltlaser', ACTUAL_ALT_LASER, 'ax', ACTUAL_AX, 'ay', ACTUAL_AY, 'az', ACTUAL_AZ)

        # 等待下一迴圈
        time.sleep(0.06)


get_sensor_data()