# 沙包盒控制參數
# 需確認PWM數值


import RPi.GPIO as GPIO

STATE_BOX = False               # 貨物艙門狀態旗標

# ========== 貨物艙參數 ================================================
# ->沙包盒參數
BOX_PWM_PIN = 12                # 沙包盒PWM輸出腳位
BOX_PWM_FREQ = 50               # PWM頻率(方波Hz)
BOX_PWM_OPEN = 1500             # 開門位置(400~2350us)*
BOX_PWM_CLOSE = 2000            # 關門位置(400~2350us)*


# ========= 沙包盒初始化 ===============================================
# 於起飛時送出脈波訊號固定艙門，防止貨物掉出，並維持較佳的空氣動力學外型
def box_initialize():
    global STATE_BOX, BOX
    # 初始化腳位
    GPIO.setmode(GPIO.BCM)                      # 設定為BCM模式
    GPIO.setwarnings(False)                     # 忽略警告
    GPIO.setup(BOX_PWM_PIN, GPIO.OUT)           # PWM腳位輸出設定
    BOX = GPIO.PWM(BOX_PWM_PIN, BOX_PWM_FREQ)   # 建立PWM控制物件

    # 啟動PWM控制，使門維持關閉
    dc = BOX_PWM_FREQ * (BOX_PWM_CLOSE / 1000000) * 100
    BOX.start(dc)

    # 回報艙門狀態
    STATE_BOX = True
    print('<飛行狀態> 沙包盒艙門：關閉\n')


# ========= 沙包盒關門 =================================================
# 於起飛時送出脈波訊號固定艙門，防止貨物掉出，並維持較佳的空氣動力學外型
def box_close():
    global STATE_BOX
    # 啟動PWM控制，使門維持關閉
    dc = BOX_PWM_FREQ * (BOX_PWM_CLOSE / 1000000) * 100
    BOX.ChangeDutyCycle(dc)

    # 回報艙門狀態
    STATE_BOX = True
    print('<飛行狀態> 沙包盒艙門：關閉\n')


# ======== 沙包盒開門 =================================================
# 輸出訊號使艙門開啟，使貨物卸出
def box_open():
    global STATE_BOX
    # 啟動PWM控制，使門打開
    dc = BOX_PWM_FREQ * (BOX_PWM_OPEN / 1000000) * 100
    BOX.ChangeDutyCycle(dc)

    # 回報艙門狀態
    STATE_BOX = False
    print('<飛行狀態> 沙包盒艙門：開啟\n')
