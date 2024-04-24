# 航向控制器 控制器物件導向寫法

# 載入模組
from dronekit import connect, VehicleMode
from openpyxl import Workbook
# import dronekit_sitl
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math
import cv2

# ====================================================================
# ========== 設定全域變數 ==============================================
# ====================================================================
# 加註*字為可調整之參數
print('<啟動程序> 載入全域變數設定')

# ========== UAV通訊參數 ==============================================
# 樹莓派Serial連線參數
CONNECT_PORT = '/dev/serial0'   # 傳輸口位址
CONNECT_BAUD = 921600           # 傳輸鮑率

# MacBook 3DR連線參數
# CONNECT_PORT = '/dev/tty.SLAB_USBtoUART'
# CONNECT_BAUD = 57600

# 軟體模擬連線參數
# sitl = dronekit_sitl.start_default()
# CONNECT_PORT = sitl.connection_string()
# CONNECT_BAUD = 921600

# ========== 旗標與變數 ================================================
# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_UAV = True                # 無人機連線狀態
STATE_ALT = False               # 高度控制器旗標
STATE_POS = False               # 位置控制器旗標
STATE_VEL = False               # 位置-速度控制器旗標
STATE_HDG = False               # 航向控制器旗標
STATE_SPD = False               # 飛行速度控制器旗標
STATE_CAM = False               # 影像擷取程式旗標
STATE_SENSOR_CAM = False        # 影像感測器旗標
STATE_SENSOR_ALT = False        # 高度感測器旗標
STATE_SENSOR_SONAR = False      # 超音波高度計旗標
STATE_SENSOR_SPD = False        # 飛行速度感測器旗標
STATE_CMD = False               # 命令傳輸程式旗標
STATE_BOX = False               # 貨物艙門狀態旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)
MISSION_COLOR = ''              # 任務顏色(blue or green)
POS_COLOR = ''                  # 定位的色塊顏色

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->目標值(命令值)
TARGET_ROLL = 0                 # 目標滾轉角(degree)
TARGET_PITCH = 0                # 目標俯仰角(degree)
TARGET_YAW = 0                  # 目標偏航角(degree)，不使用
TARGET_YAW_RATE = 0             # 偏航角速度(degree/s)，需搭配控制器控制角度
TARGET_THRUST = 0               # 目標推力，表示升降速度(0.5為中點)
TARGET_ALT = 0                  # 目標高度，主程式輸入於高度控制器(m)
TARGET_VEL = [0, 0]             # 目標移動速度(px/s，相機座標)
TARGET_POS = [0, 0]             # 目標型心位置(px，相機座標)
TARGET_SPD = 0                  # 目標飛行速度

# ->實際值(量測值)
ACTUAL_ALT = 0                  # 實際高度(m)
ACTUAL_ALT_SONAR = 0            # 超音波高度計(m)
ACTUAL_ALT_IMU = 0              # IMU氣壓高度計(m)
ACTUAL_HDG = 0                  # 與線夾角(degree)
ACTUAL_POS = [0, 0]             # 色塊實際位置(px)
ACTUAL_VEL = [0, 0]             # 色塊實際速度(px/s)
ACTUAL_AREA = 0                 # 色塊面積(px^2)
ACTUAL_SPD = 0                  # 實際飛行速度

# ========== 控制器參數設定 =============================================
# ->迴圈頻率(預設20Hz)
FREQ_ALT = 20                   # 高度控制器頻率(不快於高度感測器頻率)
FREQ_POS = 10                   # 位置控制器頻率(不快於位置-速度控制器頻率)
FREQ_VEL = 20                   # 位置-速度控制器頻率(不快於影像禎率)
FREQ_HDG = 20                   # 航向控制器頻率(不快於影像禎率)
FREQ_SPD = 20                   # 飛行速度控制器頻率
FREQ_CAM = 30                   # 影像禎率(fps，依系統負載調整)
FREQ_SENSOR_CAM = 20            # 影像感測器頻率(不快於影像禎率)
FREQ_SENSOR_ALT = 20            # 高度感測器頻率(不快於超音波感測器頻率)
FREQ_SENSOR_SONAR = 20          # 超音波感測器頻率(5~50Hz，越高雜訊越多)
FREQ_SENSOR_SPD = 20            # 飛行速度感測器頻率
FREQ_CMD = 20                   # 命令傳輸頻率(不快於控制器頻率)

# ->高度控制器參數
ALT_KP = 0.1                    # P增益值*
ALT_KI = 0                      # I增益值*
ALT_KD = 0.1                    # D增益值*
ALT_GAIN_D_LIMIT = 0.3
ALT_SPEED = 0.3                 # 昇降速率(m/s，與飛控參數一致)*

# ->位置控制器參數
POS_SAMPLE_NUM_AVE = 10         # 位置控制器均值濾波取樣次數*
POS_X_KP = 0.01                 # 位置控制器X軸-P增益值
POS_X_KI = 0                    # 位置控制器X軸-I增益值
POS_X_KD = 0                    # 位置控制器X軸-D增益值
POS_Y_KP = 0.01                 # 位置控制器Y軸-P增益值
POS_Y_KI = 0                    # 位置控制器Y軸-I增益值
POS_Y_KD = 0                    # 位置控制器Y軸-D增益值
POS_VEL_LIMIT = 2000            # 位置控制器-速度輸出限制(px/s，1.2m高時1px=3.02mm，不使用)
POS_DZ = 50                     # 位置控制器截止區參數(於截止區內使速度控制器控制零速度，防震盪)
# 特殊附註：截止區調校方式=>區域內阻力可以抵消慣性、控制器解析程度內(控制器截止區)

# ->速度控制器參數
POS_VEL_X_KP = 0.1              # 速度控制器X軸-P增益值
POS_VEL_X_KI = 0                # 速度控制器X軸-I增益值
POS_VEL_X_KD = 0                # 速度控制器X軸-D增益值
POS_VEL_Y_KP = 0.1              # 速度控制器Y軸-P增益值
POS_VEL_Y_KI = 0                # 速度控制器Y軸-I增益值
POS_VEL_Y_KD = 0                # 速度控制器Y軸-D增益值
POS_ANGLE_LIMIT = 3             # 速度控制器-最大飛行角度(degree)
POS_ANGLE_CUTOFF = 1            # 速度控制器-最小飛行角度(degree)
POS_VEL_X_FEEDBACK_GAIN = 0.01  # X軸速度回授增益
POS_VEL_Y_FEEDBACK_GAIN = 0.01  # Y軸速度回授增益
POS_VEL_CUTOFF_ALT = 0.5        # 控制器最低作動高度

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

# ->飛行速度控制器參數
SPD_KP = 0.1                    # P增益值*
SPD_KI = 0                      # I增益值*
SPD_KD = 0                      # D增益值*
SPD_ANGLE_CUTOFF = 1            # 飛行速度控制器-最小俯仰角度(degree)
SPD_ANGLE_LIMIT = 3             # 飛行速度控制器-最大俯仰角度(degree)
SPD_LIMIT = 0.5                 # 飛行速度控制器-最大飛行速度(m/s)
SPD_OFF_ANGLE = 5               # 切換跳出角度(degree)，慣性抵銷用
SPD_SAMPLE_NUM_AVE = 5          # 飛行速度感測器均值濾波取樣次數*

# ->飛行程序參數
TAKEOFF_SETTLE = 95             # 安定高度百分比(%，高度確認門檻)*
LANDING_CUTOFF = 0.05           # 油門關閉高度(m，停止油門輸出)*
TURN_SETTLE = 95                # 安定轉角百分比(%，停止轉向輸出)*
TURN_YAW_RATE = 30              # 轉角速度(degree/s)*
TURN_AREA_LIMIT = 3000          # 號誌燈號面積閥值(px)
DROP_POS_LIMIT_X = 100                      # 空投定位X範圍限制(單向，px)*
DROP_POS_LIMIT_Y = DROP_POS_LIMIT_X*(3/4)   # 空投定位Y範圍限制(單向，px)*
AREA_ARRIVE_LIMIT = 4000        # 色塊抵達偵測門檻(px)
AREA_LEAVE_LIMIT = 3000         # 色塊離開偵測門檻(px)

# ========== 影像參數設定 ===============================================
# ->相機設定參數
CAM_VIDEO = cv2.VideoCapture(0)             # 影像來源(預設來源為0)
CAM_IMAGE = 0                               # 儲存影像
CAM_IMAGE_LIVE = 0                          # 即時影像
CAM_IMAGE_WIDTH = 480                       # 影像寬度(px，4:3設定 )
CAM_IMAGE_HEIGHT = 360                      # 影像高度(px)
VIDEO_FOURCC = cv2.VideoWriter_fourcc(*'XVID')
VIDEO_SAVE = cv2.VideoWriter('Flight Camera.avi',
                             VIDEO_FOURCC, FREQ_CAM, (CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT))

# ->影像計算參數
IMG_BLUE_L = np.array([85, 76, 90])         # 藍色色域-下限
IMG_BLUE_H = np.array([127, 255, 255])      # 藍色色域-上限
IMG_GREEN_L = np.array([44, 104, 27])       # 綠色色域-下限
IMG_GREEN_H = np.array([87, 255, 255])      # 綠色色域-上限
IMG_RED_L = np.array([160, 111, 56])        # 紅色色域-下限
IMG_RED_H = np.array([180, 255, 255])       # 紅色色域-上限
IMG_BLACK_L = np.array([0, 0, 0])           # 黑色色域-下限
IMG_BLACK_H = np.array([180, 255, 75])      # 黑色色域-上限
IMG_MORPHOLOGY_EX_KERNEL = (5, 5)           # 閉運算矩陣大小(奇數，越大降噪能力越強)*
IMG_AREA_LIMIT = 3000                       # 取用面積下限值*
IMG_SAMPLE_NUM_AVE = 10                     # 均值濾波取樣次數*
BODY_REF_X = int(CAM_IMAGE_WIDTH / 2)       # 畫面參考X座標中心
BODY_REF_Y = int(CAM_IMAGE_HEIGHT / 2)      # 畫面位置Y座標中心
BODY_OFFSET_X = 0                           # 機身中心X偏移量(px，左負右正)*
BODY_OFFSET_Y = 0                           # 機身中心Y偏移量(px，上正下負)*

# ->線角度計算
ANG_THRESHOLD = 40              # 二值化門檻值(調高比較能看到線，但可能有雜訊)*
ANG_THRESHOLD_NEW = 255         # 取得黑白照填入最高值
ANG_GB_KERNEL_SIZE = (7, 7)     # 高斯模糊矩陣大小(奇數，越大越模糊)*
ANG_GB_SIGMA = 2                # 高斯模糊色域標準差(越大越模糊)*
ANG_LINE_TRACK_AREA_L = 450     # 線追蹤型心計算面積下限*
ANG_SAMPLE_NUM_AVE = 10         # 航向控制器均值濾波取樣次數*

# ========== 貨物艙參數 ================================================
# ->沙包盒參數
BOX_PWM_PIN = 12                # 沙包盒PWM輸出腳位
BOX_PWM_FREQ = 50               # PWM頻率(方波Hz)
BOX_PWM_OPEN = 1500             # 開門位置(400~2350us)*
BOX_PWM_CLOSE = 2000            # 關門位置(400~2350us)*

# ========== 高度感測器參數 =============================================
# ->高度感測器整合參數
IMU_SAMPLE_NUM_AVE = 10         # 均值濾波取樣次數*
WEIGHTED_SONAR = 5              # 超音波高度計權重*
WEIGHTED_IMU = 5                # IMU高度計權重*

# ->超音波高度計參數
SONAR_AIR_TEMP = 28             # 氣溫(攝氏)*
SONAR_TRIG_PIN = 2              # TRIG輸出腳位
SONAR_ECHO_PIN = 3              # ECHO輸入腳位
SONAR_TIMEOUT = 0.015           # 無回應超時限制(秒，過低時遠距離偵測將會變成超時)*
SONAR_OFFSET = 0                # 高度校正偏移量(m，向上為正)
SONAR_SAMPLE_NUM_CAL = 10       # 高度校正取樣次數*
SONAR_SAMPLE_NUM_AVE = 10       # 均值濾波取樣次數*
SONAR_KF_INPUT_MAX = 3          # 卡爾曼濾波器輸入值限制(高度，m)*
SONAR_KF_Q = 5                  # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*
SONAR_ERR_COUNT = 0             # 量測超時次數記錄
SONAR_FILTER_ALT = 1            # 突波濾波作用起始高度(m)
SONAR_MAX_CHANGE = 0.5          # 突波容許值(m)

# ->IMU高度計參數
IMU_OFFSET = 0                  # 高度校正偏移量(m，向上為正)
IMU_SAMPLE_NUM_CAL = 10         # 高度校正取樣次數*
IMU_KF_INPUT_MAX = 3            # 卡爾曼濾波器輸入值限制(高度，m)*
IMU_KF_Q = 5                    # 卡爾曼濾波器滑順度(越小反應快，越不平；越大反應慢，但較平滑)*

# ========== 數值記錄 ==================================================
# ->高度控制器
R_TARGET_ALT = []                       # 記錄目標高
R_ACTUAL_ALT_CTRL = []                  # 記錄實際高
R_TARGET_THRUST = []                    # 記錄目標推力
R_ALT_ERROR = []                        # 記錄高度誤差
R_ALT_GP = []                           # 記錄P增益
R_ALT_GI = []                           # 記錄I增益
R_ALT_GD = []                           # 記錄D增益

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

# ->航向控制器
R_HDG_ACTUAL = []                       # 記錄實際角度
R_HDG_TARGET_ROLL = []                  # 記錄滾轉輸出
R_HDG_TARGET_YAW_RATE = []              # 記錄目標偏航速率
R_HDG_ERROR = []                        # 記錄角度誤差
R_HDG_GP = []                           # 記錄P增益
R_HDG_GI = []                           # 記錄I增益
R_HDG_GD = []                           # 記錄D增益

# ->飛行速度控制器
R_SPD_TARGET = []                       # 記錄目標速度
R_SPD_ACTUAL = []                       # 記錄實際速度
R_SPD_TARGET_PITCH = []                 # 記錄目標俯仰角
R_SPD_ACTUAL_PITCH = []                 # 記錄實際俯仰角
R_SPD_ERROR = []                        # 記錄速度誤差
R_SPD_GP = []                           # 記錄P增益
R_SPD_GI = []                           # 記錄I增益
R_SPD_GD = []                           # 記錄D增益

# ->飛行速度控制器
R_ACTUAL_ALT = []                       # 記錄加權值
R_ACTUAL_ALT_SONAR = []                 # 記錄超音波高度計值
R_ACTUAL_ALT_BAROM = []                 # 記錄氣壓高度計值

# ->超音波高度計
R_ACTUAL_ALT_SONAR_RAW = []             # 記錄原始值
R_ACTUAL_ALT_SONAR_OFFSET = []          # 記錄補正值
R_ACTUAL_ALT_SONAR_AVE = []             # 記錄平均值
R_ACTUAL_ALT_SONAR_KF = []              # 記錄KF值

# ->氣壓高度計
R_ACTUAL_ALT_IMU_RAW = []               # 記錄原始值
R_ACTUAL_ALT_IMU_OFFSET = []            # 記錄補正值
R_ACTUAL_ALT_IMU_AVE = []               # 記錄平均值
R_ACTUAL_ALT_IMU_KF = []                # 記錄KF值

print('<啟動程序> 載入全域變數設定完畢')

# ========== 與UAV執行連線 =============================================
print('<啟動程序> 與UAV連線中，請稍候...')
# 樹莓派連線指令
uav = connect(CONNECT_PORT, wait_ready=True, baud=CONNECT_BAUD)
# 電腦連線指令
# uav = connect(CONNECT_PORT, baud=CONNECT_BAUD)
print('<啟動程序> UAV已連線')

# ====================================================================
# ========== 控制器物件類區塊 ===========================================
# ====================================================================


# 控制器類式
class PidController:
    """
    Use this PID Controller should call set_gain and set_limit(option).
    Then call reference_update, feedback_update, ctrl_process.
    if controller is not used, call reset_controller before reuse.
    """
    time = __import__('time')

    # 初始化設定
    def __init__(self):
        self.reference = 0
        self.feedback = 0
        self.error = 0
        self.int_error = 0
        self.previous_error = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.gain_p = 0
        self.gain_i = 0
        self.gain_d = 0
        self.integral_limit = 10000
        self.gain_d_limit = 100000
        self.previous_time = self.time.time()
        self.first_loop_stamp = True

    # 設定控制器增益值
    def set_gain(self, kp=0.0, ki=0.0, kd=0.0):
        """
        Set PID gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    # 設定增益限制
    def set_limit(self, integral_limit, gain_d_limit):
        """
        Set integral and derivative limit.
        """
        self.integral_limit = integral_limit
        self.gain_d_limit = gain_d_limit

    # 更新目標值
    def reference_update(self, r):
        self.reference = r

    # 更新回授值
    def feedback_update(self, h):
        self.feedback = h

    def ctrl_process(self):
        # 更新時間標記
        current_time = self.time.time()

        # 第一迴圈圈僅作時間更新
        if self.first_loop_stamp:
            self.previous_time = current_time
            self.first_loop_stamp = False
            return 0

        # 後續迴圈正常執行
        else:
            # 計算誤差值
            self.error = self.reference - self.feedback

            # 誤差值微積分計算
            delta_time = current_time - self.previous_time
            delta_error = self.error - self.previous_error
            self.int_error += (self.error * delta_time)
            derivative = delta_error / delta_time

            # 積分器限制
            if self.int_error > self.integral_limit:
                self.int_error = self.integral_limit
            elif self.int_error < -self.integral_limit:
                self.int_error = -self.integral_limit

            # 誤差增益處理
            self.gain_p = self.error * self.kp
            self.gain_i = self.int_error * self.ki
            self.gain_d = derivative * self.kp

            # 微分增益限制器
            if self.gain_d > self.gain_d_limit:
                self.gain_d = self.gain_d_limit
            elif self.gain_d < -self.gain_d_limit:
                self.gain_d = -self.gain_d_limit

            # 加總輸出
            gain_out = self.gain_p + self.gain_i + self.gain_d

            # 更新數值
            self.previous_error = self.error
            self.previous_time = current_time

            # 返還增益值
            return gain_out

    # 重設控制器(於休眠後重新啟用時輸入)
    def reset_controller(self):
        self.feedback = 0
        self.int_error = 0
        self.previous_error = 0
        self.previous_time = self.time.time()
        self.first_loop_stamp = True


# 控制器計時器
class ControllerTimer:
    time = __import__('time')

    # 初始化設定
    def __init__(self):
        self.t_start = self.time.time()

    # 計時開始
    def time_start(self):
        self.t_start = self.time.time()

    # 等待時間抵達(使用頻率輸入)
    def wait_time(self, freq):
        t_elapsed = 1 / freq
        t_current = self.time.time()
        while t_current - self.t_start < t_elapsed:
            t_current = self.time.time()


# 移動平均濾波器
class MovingAverageFilter:
    np = __import__('numpy')

    # 初始化設定
    def __init__(self, sample_number=10):
        self.register = []
        self.sample_number = sample_number

    # 計算平均值
    def calculate(self, value):
        self.register.insert(0, value)
        if len(self.register) > self.sample_number:
            self.register.pop()
        maf = self.np.average(self.register)

        return maf

    # 重設暫存器
    def reset_register(self):
        self.register = []


# ====================================================================
# ========== 背景主函式函式區塊 =========================================
# ====================================================================
# 本區為主要運作的函式，以控制器為主，包含高度、位置、航向、雙高度感測器、
# 相機、命令傳送，多線程執行時能達到協作的處理功能
# (但仍受每個迴圈處理時間不一影響，為非同步處理)


# ========== 高度控制器 ================================================
# 說明：
# 為PID控制器型態，於任務期間全程作用，輸入目標高度後，依據高度感測器執行誤差PID增益，
# 最終輸出成爬升速率型態(符合MavLink指令)，設有輸出範圍限制器、中點飽和區間功能
def altitude_controller():
    global TARGET_THRUST, STATE_ALT
    # 等待感測器啟動
    if not STATE_SENSOR_ALT:
        print('<啟動程序> 高度控制器：等待高度感測器啟動')
    while not STATE_SENSOR_ALT:
        time.sleep(0.1)

    # 回報高度控制器啟動
    STATE_ALT = True
    print('<啟動程序> 高度控制器：啟動')

    # 建立控制器物件
    alt_ctrl = PidController()
    alt_ctrl.set_gain(ALT_KP, ALT_KI, ALT_KD)
    alt_timer = ControllerTimer()

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        # 同捆執行起點
        THREAD_OCCUPY.acquire()

        # 計時開始
        alt_timer.time_start()

        # 更新控制器
        alt_ctrl.reference_update(TARGET_ALT)
        alt_ctrl.feedback_update(ACTUAL_ALT)

        # 執行控制器
        speed_out = alt_ctrl.ctrl_process()

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

        # 記錄數值
        R_TARGET_ALT.append(TARGET_ALT)
        R_ACTUAL_ALT_CTRL.append(ACTUAL_ALT)
        R_TARGET_THRUST.append(TARGET_THRUST)
        R_ALT_ERROR.append(alt_ctrl.error)
        R_ALT_GP.append(alt_ctrl.gain_p)
        R_ALT_GI.append(alt_ctrl.gain_i)
        R_ALT_GD.append(alt_ctrl.gain_d)

        # 同捆執行終點
        THREAD_OCCUPY.release()

        # 迴圈執行間隔
        alt_timer.wait_time(FREQ_ALT)

    # 當多線程狀態為否時，跳出迴圈並關閉函式
    STATE_ALT = False
    print('<關閉程序> 高度控制器：關閉')


# ========== 位置控制器 ================================================
# 說明：
# 外迴圈的位置控制
# 先以影像取得的色塊座標計算速度增益值，在輸出至速度控制器做角度-加速度控制
def position_controller():
    global TARGET_VEL, STATE_POS
    # 等待影像感測器啟動
    if not STATE_SENSOR_CAM:
        print('<啟動程序> 位置控制器：等待影像感測器啟動')
    while not STATE_SENSOR_CAM:
        time.sleep(0.1)

    # 回報位置控制器啟用
    STATE_POS = True
    print('<啟動程序> 位置控制器：啟動')

    # 均值濾波設定
    vel_x_maf = MovingAverageFilter(POS_SAMPLE_NUM_AVE)
    vel_y_maf = MovingAverageFilter(POS_SAMPLE_NUM_AVE)

    # 建立控制器物件
    pos_x_ctrl = PidController()
    pos_x_ctrl.set_gain(POS_X_KP, POS_X_KI, POS_X_KD)
    pos_y_ctrl = PidController()
    pos_y_ctrl.set_gain(POS_Y_KP, POS_Y_KI, POS_Y_KD)
    pos_timer = ControllerTimer()

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        while MODE == 'POS' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 計時開始
            pos_timer.time_start()

            # 讀取回授值
            pos_actual_x = ACTUAL_POS[0]
            pos_actual_y = ACTUAL_POS[1]

            # 回授值截止區修正
            if TARGET_POS[0] - POS_DZ < pos_actual_x < TARGET_POS[0] + POS_DZ:
                pos_actual_x = TARGET_POS[0]
            if TARGET_POS[1] - POS_DZ < pos_actual_y < TARGET_POS[1] + POS_DZ:
                pos_actual_y = TARGET_POS[1]

            # 更新控制器
            pos_x_ctrl.reference_update(TARGET_POS[0])
            pos_y_ctrl.reference_update(TARGET_POS[1])
            pos_x_ctrl.feedback_update(pos_actual_x)
            pos_y_ctrl.feedback_update(pos_actual_y)

            # 執行控制器
            vel_x_out = pos_x_ctrl.ctrl_process()
            vel_y_out = pos_y_ctrl.ctrl_process()

            # 濾波處理
            vel_x_cmd = vel_x_maf.calculate(vel_x_out)
            vel_y_cmd = vel_y_maf.calculate(vel_y_out)

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

            # 記錄數值
            R_POS_BLOCK_X.append(ACTUAL_POS[0])
            R_POS_BLOCK_Y.append(ACTUAL_POS[1])
            R_POS_BLOCK_AREA.append(ACTUAL_AREA / R_POS_RECORD_SCALE)

            R_POS_X_ERROR.append(pos_x_ctrl.error)
            R_POS_Y_ERROR.append(pos_y_ctrl.error)
            R_POS_X_GP.append(pos_x_ctrl.gain_p)
            R_POS_X_GI.append(pos_x_ctrl.gain_i)
            R_POS_X_GD.append(pos_x_ctrl.gain_d)
            R_POS_Y_GP.append(pos_y_ctrl.gain_p)
            R_POS_Y_GI.append(pos_y_ctrl.gain_i)
            R_POS_Y_GD.append(pos_y_ctrl.gain_d)

            R_POS_X_TARGET_VEL.append(vel_x_cmd)
            R_POS_Y_TARGET_VEL.append(vel_y_cmd)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            pos_timer.wait_time(FREQ_POS)

            # 當飛行模式不為定位時，跳出迴圈並暫停函式
            if not MODE == 'POS':
                # 歸零速度命令數值
                TARGET_VEL = [0, 0]

                # 重設暫存器
                pos_x_ctrl.reset_controller()
                pos_y_ctrl.reset_controller()
                vel_x_maf.reset_register()
                vel_y_maf.reset_register()

                print('<飛行狀態> 位置控制器：暫停')

        # 計時開始
        pos_timer.time_start()

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
        pos_timer.wait_time(FREQ_POS)

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
    # 等待位置感測器啟動
    if not STATE_POS:
        print('<啟動程序> 速度控制器：等待位置控制器啟動')
    while not STATE_POS:
        time.sleep(0.1)

    # 回報速度控制器啟用
    STATE_VEL = True
    print('<啟動程序> 速度控制器：啟動')

    # 建立控制器物件
    vel_x_ctrl = PidController()
    vel_x_ctrl.set_gain(POS_VEL_X_KP, POS_VEL_X_KI, POS_VEL_X_KD)
    vel_y_ctrl = PidController()
    vel_y_ctrl.set_gain(POS_VEL_Y_KP, POS_VEL_Y_KI, POS_VEL_Y_KD)
    vel_timer = ControllerTimer()

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        while MODE == 'POS' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 計時開始
            vel_timer.time_start()

            # 誤差增益
            vel_actual_x = ACTUAL_VEL[0] * POS_VEL_X_FEEDBACK_GAIN
            vel_actual_y = ACTUAL_VEL[1] * POS_VEL_Y_FEEDBACK_GAIN

            # 更新控制器
            vel_x_ctrl.reference_update(TARGET_VEL[0])
            vel_y_ctrl.reference_update(TARGET_VEL[1])
            vel_x_ctrl.feedback_update(vel_actual_x)
            vel_y_ctrl.feedback_update(vel_actual_y)

            # 執行控制器
            roll_out = vel_x_ctrl.ctrl_process()
            pitch_out = vel_y_ctrl.ctrl_process()

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

            # 記錄數值
            R_POS_VEL_X_TAR.append(TARGET_VEL[0])
            R_POS_VEL_Y_TAR.append(TARGET_VEL[1])
            R_POS_VEL_X_ACT.append(vel_actual_x)
            R_POS_VEL_Y_ACT.append(vel_actual_y)

            R_POS_VEL_X_ERROR.append(vel_x_ctrl.error)
            R_POS_VEL_Y_ERROR.append(vel_y_ctrl.error)
            R_POS_VEL_X_GP.append(vel_x_ctrl.gain_p)
            R_POS_VEL_X_GI.append(vel_x_ctrl.gain_i)
            R_POS_VEL_X_GD.append(vel_x_ctrl.gain_d)
            R_POS_VEL_Y_GP.append(vel_y_ctrl.gain_p)
            R_POS_VEL_Y_GI.append(vel_y_ctrl.gain_i)
            R_POS_VEL_Y_GD.append(vel_y_ctrl.gain_d)

            R_POS_TARGET_ROLL.append(TARGET_ROLL)
            R_POS_TARGET_PITCH.append(TARGET_PITCH)
            R_POS_ACTUAL_ROLL.append(math.degrees(uav.attitude.roll))
            R_POS_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            vel_timer.wait_time(FREQ_VEL)

            # 當飛行模式不為定位時，跳出迴圈並暫停函式
            if not MODE == 'POS':
                # 歸零姿態數值
                TARGET_ROLL = 0
                TARGET_PITCH = 0

                # 重設控制器
                vel_x_ctrl.reset_controller()
                vel_y_ctrl.reset_controller()

                print('<飛行狀態> 速度控制器：暫停')

        # 計時開始
        vel_timer.time_start()

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
        vel_timer.wait_time(FREQ_VEL)

        # 當飛行模式為定位時，跳入迴圈並啟動函式
        if MODE == 'POS' and STATE_THREAD:
            print('<飛行狀態> 速度控制器：啟動')

        # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_VEL = False
    print('<關閉程序> 速度控制器：關閉')


# =========== 航向控制器 ===============================================
# 說明：
# 採用PID控制器型態，每一迴圈讀取一次影像計算線條型心，並將型心值做均值綠波
# 濾波完畢後再以該值計算出型心至畫面下方中心之夾角，並以此值為控制器的誤差值
# 以該值來做PID增益處理，輸出角速度來追尋角度，此控制器可處理平行、轉角、Ｔ路口轉向
# 為不可逆型循跡控制器，參數可調最大角速度輸出、切斷區間、濾波等級、面積門檻值
# 使用Global-Frame參照，以地面標線為參照座標，預測為PD控制器(前進速度控制分離)
def heading_controller():
    global TARGET_ROLL, TARGET_YAW_RATE, STATE_HDG
    # 等待相機啟動
    if not STATE_SENSOR_CAM:
        print('<啟動程序> 位置控制器：等待影像感測器啟動')
    while not STATE_SENSOR_CAM:
        time.sleep(0.1)

    # 回報定位控制器啟用
    STATE_HDG = True
    print('<啟動程序> 航向控制器：啟動')

    # 建立控制器物件
    hdg_ctrl = PidController()
    hdg_ctrl.set_gain(HDG_KP, HDG_KI, HDG_KD)
    hdg_timer = ControllerTimer()

    # 程式迴圈
    while STATE_THREAD:
        # 定位作動迴圈(同捆執行模式，於循跡模式作動)
        while MODE == 'LINE' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 計時開始
            hdg_timer.time_start()

            # 讀取回授值
            hdg_actual = ACTUAL_HDG

            # 回授值截止區修正
            if HDG_DZ > hdg_actual > - HDG_DZ:
                hdg_actual = 0

            # 更新控制器
            hdg_ctrl.reference_update(0)
            hdg_ctrl.feedback_update(hdg_actual)

            # 執行控制器
            gain_out = hdg_ctrl.ctrl_process()

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

            # 記錄數值
            R_HDG_ACTUAL.append(hdg_actual)
            R_HDG_TARGET_ROLL.append(TARGET_ROLL)
            R_HDG_TARGET_YAW_RATE.append(TARGET_YAW_RATE)

            R_HDG_ERROR.append(hdg_ctrl.error)
            R_HDG_GP.append(hdg_ctrl.gain_p)
            R_HDG_GI.append(hdg_ctrl.gain_i)
            R_HDG_GD.append(hdg_ctrl.gain_d)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            hdg_timer.wait_time(FREQ_HDG)

            # 當飛行模式不為循跡時，跳出迴圈並暫停函式
            if not MODE == 'LINE':
                # 歸零姿態數值
                TARGET_ROLL = 0
                TARGET_YAW_RATE = 0

                # 重設控制器
                hdg_ctrl.reset_controller()

                # 狀態設定
                STATE_HDG = False
                print('<飛行狀態> 航向控制器：暫停')

        # 無循跡作動區間
        # 計時開始
        hdg_timer.time_start()

        # 回報迴圈休眠狀態
        if MODE == 'POS' and STATE_HDG:
            STATE_HDG = False
            print('<飛行狀態> 航向控制器：暫停')

        # 記錄數值
        R_HDG_ACTUAL.append(0)
        R_HDG_TARGET_ROLL.append(0)
        R_HDG_TARGET_YAW_RATE.append(0)

        R_HDG_ERROR.append(0)
        R_HDG_GP.append(0)
        R_HDG_GI.append(0)
        R_HDG_GD.append(0)

        # 迴圈執行間隔
        hdg_timer.wait_time(FREQ_HDG)

        # 當飛行模式為循跡時，跳入迴圈並啟動函式
        if MODE == 'LINE' and STATE_THREAD:
            STATE_HDG = True
            print('<飛行狀態> 航向控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_HDG = False
    print('<關閉程序> 航向控制器：關閉')


# =========== 飛行速度控制器 ============================================
# 說明：
# 於循線期間控制往前飛行的速度
# 不一定會使用(視感測器響應情形而定)
def flight_speed_controller():
    global TARGET_PITCH, STATE_SPD
    # 等待飛行速度感測器啟動
    if not STATE_SENSOR_SPD:
        print('<啟動程序> 飛行速度控制器：等待飛行速度感測器啟動')
    while not STATE_SENSOR_SPD:
        time.sleep(0.1)

    # 回報飛行速度控制器啟用
    STATE_SPD = True
    print('<啟動程序> 飛行速度控制器：啟動')

    # 建立控制器物件
    spd_ctrl = PidController()
    spd_ctrl.set_gain(SPD_KP, SPD_KI, SPD_KD)
    spd_timer = ControllerTimer()

    # 程式迴圈(同捆執行模式)
    while STATE_THREAD:
        while MODE == 'LINE' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 計時開始
            spd_timer.time_start()

            # 讀取目標值
            spd_target = TARGET_SPD

            # 目標輸入限制
            if spd_target > SPD_LIMIT:
                spd_target = SPD_LIMIT
            if spd_target < -SPD_LIMIT:
                spd_target = -SPD_LIMIT

            # 更新控制器
            spd_ctrl.reference_update(spd_target)
            spd_ctrl.feedback_update(ACTUAL_SPD)

            # 執行控制器
            pitch_out = spd_ctrl.ctrl_process()

            # 角度限制(包含截止區設定)
            if pitch_out > SPD_ANGLE_LIMIT:
                pitch_out = SPD_ANGLE_LIMIT
            elif pitch_out < -SPD_ANGLE_LIMIT:
                pitch_out = -SPD_ANGLE_LIMIT
            elif -SPD_ANGLE_CUTOFF < pitch_out < SPD_ANGLE_CUTOFF:
                pitch_out = 0

            # 輸出至全域變數
            TARGET_PITCH = -pitch_out

            # 記錄數值
            R_SPD_TARGET.append(spd_target)
            R_SPD_ACTUAL.append(ACTUAL_SPD)
            R_SPD_TARGET_PITCH.append(TARGET_PITCH)
            R_SPD_ACTUAL_PITCH.append(math.degrees(uav.attitude.pitch))

            R_SPD_ERROR.append(spd_ctrl.error)
            R_SPD_GP.append(spd_ctrl.gain_p)
            R_SPD_GI.append(spd_ctrl.gain_i)
            R_SPD_GD.append(spd_ctrl.gain_d)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            spd_timer.wait_time(FREQ_SPD)

            # 當飛行模式不為循跡時，跳出迴圈並暫停函式
            if not MODE == 'LINE':
                # 切換跳出姿態數值(慣性抵銷用)
                TARGET_PITCH = SPD_OFF_ANGLE

                # 重設控制器
                spd_ctrl.reset_controller()

                # 更新狀態
                STATE_SPD = False
                print('<飛行狀態> 飛行速度控制器：暫停')

        # 無循跡作動區間
        # 計時開始
        spd_timer.time_start()

        # 回報迴圈休眠狀態
        if MODE == 'POS' and STATE_SPD:
            STATE_SPD = False
            print('<飛行狀態> 飛行速度控制器：暫停')

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
        spd_timer.wait_time(FREQ_SPD)

        # 當飛行模式為循跡時，跳入迴圈並啟動函式
        if MODE == 'LINE' and STATE_THREAD:
            print('<飛行狀態> 飛行速度控制器：啟動')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_SPD = False
    print('<關閉程序> 飛行速度控制器：關閉')