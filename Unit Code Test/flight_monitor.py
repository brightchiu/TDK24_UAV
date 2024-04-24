# 飛控監看介面程式
import tkinter as tk
import threading
import time

# ========== 旗標與變數 ================================================
# 旗標
STATE_SYSTEM = True             # 系統旗標(關閉線程用)
STATE_ALT = False               # 高度控制器旗標
STATE_POS = False               # 位置控制器旗標
STATE_HDG = False               # 航向控制器旗標
STATE_CMD = False               # 命令傳輸程式旗標
STATE_CAM = False               # 影像擷取程式旗標
STATE_SENSOR_SONAR = False             # 超音波高度計旗標
STATE_SENSOR = False            # 高度感測器旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)
POS_COLOR = 'red'                      # 定位的色塊顏色
MISSION_COLOR = 'blue'              # 任務顏色(blue or green)

# 多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# 目標值(命令值)
TARGET_ALT = 0                  # 目標高度，主程式輸入於高度控制器(m)
TARGET_ROLL = 0                 # 目標滾轉角(degree)
TARGET_PITCH = 0                # 目標俯仰角(degree)
TARGET_YAW = 0                  # 目標偏航角(degree)，不使用
TARGET_YAW_OFFSET = 0           # 目標偏航角補正值(degree)
TARGET_THRUST = 0               # 目標推力，表示升降速度(0.5為中點)
TARGET_ROLL_RATE = 0            # 滾轉角速度(degree/s)，不使用
TARGET_PITCH_RATE = 0           # 俯仰角速度(degree/s)，不使用
TARGET_YAW_RATE = 0             # 偏航角速度(degree/s)，需搭配控制器控制角度

# 實際值(量測值)
ACTUAL_ROLL = 65                  # 實際高度
ACTUAL_PITCH = 34                 # 實際高度
ACTUAL_YAW = 140                  # 實際高度
ACTUAL_HDG = 245
ACTUAL_ALT = 0                  # 實際高度
ACTUAL_ALT_SONAR = 0            # 超音波高度計
ACTUAL_ALT_BAROM = 0            # 氣壓高度計
ACTUAL_POS = [100, 200]         # 色塊實際位置
ACTUAL_HDG_ERR = 0              # 中線夾角


def info():
    state_system.set('[ON]')
    time.sleep(5)
    global STATE_SYSTEM

    STATE_SYSTEM = False
    sys_state.configure(fg='red', text=ACTUAL_HDG)
    pitch.configure(text=ACTUAL_PITCH)
    print('do')


def activate_thread():
    print('thread do')


# 狀態更新程式(
def info_update():
    pass


# 圖形化介面建構
flight_monitor = tk.Tk()
flight_monitor.title('TDK 24th 飛行組 C08 台灣大艦隊 飛行控制介面')
flight_monitor.geometry('1400x800')
flight_monitor.configure(background='white')

state_system = tk.StringVar()

# 第一行
line_1_frame = tk.Frame(flight_monitor, height=270, width=1250)
line_1_frame.pack_propagate(0)
line_1_frame.pack(side=tk.TOP)
line_1_label = tk.Label(line_1_frame, font=('', 25), fg='blue',
                        text='\n----- TDK 24th 飛行組 C08 台灣大艦隊 飛行控制介面 -----\n')
line_1_label.pack(side=tk.TOP)

# 即時姿態
att_frame = tk.Frame(line_1_frame, height=200, width=600)
att_frame.pack(side=tk.LEFT)
att_label = tk.Label(att_frame, font=('', 20), text='------------ 即時姿態 ------------')
att_label.pack(side=tk.TOP)

# 滾轉
roll_frame = tk.Frame(att_frame, height=130, width=80)
roll_frame.pack_propagate(0)
roll_frame.pack(side=tk.LEFT)
roll_label = tk.Label(roll_frame, font=('', 25), text='Roll')
roll_label.pack(side=tk.TOP)
roll = tk.Label(roll_frame, font=('', 35), text=ACTUAL_ROLL)
roll.pack(side=tk.TOP)
roll_unit = tk.Label(roll_frame, font=('', 15), text='Degree')
roll_unit.pack(side=tk.TOP)

# 俯仰
pitch_frame = tk.Frame(att_frame, height=130, width=80)
pitch_frame.pack_propagate(0)
pitch_frame.pack(side=tk.LEFT)
pitch_label = tk.Label(pitch_frame, font=('', 25), text='Pitch')
pitch_label.pack(side=tk.TOP)
pitch = tk.Label(pitch_frame, font=('', 35))
pitch.pack(side=tk.TOP)
pitch_unit = tk.Label(pitch_frame, font=('', 15), text='Degree')
pitch_unit.pack(side=tk.TOP)

# 偏航
yaw_frame = tk.Frame(att_frame, height=130, width=80)
yaw_frame.pack_propagate(0)
yaw_frame.pack(side=tk.LEFT)
yaw_label = tk.Label(yaw_frame, font=('', 25), text='Yaw')
yaw_label.pack(side=tk.TOP)
yaw = tk.Label(yaw_frame, font=('', 35), text=ACTUAL_YAW)
yaw.pack(side=tk.TOP)
yaw_unit = tk.Label(yaw_frame, font=('', 15), text='Degree')
yaw_unit.pack(side=tk.TOP)

# 高度
alt_frame = tk.Frame(att_frame, height=130, width=80)
alt_frame.pack_propagate(0)
alt_frame.pack(side=tk.LEFT)
alt_label = tk.Label(alt_frame, font=('', 25), text='ALT')
alt_label.pack(side=tk.TOP)
alt = tk.Label(alt_frame, font=('', 35), text=ACTUAL_ALT)
alt.pack(side=tk.TOP)
alt_unit = tk.Label(alt_frame, font=('', 15), text='Meter')
alt_unit.pack(side=tk.TOP)

# 航向
hdg_frame = tk.Frame(att_frame, height=130, width=80)
hdg_frame.pack_propagate(0)
hdg_frame.pack(side=tk.LEFT)
hdg_label = tk.Label(hdg_frame, font=('', 25), text='HDG')
hdg_label.pack(side=tk.TOP)
hdg = tk.Label(hdg_frame, font=('', 35), text=ACTUAL_HDG)
hdg.pack(side=tk.TOP)
hdg_unit = tk.Label(hdg_frame, font=('', 15), text='Degree')
hdg_unit.pack(side=tk.TOP)


# =====控制器狀態=====
ctrl_frame = tk.Frame(line_1_frame, height=200, width=800)

ctrl_frame.pack(side=tk.RIGHT)
ctrl_label = tk.Label(ctrl_frame, font=('', 20),
                      text='---------------------------------- 控制器狀態 ----------------------------------')
ctrl_label.pack_propagate(0)
ctrl_label.pack(side=tk.TOP)

# 系統控制器
sys_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
sys_state_frame.pack_propagate(0)
sys_state_frame.pack(side=tk.LEFT)
sys_state_label = tk.Label(sys_state_frame, font=('', 25), text='SYS\n系統')
sys_state_label.pack(side=tk.TOP)
sys_state = tk.Label(sys_state_frame, fg='green', font=('', 35), text='[ON]')
sys_state.pack(side=tk.TOP)

# 高度控制器
alt_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
alt_state_frame.pack_propagate(0)
alt_state_frame.pack(side=tk.LEFT)
alt_state_label = tk.Label(alt_state_frame, font=('', 25), text='ALT\n高度')
alt_state_label.pack(side=tk.TOP)
alt_state = tk.Label(alt_state_frame, font=('', 35), text='[ON]')
alt_state.pack(side=tk.TOP)

# 位置控制器
pos_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
pos_state_frame.pack_propagate(0)
pos_state_frame.pack(side=tk.LEFT)
pos_state_label = tk.Label(pos_state_frame, font=('', 25), text='POS\n位置')
pos_state_label.pack(side=tk.TOP)
pos_state = tk.Label(pos_state_frame, font=('', 35), text='[ON]')
pos_state.pack(side=tk.TOP)

# 航向控制器
hdg_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
hdg_state_frame.pack_propagate(0)
hdg_state_frame.pack(side=tk.LEFT)
hdg_state_label = tk.Label(hdg_state_frame, font=('', 25), text='HDG\n航向')
hdg_state_label.pack(side=tk.TOP)
hdg_state = tk.Label(hdg_state_frame, font=('', 35), text='[OFF]')
hdg_state.pack(side=tk.TOP)

# 命令控制器
cmd_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
cmd_state_frame.pack_propagate(0)
cmd_state_frame.pack(side=tk.LEFT)
cmd_state_label = tk.Label(cmd_state_frame, font=('', 25), text='CMD\n命令')
cmd_state_label.pack(side=tk.TOP)
cmd_state = tk.Label(cmd_state_frame, font=('', 35), text='[ON]')
cmd_state.pack(side=tk.TOP)

# 相機控制器
cam_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
cam_state_frame.pack_propagate(0)
cam_state_frame.pack(side=tk.LEFT)
cam_state_label = tk.Label(cam_state_frame, font=('', 25), text='CAM\n相機')
cam_state_label.pack(side=tk.TOP)
cam_state = tk.Label(cam_state_frame, font=('', 35), text='[OFF]')
cam_state.pack(side=tk.TOP)

# 高度感測器
alsen_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
alsen_state_frame.pack_propagate(0)
alsen_state_frame.pack(side=tk.LEFT)
alsen_state_label = tk.Label(alsen_state_frame, font=('', 25), text='ALSEN\n高感')
alsen_state_label.pack(side=tk.TOP)
alsen_state = tk.Label(alsen_state_frame, font=('', 35), text='[OFF]')
alsen_state.pack(side=tk.TOP)

# 超音波感測器
sonar_state_frame = tk.Frame(ctrl_frame, height=130, width=100)
sonar_state_frame.pack_propagate(0)
sonar_state_frame.pack(side=tk.LEFT)
sonar_state_label = tk.Label(sonar_state_frame, font=('', 25), text='SONAR\n超音波')
sonar_state_label.pack(side=tk.TOP)
sonar_state = tk.Label(sonar_state_frame, font=('', 35), text='[OFF]')
sonar_state.pack(side=tk.TOP)

# ===================================第二行=================================
line_2_frame = tk.Frame(flight_monitor, height=200, width=1100)
line_2_frame.pack(side=tk.TOP)

# =====高度控制器=====
alt_ctrl_frame = tk.Frame(line_2_frame)
alt_ctrl_frame.pack(side=tk.LEFT)
alt_ctrl_label = tk.Label(alt_ctrl_frame, font=('', 20), text='----- 高度控制器 -----')
alt_ctrl_label.pack(side=tk.TOP)

# 目標高
alt_target_frame = tk.Frame(alt_ctrl_frame, height=130, width=100)
alt_target_frame.pack_propagate(0)
alt_target_frame.pack(side=tk.LEFT)
alt_target_label = tk.Label(alt_target_frame, font=('', 20), text='目標高度')
alt_target_label.pack(side=tk.TOP)
alt_target = tk.Label(alt_target_frame, font=('', 35), text=TARGET_ALT)
alt_target.pack(side=tk.TOP)
alt_target_unit = tk.Label(alt_target_frame, font=('', 15), text='Meter')
alt_target_unit.pack(side=tk.TOP)

# 實際高度
alt_actual_frame = tk.Frame(alt_ctrl_frame, height=130, width=100)
alt_actual_frame.pack_propagate(0)
alt_actual_frame.pack(side=tk.LEFT)
alt_actual_label = tk.Label(alt_actual_frame, font=('', 20), text='實際高度')
alt_actual_label.pack(side=tk.TOP)
alt_actual = tk.Label(alt_actual_frame, font=('', 35), text=ACTUAL_ALT)
alt_actual.pack(side=tk.TOP)
alt_actual_unit = tk.Label(alt_actual_frame, font=('', 15), text='Meter')
alt_actual_unit.pack(side=tk.TOP)

# 輸出推力
alt_thrust_frame = tk.Frame(alt_ctrl_frame, height=130, width=100)
alt_thrust_frame.pack_propagate(0)
alt_thrust_frame.pack(side=tk.LEFT)
alt_thrust_label = tk.Label(alt_thrust_frame, font=('', 20), text='爬升輸出')
alt_thrust_label.pack(side=tk.TOP)
alt_thrust = tk.Label(alt_thrust_frame, font=('', 35), text=TARGET_THRUST)
alt_thrust.pack(side=tk.TOP)
alt_thrust_unit = tk.Label(alt_thrust_frame, font=('', 15), text='%')
alt_thrust_unit.pack(side=tk.TOP)

# =====位置控制器=====
pos_ctrl_frame = tk.Frame(line_2_frame)
pos_ctrl_frame.pack(side=tk.LEFT)
pos_ctrl_label = tk.Label(pos_ctrl_frame, font=('', 20),
                          text='-------------------- 位置控制器 --------------------')
pos_ctrl_label.pack(side=tk.TOP)

# 任務顏色
pos_mis_color_frame = tk.Frame(pos_ctrl_frame, height=130, width=100)
pos_mis_color_frame.pack_propagate(0)
pos_mis_color_frame.pack(side=tk.LEFT)
pos_mis_color_label = tk.Label(pos_mis_color_frame, font=('', 20), text='任務顏色')
pos_mis_color_label.pack(side=tk.TOP)
pos_mis_color= tk.Label(pos_mis_color_frame, font=('', 35), text=MISSION_COLOR)
pos_mis_color.pack(side=tk.TOP)

# 定位顏色
pos_color_frame = tk.Frame(pos_ctrl_frame, height=130, width=100)
pos_color_frame.pack_propagate(0)
pos_color_frame.pack(side=tk.LEFT)
pos_color_label = tk.Label(pos_color_frame, font=('', 20), text='定位顏色')
pos_color_label.pack(side=tk.TOP)
pos_color= tk.Label(pos_color_frame, font=('', 35), text=POS_COLOR)
pos_color.pack(side=tk.TOP)

# 型心位置
pos_actual_frame = tk.Frame(pos_ctrl_frame, height=130, width=150)
pos_actual_frame.pack_propagate(0)
pos_actual_frame.pack(side=tk.LEFT)
pos_actual_label = tk.Label(pos_actual_frame, font=('', 20), text='型心位置')
pos_actual_label.pack(side=tk.TOP)
pos_actual = tk.Label(pos_actual_frame, font=('', 35), text=ACTUAL_POS)
pos_actual.pack(side=tk.TOP)
pos_actual_unit = tk.Label(pos_actual_frame, font=('', 15), text='( X , Y )')
pos_actual_unit.pack(side=tk.TOP)

# 滾轉
pos_roll_frame = tk.Frame(pos_ctrl_frame, height=130, width=100)
pos_roll_frame.pack_propagate(0)
pos_roll_frame.pack(side=tk.LEFT)
pos_roll_label = tk.Label(pos_roll_frame, font=('', 20), text='Tar-Roll')
pos_roll_label.pack(side=tk.TOP)
pos_roll = tk.Label(pos_roll_frame, font=('', 35), text=TARGET_ROLL)
pos_roll.pack(side=tk.TOP)
pos_roll_unit = tk.Label(pos_roll_frame, font=('', 15), text='Degree')
pos_roll_unit.pack(side=tk.TOP)

# 俯仰
pos_pitch_frame = tk.Frame(pos_ctrl_frame, height=130, width=100)
pos_pitch_frame.pack_propagate(0)
pos_pitch_frame.pack(side=tk.LEFT)
pos_pitch_label = tk.Label(pos_pitch_frame, font=('', 20), text='Tar-Pitch')
pos_pitch_label.pack(side=tk.TOP)
pos_pitch = tk.Label(pos_pitch_frame, font=('', 35), text=TARGET_PITCH)
pos_pitch.pack(side=tk.TOP)
pos_pitch_unit = tk.Label(pos_pitch_frame, font=('', 15), text='Degree')
pos_pitch_unit.pack(side=tk.TOP)

# =====航向控制器=====
hdg_ctrl_frame = tk.Frame(line_2_frame)
hdg_ctrl_frame.pack(side=tk.RIGHT)
hdg_ctrl_label = tk.Label(hdg_ctrl_frame, font=('', 20), text='----- 航向控制器 -----')
hdg_ctrl_label.pack(side=tk.TOP)

# 角度差
hdg_error_frame = tk.Frame(hdg_ctrl_frame, height=130, width=100)
hdg_error_frame.pack_propagate(0)
hdg_error_frame.pack(side=tk.LEFT)
hdg_error_label = tk.Label(hdg_error_frame, font=('', 20), text='角度差')
hdg_error_label.pack(side=tk.TOP)
hdg_error = tk.Label(hdg_error_frame, font=('', 35), text=ACTUAL_HDG_ERR)
hdg_error.pack(side=tk.TOP)
hdg_error_unit = tk.Label(hdg_error_frame, font=('', 15), text='Degree')
hdg_error_unit.pack(side=tk.TOP)

# 角速度
hdg_rate_frame = tk.Frame(hdg_ctrl_frame, height=130, width=100)
hdg_rate_frame.pack_propagate(0)
hdg_rate_frame.pack(side=tk.LEFT)
hdg_rate_label = tk.Label(hdg_rate_frame, font=('', 20), text='Tar-Y Rate')
hdg_rate_label.pack(side=tk.TOP)
hdg_rate = tk.Label(hdg_rate_frame, font=('', 35), text=TARGET_YAW_RATE)
hdg_rate.pack(side=tk.TOP)
hdg_rate_unit = tk.Label(hdg_rate_frame, font=('', 15), text='Degree/s')
hdg_rate_unit.pack(side=tk.TOP)

# 滾轉角
hdg_roll_frame = tk.Frame(hdg_ctrl_frame, height=130, width=100)
hdg_roll_frame.pack_propagate(0)
hdg_roll_frame.pack(side=tk.LEFT)
hdg_roll_label = tk.Label(hdg_roll_frame, font=('', 20), text='Tar-Roll')
hdg_roll_label.pack(side=tk.TOP)
hdg_roll = tk.Label(hdg_roll_frame, font=('', 35), text=TARGET_ROLL)
hdg_roll.pack(side=tk.TOP)
hdg_roll_unit = tk.Label(hdg_roll_frame, font=('', 15), text='Degree')
hdg_roll_unit.pack(side=tk.TOP)


# ===================================第三行=================================
line_3_frame = tk.Frame(flight_monitor)
line_3_frame.pack(side=tk.TOP)

# 高度感測器
alsen_frame = tk.Frame(line_3_frame)
alsen_frame.pack(side=tk.LEFT)
alsen_label = tk.Label(alsen_frame, font=('', 20), text='----- 高度感測器 -----')
alsen_label.pack(side=tk.TOP)

# 加權高度
alsen_weight_frame = tk.Frame(alsen_frame, height=130, width=100)
alsen_weight_frame.pack_propagate(0)
alsen_weight_frame.pack(side=tk.LEFT)
alsen_weight_label = tk.Label(alsen_weight_frame, font=('', 20), text='加權高度')
alsen_weight_label.pack(side=tk.TOP)
alsen_weight = tk.Label(alsen_weight_frame, font=('', 35), text=ACTUAL_ALT)
alsen_weight.pack(side=tk.TOP)
alsen_weight_unit = tk.Label(alsen_weight_frame, font=('', 15), text='Meter')
alsen_weight_unit.pack(side=tk.TOP)

# 超音波高度
alsen_sonar_frame = tk.Frame(alsen_frame, height=130, width=100)
alsen_sonar_frame.pack_propagate(0)
alsen_sonar_frame.pack(side=tk.LEFT)
alsen_sonar_label = tk.Label(alsen_sonar_frame, font=('', 20), text='超音波計')
alsen_sonar_label.pack(side=tk.TOP)
alsen_sonar = tk.Label(alsen_sonar_frame, font=('', 35), text=ACTUAL_ALT_SONAR)
alsen_sonar.pack(side=tk.TOP)
alsen_sonar_unit = tk.Label(alsen_sonar_frame, font=('', 15), text='Meter')
alsen_sonar_unit.pack(side=tk.TOP)

# 氣壓計高度
alsen_barom_frame = tk.Frame(alsen_frame, height=130, width=100)
alsen_barom_frame.pack_propagate(0)
alsen_barom_frame.pack(side=tk.LEFT)
alsen_barom_label = tk.Label(alsen_barom_frame, font=('', 20), text='氣壓計')
alsen_barom_label.pack(side=tk.TOP)
alsen_barom = tk.Label(alsen_barom_frame, font=('', 35), text=ACTUAL_ALT_BAROM)
alsen_barom.pack(side=tk.TOP)
alsen_barom_unit = tk.Label(alsen_barom_frame, font=('', 15), text='Meter')
alsen_barom_unit.pack(side=tk.TOP)

# 事件通知
info_frame = tk.Frame(line_3_frame)
info_frame.pack(side=tk.LEFT)
info_label = tk.Label(info_frame, font=('', 20), text='----- 系統狀態提示 -----')
info_label.pack(side=tk.TOP)

# 提示窗
info_window = tk.Text(info_frame, font=('', 15), width=75, height='6')
info_window_scroll = tk.Scrollbar(info_window, width=10)
info_window_scroll.pack(side=tk.RIGHT, fill=tk.Y)
info_window.pack_propagate(0)
info_window.pack(side=tk.TOP)
info_window_scroll.config(command=info_window.yview)
info_window.config(yscrollcommand=info_window_scroll.set)

# ===================================第四行=================================
line_4_frame = tk.Frame(flight_monitor)
line_4_frame.pack(side=tk.TOP)

# 多線程啟動
start_frame = tk.Frame(line_4_frame)
start_frame.pack(side=tk.LEFT)
thread_button_label = tk.Label(start_frame, font=('', 17), text='<多線程程序>\n')
thread_button_label.pack(side=tk.TOP)
thread_button = tk.Button(start_frame, relief='raised', font=('', 13),
                          text='啟 動', width=5, height=2, command=activate_thread)
thread_button.pack(side=tk.TOP)

ret = threading.Thread(target=info)
ret.start()

flight_monitor.after(1000)
flight_monitor.mainloop()
