# 影像偵測(包含定位的位置與速度、循跡的角度)
# 需針對OpenCV版本調整程式碼
# 添加型心與箭頭標記(待驗證)
# 設定出界保留、增加目標位置設定

# 載入模組
import numpy as np
import threading
import time
import math
import cv2

# ========== 旗標與變數 ================================================
# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_CAM = False               # 影像擷取程式旗標
STATE_SENSOR_CAM = False        # 影像感測器旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)
MISSION_COLOR = ''              # 任務顏色(blue or green)
POS_COLOR = ''                  # 定位的色塊顏色

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->實際值(量測值)
ACTUAL_POS = [0, 0]             # 色塊實際位置(px)
ACTUAL_VEL = [0, 0]             # 色塊實際速度(px/s)
ACTUAL_HDG = 0                  # 與線夾角(degree，型心法，循跡與平行線防止用)
ACTUAL_ANG_LINE = 0             # 線角度(degree，Y軸為基準，最大轉角限制用)
ACTUAL_AREA = 0                 # 色塊面積(px^2)

# ========== 控制器參數設定 =============================================
# ->迴圈頻率(預設20Hz)
FREQ_SENSOR_CAM = 20            # 影像感測器頻率(不快於影像禎率)

# ========== 影像參數設定 ===============================================
# ->相機設定參數
CAM_IMAGE = 0                               # 儲存影像
CAM_IMAGE_LIVE = 0                          # 即時影像
CAM_IMAGE_WIDTH = 480                       # 影像寬度(px，4:3設定 )
CAM_IMAGE_HEIGHT = 360                      # 影像高度(px)

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


# ========== 影像感測器 ===============================================
# 負責計算影像的位置、速度、角度值，結合位置與循線模式
def camera_sensor():
    global STATE_SENSOR_CAM, ACTUAL_POS, ACTUAL_VEL, ACTUAL_AREA, ACTUAL_HDG, ACTUAL_ANG_LINE
    # 均值濾波器設定
    pos_x_register = []
    pos_y_register = []
    vel_x_register = []
    vel_y_register = []
    hdg_register = []

    # 速度值計算初始設定
    pos_x_old = 0
    pos_y_old = 0
    vel_x = 0
    vel_y = 0
    previous_time = time.time()

    # 等待影像擷取程式啟動
    if not STATE_CAM:
        print('<啟動程序> 影像感測器：等待影像擷取程式啟動')
    while not STATE_CAM:
        time.sleep(0.1)

    # 回報影像感測器啟用
    STATE_SENSOR_CAM = True
    print('<啟動程序> 影像感測器：啟動')

    # 程式迴圈
    while STATE_THREAD:
        # 定位模式計算迴圈(同捆執行模式)
        while MODE == 'POS' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 取得色塊XY座標
            x_row, y_row, block_area = image_moment(POS_COLOR)

            # 位置出界狀態值保留
            if x_row == 0 and math.fabs(pos_x_old) > CAM_IMAGE_WIDTH*0.25:
                x_row = pos_x_old
                outbound_x = True
            else:
                outbound_x = False

            if y_row == 0 and math.fabs(pos_y_old) > CAM_IMAGE_HEIGHT*0.25:
                y_row = pos_y_old
                outbound_y = True
            else:
                outbound_y = False

            # X座標均值濾波器
            pos_x_register.insert(0, x_row)
            if len(pos_x_register) > IMG_SAMPLE_NUM_AVE:
                pos_x_register.pop()
            pos_x = np.average(pos_x_register)

            # Y座標均值濾波器
            pos_y_register.insert(0, y_row)
            if len(pos_y_register) > IMG_SAMPLE_NUM_AVE:
                pos_y_register.pop()
            pos_y = np.average(pos_y_register)

            # 計算型心移動速度
            current_time = time.time()
            delta_time = current_time - previous_time

            pos_delta_x = pos_x - pos_x_old
            pos_delta_y = pos_y - pos_y_old
            vel_x_raw = pos_delta_x / delta_time
            vel_y_raw = pos_delta_y / delta_time

            # 速度出界狀態值保留
            if outbound_x:
                vel_x_raw = vel_x

            if outbound_y:
                vel_y_raw = vel_y

            # X速度均值濾波器
            vel_x_register.insert(0, vel_x_raw)
            if len(vel_x_register) > IMG_SAMPLE_NUM_AVE:
                vel_x_register.pop()
            vel_x = np.average(vel_x_register)

            # Y速度均值濾波器
            vel_y_register.insert(0, vel_y_raw)
            if len(vel_y_register) > IMG_SAMPLE_NUM_AVE:
                vel_y_register.pop()
            vel_y = np.average(vel_y_register)

            # 輸出數值至全域變數
            ACTUAL_POS = [pos_x, pos_y]
            ACTUAL_VEL = [vel_x, vel_y]
            ACTUAL_AREA = block_area

            # 傳遞值給下次迴圈使用
            pos_x_old = pos_x
            pos_y_old = pos_y
            previous_time = current_time

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_SENSOR_CAM)

            # 當飛行模式不為定位時，跳出迴圈並暫停函式
            if MODE == 'LINE':
                # 歸零數值
                ACTUAL_POS = [0, 0]
                ACTUAL_VEL = [0, 0]
                ACTUAL_AREA = 0
                print('<飛行狀態> 影像感測器：切換至角度輸出')

        # 循跡模式計算迴圈(同捆執行模式)
        while MODE == 'LINE' and STATE_THREAD:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 取得線條型心位置
            pos_x_raw, pos_y_raw, line_angle = line_moment()

            ACTUAL_ANG_LINE = line_angle

            # 型心位置均值濾波器
            # X座標均值濾波器
            pos_x_register.insert(0, pos_x_raw)
            if len(pos_x_register) > ANG_SAMPLE_NUM_AVE:
                pos_x_register.pop()
            pos_x = np.average(pos_x_register)

            # Y座標均值濾波器
            pos_y_register.insert(0, pos_y_raw)
            if len(pos_y_register) > ANG_SAMPLE_NUM_AVE:
                pos_y_register.pop()
            pos_y = np.average(pos_y_register)

            # 取得現在角度值(中心為目標值，朝右正，朝左負，使用global Frame參照)
            hdg_raw = angle_calculate(pos_x, pos_y)

            # 誤差角均值濾波器
            hdg_register.insert(0, hdg_raw)
            if len(hdg_register) > ANG_SAMPLE_NUM_AVE:
                hdg_register.pop()
            hdg = np.average(hdg_register)

            # 輸出角度誤差
            ACTUAL_HDG = hdg

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 迴圈執行間隔
            time.sleep(1 / FREQ_SENSOR_CAM)

            # 當飛行模式不為循跡時，跳出迴圈並暫停函式
            if MODE == 'POS':
                # 歸零數值
                ACTUAL_HDG = 0
                print('<飛行狀態> 影像感測器：切換至位置-速度輸出')

    # 當系統狀態為否時，跳出迴圈並關閉函式
    STATE_SENSOR_CAM = False
    print('<關閉程序> 影像感測器：關閉')


# ========== 色塊型心計算程序 ==========================================
# 為計算當禎影像中，目標顏色的型心位置、色塊面積大小，主要供位置控制器、飛行條件判斷使用
# 為防止雜訊可設置最低面積值，因此若色塊大小在此之下將無法被偵測到
# 處理流程為將影像先依目標顏色做過濾，再依此做型心、面積運算
def image_moment(color):
    global CAM_IMAGE_LIVE
    # 色域轉換BGR->HSV
    image_hsv = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2HSV)
    # 閉運算
    image_close = cv2.morphologyEx(image_hsv, cv2.MORPH_CLOSE,
                                   np.ones(IMG_MORPHOLOGY_EX_KERNEL, np.uint8))

    # 依照目標色抓該顏色輪廓
    if color == 'red':          # 紅色
        image_mask = cv2.inRange(image_close, IMG_RED_L, IMG_RED_H)
    elif color == 'green':      # 綠色
        image_mask = cv2.inRange(image_close, IMG_GREEN_L, IMG_GREEN_H)
    elif color == 'blue':       # 藍色
        image_mask = cv2.inRange(image_close, IMG_BLUE_L, IMG_BLUE_H)
    else:  # 黑色(預設顏色)
        image_mask = cv2.inRange(image_close, IMG_BLACK_L, IMG_BLACK_H)

    # 取得影像、輪廓點與索引(OpenCV Ver.3)
    image_c, contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)
    # 取得輪廓點與索引(OpenCV Ver.2&4)
    # contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 設定型心暫存列表
    moment_x_register = []
    moment_y_register = []
    area_register = []

    # 計算型心與面積
    for c in contours:
        # 計算色塊面積
        block_area = cv2.contourArea(c)

        # 面積符合使用門檻方取用(降噪處理)
        if block_area >= IMG_AREA_LIMIT:
            # 取得型心位置(左上為原點，加法增量)
            moment = cv2.moments(c)
            if moment["m00"] != 0:
                moment_x_raw = int(moment["m10"] / moment["m00"])
                moment_y_raw = int(moment["m01"] / moment["m00"])
            else:
                moment_x_raw = 0
                moment_y_raw = 0

            # 計算色塊位置(相對於畫面中心)
            block_x_offset = moment_x_raw - BODY_REF_X - BODY_OFFSET_X
            block_y_offset = BODY_REF_Y - moment_y_raw - BODY_OFFSET_Y

            moment_x_register.append(block_x_offset)
            moment_y_register.append(block_y_offset)
            area_register.append(block_area)

    # 設定型心運算暫存列表
    moment_area_x = 0
    moment_area_y = 0
    area_sum = 0

    # 計算面積一次矩
    for i in range(len(moment_x_register)):
        moment_area_x += moment_x_register[i] * area_register[i]
        moment_area_y += moment_y_register[i] * area_register[i]
        area_sum += area_register[i]

    # 求得組合型心位置
    if len(moment_x_register) != 0:
        block_x = moment_area_x / area_sum
        block_y = moment_area_y / area_sum
    else:
        block_x = 0
        block_y = 0

    # 標記型心位置(須轉換回原座標)
    cv2.circle(image_mask,
               (block_x + BODY_REF_X + BODY_OFFSET_X, block_y - BODY_REF_Y + BODY_OFFSET_Y),
               10, (0, 0, 0), -1)

    # 輸出影像(GUI顯示用)
    if MODE == 'POS':
        CAM_IMAGE_LIVE = image_mask

    return block_x, block_y, area_sum


# ========== 線條型心計算 =============================================
# 將影像做二值化後再做型心與面積計算，除影響去用過程與畫面中心設定點不同外，其餘與前函式相同
# 但因中間有程式碼不同，為避免增加樹狀複雜度，故不再做副程式呼叫
# 本函式供航向控制器使用，僅回傳型心位置值，面積閥值亦獨立設定
def line_moment():
    global CAM_IMAGE_LIVE
    # 色域轉換BGR->GRAY
    img_gray = cv2.cvtColor(CAM_IMAGE, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    img_blur = cv2.GaussianBlur(img_gray, ANG_GB_KERNEL_SIZE, ANG_GB_SIGMA)
    # 二值化
    ret, img_th = cv2.threshold(img_blur, ANG_THRESHOLD,
                                ANG_THRESHOLD_NEW, cv2.THRESH_BINARY_INV)

    # 取得影像、輪廓點與索引(OpenCV Ver.3)
    image_c, contours, hierarchy = cv2.findContours(img_th, cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)

    # 取得輪廓點與索引(OpenCV Ver.2&4)
    # contours, hierarchy = cv2.findContours(img_th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 設定型心暫存列表
    moment_x_register = []
    moment_y_register = []
    area_register = []
    angle_line = 0

    # 計算型心與面積
    for c in contours:
        # 計算色塊面積
        block_area = cv2.contourArea(c)

        # 面積符合使用門檻方取用(降噪處理)
        if block_area >= ANG_LINE_TRACK_AREA_L:
            # 取得型心位置(左上為原點，加法增量)
            moment = cv2.moments(c)
            if moment["m00"] != 0:
                moment_x_raw = int(moment["m10"] / moment["m00"])
                moment_y_raw = int(moment["m01"] / moment["m00"])
            else:
                moment_x_raw = 0
                moment_y_raw = 0

            # 矩形框選
            (x, y, w, h) = cv2.boundingRect(c)

            # 計算角度
            if h == 0 and img_th[y, x] == 255:
                angle_line = -90

            elif h == 0 and img_th[y, x] == 255:
                angle_line = 90

            else:
                angle_line = math.degrees(math.atan(w / h + 0.01))
                if img_th[y, x] == 255:
                    angle_line *= -1

            # 計算色塊位置(相對於畫面下方中心)
            block_x_offset = moment_x_raw - BODY_REF_X - BODY_OFFSET_X
            block_y_offset = 2 * BODY_REF_Y - moment_y_raw - BODY_OFFSET_Y

            moment_x_register.append(block_x_offset)
            moment_y_register.append(block_y_offset)
            area_register.append(block_area)

    # 設定型心運算暫存列表
    moment_area_x = 0
    moment_area_y = 0
    area_sum = 0

    # 計算面積一次矩
    for i in range(len(moment_x_register)):
        moment_area_x += moment_x_register[i] * area_register[i]
        moment_area_y += moment_y_register[i] * area_register[i]
        area_sum += area_register[i]

    # 求得組合型心位置
    if len(moment_x_register) != 0:
        block_x = moment_area_x / area_sum
        block_y = moment_area_y / area_sum
    else:
        block_x = 0
        block_y = 0

    # 輸出影像(GUI顯示用)
    if MODE == 'LINE':
        CAM_IMAGE_LIVE = img_th

    return block_x, block_y, angle_line


# ========== 角度計算 =================================================
# 本函式供計算型心點與原點連線和畫面Y軸夾角計算，於航向控制器使用
# 可解決平行線夾角為零的問題，所得的計算值相當於誤差值為本式特點，整合目標與回授
# 輸出值右側為正，左側為負(使用global Frame參照)
def angle_calculate(block_x, block_y):
    # 計算與中心線夾角
    if block_x == 0:
        angle = 0
    elif block_y == 0 and block_x > 0:
        angle = 90
    elif block_y == 0 and block_x < 0:
        angle = -90
    else:
        angle = math.degrees(math.atan(block_x/block_y))

    return -angle
