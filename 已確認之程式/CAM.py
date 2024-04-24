# 錄影模組(包含影像紀錄與存檔)
# 新增加處理圖轉檔儲存(待驗證)

# 載入模組
import threading
import cv2

# ========== 旗標與變數 ================================================
# ->旗標
STATE_THREAD = False            # 多線程旗標
STATE_CAM = False               # 影像擷取程式旗標
MODE = ''                       # 模式旗標(POS:定位模式，LINE:循跡模式)

# ->多線程優先執行命令
THREAD_OCCUPY = threading.Lock()

# ->迴圈頻率(預設20Hz)
FREQ_CAM = 30                   # 影像禎率(fps，依系統負載調整)

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


# ========== 影像擷取程式 ==============================================
# 負責截取機載影像畫面，儲存至全域值供控制器使用
# 因主線程衝突故將畫面顯示整合至GUI介面中方便瀏覽
# 另將飛行影像存成影片以供記錄(處理後之影像若需錄下，須經格式轉換處理，暫不增加)
def camera():
    global CAM_IMAGE, STATE_CAM, STATE_THREAD
    # 程式迴圈(同捆執行模式)
    while CAM_VIDEO.isOpened() and STATE_THREAD:
        # 確認影像擷取狀態
        state_grab = CAM_VIDEO.grab()

        # 擷取成功
        if state_grab:
            # 同捆執行起點
            THREAD_OCCUPY.acquire()

            # 擷取影像並儲存
            state_retrieve, image = CAM_VIDEO.retrieve()
            CAM_IMAGE = cv2.resize(image, (CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT))

            # 寫入錄影檔
            if MODE != '':
                # 從灰階轉換回BGR色域
                img_origin = cv2.cvtColor(CAM_IMAGE_LIVE, cv2.COLOR_GRAY2BGR)
                VIDEO_SAVE.write(img_origin)
            else:
                VIDEO_SAVE.write(CAM_IMAGE)

            # 回報影像擷取程式啟動
            if not STATE_CAM:
                STATE_CAM = True
                print('<啟動程序> 影像擷取程式：啟動\n')

            # 顯示影像視窗
            if MODE == 'POS' or MODE == 'LINE':
                cv2.imshow("Flight Camera", CAM_IMAGE_LIVE)
            else:
                cv2.imshow("Flight Camera", CAM_IMAGE)

            # 同捆執行終點
            THREAD_OCCUPY.release()

            # 手動關閉影像顯示
            if cv2.waitKey(int(1000 / FREQ_CAM)) & 0xff == ord("q"):
                STATE_THREAD = False
                print('<關閉程序> 已手動關閉相機，請手動重啟程式')
                break

        # 擷取失敗
        else:
            print('<錯誤> 影像擷取程式：無法擷取影像')
            STATE_THREAD = False
            print('<嚴重錯誤> 請檢查相機並手動重啟程式!!!!')
            break

    # 當系統狀態為否時，跳出迴圈並關閉函式
    CAM_VIDEO.release()
    VIDEO_SAVE.release()
    cv2.destroyAllWindows()  # 關閉影像顯示
    STATE_CAM = False
    print('<關閉程序> 影像擷取程式：關閉')
