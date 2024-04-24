import tkinter as tk
import cv2
import threading
from PIL import Image, ImageTk

video = cv2.VideoCapture(0)


def recorder():
    while video.isOpened():
        ret, cap = video.read()
        if ret:
            image = cv2.resize(cap, (480, 360))
            #cv2.imshow("test", image)

            cam_process = cv2.cvtColor(image, cv2.COLOR_BGR2RGBA)
            cam_process = Image.fromarray(cam_process)
            cam_process = ImageTk.PhotoImage(cam_process)
            cam_image.configure(image=cam_process)
            window.update()
                
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        else:
            break
    video.release()
    cv2.destroyAllWindows()


def thread():
    vid = threading.Thread(target=recorder)
    vid.start()


window = tk.Tk()
window.title('TDK 24th 飛行組 C08 台灣大艦隊 飛行控制介面')
window.geometry('1280x700')
window.configure(background='#eff8fd')


cam_frame = tk.Frame(window, width=360, height=360, bg='black')
cam_frame.pack_propagate(0)
cam_frame.pack(side=tk.TOP, padx=5, pady=5)
cam_headline = tk.Label(cam_frame, font=('', 20), text='----- 影像處理畫面 -----',
                        fg='#007dd1', bg='#eff8fd')
cam_headline.pack(side=tk.TOP)
cam_image = tk.Label(cam_frame, bg='#123456')

cam_image.pack(side=tk.TOP)

thread()
window.mainloop()
