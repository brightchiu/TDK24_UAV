import threadsafe_tkinter as tk
import threading
from tkinter import messagebox
import time
HDG = 0
ALT = 0
safe = threading.Lock()


def info():
    global HDG, ALT
    for i in range(1, 100):
        safe.acquire()
        HDG += 5
        ALT += 10
        var1.set(HDG)
        var2.set(ALT)
        safe.release()
        time.sleep(1)




window = tk.Tk()
window.title('TDK 飛行組 飛行控制軟體')
window.geometry('1400x800')
window.configure(background='white')

flight_info_label = tk.Label(window, text='即時飛行資訊')
flight_info_label.pack()

# 以下為 height_frame 群組
info_frame = tk.Frame(window)
info_frame.pack(side=tk.TOP)

alt_frame = tk.Frame(info_frame, height=200, width=200)
alt_frame.pack_propagate(0)
alt_frame.pack(side=tk.LEFT)
hdg_frame = tk.Frame(info_frame, height=200, width=200)
hdg_frame.pack_propagate(0)
hdg_frame.pack(side=tk.LEFT)

var1 = tk.StringVar()
var2 = tk.StringVar()


alt_label = tk.Label(alt_frame, font=('雅痞', 20), text='目前高度')
alt_label.pack(side=tk.TOP)
alt_label_1 = tk.Label(alt_frame, font=('雅痞', 30), textvariable=var2)
alt_label_1.pack(side=tk.TOP)
alt_label_2 = tk.Label(alt_frame, font=('圓體', 30), text='公尺')
alt_label_2.pack(side=tk.TOP)

hdg_label = tk.Label(hdg_frame, font=('', 20), text='航向')
hdg_label.pack(side=tk.TOP)
hdg_label_1 = tk.Label(hdg_frame, font=('', 30), textvariable=var1)
hdg_label_1.pack(side=tk.TOP)
hdg_label_2 = tk.Label(hdg_frame, font=('', 30), text='度')
hdg_label_2.pack(side=tk.TOP)

# 以下為 weight_frame 群組
weight_frame = tk.Frame(window)
weight_frame.pack(side=tk.TOP)
weight_label = tk.Label(weight_frame, text='體重（kg）')
weight_label.pack(side=tk.LEFT)
weight_entry = tk.Entry(weight_frame)
weight_entry.pack(side=tk.LEFT)

result_label = tk.Label(window)
result_label.pack()

calculate_btn = tk.Button(window, text='馬上計算')
calculate_btn.pack()

ret = threading.Thread(target=info)
ret.start()
messagebox.askyesno('起飛確認程序', '請確認起飛區淨空')


window.mainloop()



