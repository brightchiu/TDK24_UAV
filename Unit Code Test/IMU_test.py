import matplotlib.pyplot as plt
from dronekit import connect
import math
import time

# 與UAV執行連線
print('<啟動程序> 與UAV連線中，請稍候...')
uav = connect('/dev/serial0', wait_ready=True, baud=921600)
print('<啟動程序> UAV已連線')

R_V_N = []
R_V_E = []
R_V_D = []
R_YAW = []

for i in range(20):
    hdg = uav.heading
    yaw = math.degrees(uav.attitude.yaw)
    vn = uav.velocity[0]
    ve = uav.velocity[1]
    vd = uav.velocity[2]
    print('HDG = %d, YAW = %.2f' % (hdg, yaw))
    print('V_N = %.2f, V_E = %.2f, V_D = %.2f\n' % (vn, ve, vd))

    R_V_N.append(vn)
    R_V_E.append(ve)
    R_V_D.append(vd)
    R_YAW.append(yaw)

    time.sleep(0.5)

num = []
for i in range(1, len(R_YAW) + 1):
    num.append(i)

plt.plot(num, R_V_N, label="North Velocity")
plt.plot(num, R_V_E, label="East Velocity")
plt.plot(num, R_V_D, label="Down Velocity")
plt.plot(num, R_YAW, label="Yaw")

plt.xlabel('Data number')
plt.ylabel('Velocity(m/s), Yaw-Att(Degree)')
plt.title('Velocity Data')
plt.legend()
plt.savefig('Velocity Data.png')
plt.show()
