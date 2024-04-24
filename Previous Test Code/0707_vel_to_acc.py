# 0707 Generate the acceleration command from velocity

import time
import math
import matplotlib.pyplot as plt

TARGET_VEL = 0
TARGET_ACC = 0
ACTUAL_VEL = 0


BETA = 1
STATE = 1
M = 1
FREQ = 10
R_ACC = []
R_VEL = []
R_NUM = []
v = 0

while not STATE == 0:
    TARGET_VEL = float(input('Please input the velocity target:'))
    delta_vel = TARGET_VEL - ACTUAL_VEL
    t1 = math.sqrt(math.fabs(delta_vel)/(M*(1+BETA)))
    t2 = t1 + t1 * BETA
    t3 = 2 * t1 + t1 * BETA



    print('rising_time:', t1)
    print('middle_time:', t2)
    print('working_time:', t3)

    # Set timer
    t_start = time.time()
    t_now = time.time()
    while t_now-t_start <= t3:
        t = t_now-t_start
        if t <= t1:
            a = M * t
            print('Rising.')
        elif t <= t2:
            a = M * t1
            print('Middle')
        else:
            a = M * (t1*(2+BETA)-t)
            print('Decreasing.')

        if delta_vel < 0:
            a *= -1

        TARGET_ACC = a
        R_ACC.append(TARGET_ACC)


        time.sleep(1/FREQ)
        t_now = time.time()
        v += a * ((t_now - t_start)-t)
        R_VEL.append(v)

    ACTUAL_VEL = TARGET_VEL
    STATE = float(input('1:Keep in the loop, 0:Close the loop:'))


for i in range(1, len(R_ACC) + 1, 1):
    R_NUM.append(i)

plt.plot(R_NUM, R_ACC, label="ACC_out")

# naming the x axis
plt.ylabel('Value')
# naming the y axis
plt.xlabel('Data number')
# giving a title to my graph
plt.title('Output Data Diagram')
# show a legend on the plot
plt.legend()
# function to show the plot
plt.show()

plt.plot(R_NUM, R_VEL, label="VEL_out")

# naming the x axis
plt.ylabel('Value')
# naming the y axis
plt.xlabel('Data number')
# giving a title to my graph
plt.title('Output Data Diagram')
# show a legend on the plot
plt.legend()
# function to show the plot
plt.show()
