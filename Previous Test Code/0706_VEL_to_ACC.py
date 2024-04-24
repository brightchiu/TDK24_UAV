import time
import math
import matplotlib.pyplot as plt

VEL_CMD = 1
ACC_OUT = 0
R_ACC = []
R_NUM = []
R_VEL = []
R_POS = []
sleep = 0.01

M = 0.5
BETA = 1

rising_time = math.sqrt(VEL_CMD/(M*(1+BETA)))
middle_time = rising_time * BETA
working_time = 2*rising_time+middle_time
print('rising_time:',rising_time)
print('middle_time:',middle_time)
print('working_time:',working_time)

start_time = time.time()
now_time = time.time()
v = 0
s = 0

while now_time-start_time < working_time:
    t = now_time-start_time
    if t < rising_time:
        ACC_OUT = M*t
        print('rising')
    elif t < (rising_time+middle_time):
        ACC_OUT = rising_time*M
        print('middle')
    elif t < working_time:
        ACC_OUT = M * (rising_time*(2+BETA)-t)
        print('decreasing')
    R_ACC.append(ACC_OUT)
    v += sleep * ACC_OUT
    R_VEL.append(v)
    s += sleep * v
    R_POS.append(s)
    time.sleep(sleep)
    now_time = time.time()

VEL_CMD = -1
rising_time = math.sqrt(math.fabs(VEL_CMD)/(M*(1+BETA)))
middle_time = rising_time * BETA
working_time = 2*rising_time+middle_time
start_time = time.time()
now_time = time.time()

while now_time-start_time < working_time:
    t = now_time-start_time
    if t < rising_time:
        ACC_OUT = -M*t
        print('rising')
    elif t < (rising_time+middle_time):
        ACC_OUT = rising_time* -M
        print('middle')
    elif t < working_time:
        ACC_OUT = -M * (rising_time*(2+BETA)-t)
        print('decreasing')
    R_ACC.append(ACC_OUT)

    v += sleep * ACC_OUT
    R_VEL.append(v)
    s += sleep * v
    R_POS.append(s)
    time.sleep(sleep)
    now_time = time.time()

VEL_CMD = 0.5
rising_time = math.sqrt(math.fabs(VEL_CMD)/(M*(1+BETA)))
middle_time = rising_time * BETA
working_time = 2*rising_time+middle_time
start_time = time.time()
now_time = time.time()

while now_time-start_time < working_time:
    t = now_time-start_time
    if t < rising_time:
        ACC_OUT = -M*t
        print('rising')
    elif t < (rising_time+middle_time):
        ACC_OUT = rising_time* -M
        print('middle')
    elif t < working_time:
        ACC_OUT = -M * (rising_time*(2+BETA)-t)
        print('decreasing')
    R_ACC.append(ACC_OUT)
    v += sleep * ACC_OUT
    R_VEL.append(v)
    s += sleep * v
    R_POS.append(s)
    time.sleep(sleep)
    now_time = time.time()

for i in range(1, len(R_ACC) + 1, 1):
    R_NUM.append(i)

print(R_ACC)
print(R_NUM)

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

plt.plot(R_NUM, R_POS, label="POS_out")

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