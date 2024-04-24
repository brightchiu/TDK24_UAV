# PID控制器 物件導向寫法
import time


class PidController:
    time = __import__('time')

    def __init__(self):
        self.reference = 0
        self.feedback = 0
        self.error = 0
        self.int_error = 0
        self.previous_error = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.int_limit = 100000
        self.gain_i_limit = 100000
        self.gain_d_limit = 100000
        self.previous_time = self.time.time()
        self.first_loop = True

    def set_k_gain(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_gain_limit(self, i_limit, d_limit):
        self.gain_i_limit = i_limit
        self.gain_d_limit = d_limit

    def set_integral_limit(self, int_limit):
        self.int_limit = int_limit

    def reference_update(self, r):
        self.reference = r

    def feedback_update(self, h):
        self.feedback = h

    def ctrl_output(self):
        # 計算誤差值
        self.error = self.reference - self.feedback

        # 誤差值微積分計算
        current_time = self.time.time()

        # 第一迴圈圈僅作時間更新
        if self.first_loop:
            self.previous_time = current_time
            self.first_loop = False
            return 0

        # 正常迴圈
        else:
            # 誤差值微積分計算
            delta_error = self.error - self.previous_error
            delta_time = current_time - self.previous_time
            self.int_error += (self.error * delta_time)
            derivative = delta_error / delta_time

            # 積分器限制
            if self.int_error > self.int_limit:
                self.int_error = self.int_limit
            elif self.int_error < -self.int_limit:
                self.int_error = -self.int_limit

            # 誤差增益處理
            gain_p = self.error * self.kp
            gain_i = self.int_error * self.ki
            gain_d = derivative * self.kp

            # 增益限制器
            if gain_i > self.gain_i_limit:
                gain_i = self.gain_i_limit
            elif gain_i < -self.gain_i_limit:
                gain_i = -self.gain_i_limit

            if gain_d > self.gain_d_limit:
                gain_d = self.gain_d_limit
            elif gain_d < -self.gain_d_limit:
                gain_d = -self.gain_d_limit

            # 加總輸出
            gain_out = gain_p + gain_i + gain_d

            # 更新數值
            self.previous_error = self.error
            self.previous_time = current_time

            # 返還增益值
            return gain_out


class MoveAverage:
    np = __import__('numpy')

    def __init__(self, sn=10):
        self.register = []
        self.sample_number = sn

    def maf(self, value):
        self.register.insert(0, value)
        if len(self.register) > self.sample_number:
            self.register.pop()
        maf = self.np.average(self.register)

        return maf


# 應用示例
alt = PidController()
alt.set_k_gain(5, 0, 0)
alt_maf = MoveAverage()
for i in range(100):
    alt.reference_update(50)
    alt.feedback_update(i)
    cmd = alt.ctrl_output()
    cmd = alt_maf.maf(cmd)
    print(cmd)
    time.sleep(0.5)
