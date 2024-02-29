# encoding: utf-8

import time

# simplePID.py
class PID(object):
    def __init__(self, target, P, I, D):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.setPoint = target
        self.err = 0
        self.err_next = 0
        self.err_last = 0
        self.last_result = 0


    def __del__(self):
        print("DEL PID")

    # 重新设置目标值
    def reset_target(self, target):
        self.setPoint = target


    # 增量式PID计算方式
    def incremental(self, current_value, limit=0):
        self.err = self.setPoint - current_value
        result = self.last_result + self.Kp * (self.err - self.err_next) + self.Ki * self.err + self.Kd * (self.err - 2 * self.err_next + self.err_last)
        self.err_last = self.err_next
        self.err_next = self.err
        if limit > 0:
            if result > limit:
                result = limit
            if result < -limit:
                result = -limit
        self.last_result = result
        return result

