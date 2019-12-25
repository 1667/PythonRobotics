import numpy as np
import math


###
# 卡尔曼滤波只能用于线性系统，拓展卡尔曼将非线性系统进行线性优化，以扩大卡尔曼滤波的使用范围
# 两者的主要区别在于对状态方程和观测方程的泰勒展开
###

class ExtendedKalmanFilter(object):

    def __init__(self,x_in,F_in = None,Q_in = None,P_in = None):
        self.x_ = x_in 
        self.F_ = F_in
        self.Q_ = Q_in
        self.P_ = P_in

    def Prediction(self,u_in):

        self.B_ = np.array([[self.DT * math.cos(self.x_[2, 0]), 0],
                  [self.DT * math.sin(self.x_[2, 0]), 0],
                  [0.0, self.DT],
                  [1.0, 0.0]])
        self.x_ = self.F_ @ self.x_ + self.B_@u_in
        self.P_ = self.F_ @ self.P_ @ self.F_.T+self.Q_