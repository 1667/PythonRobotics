import numpy as np
import math

###
# 卡尔曼滤波
# 第一步：预测部分
# x' = Fx + u 根据输入值更新状态值
# P' = FPFT+Q
# 第二步：观测
# y = z - Hx' 计算测量值与预测值的差值 H 为测量矩阵，将状态值转换成测量值

# S = HP'HT+R R是测量噪声矩阵，表示测量值与真值之间的差值
# K = P'HTS-1 求卡尔曼增益K，也就是y值的权重

# x = x'+Ky 更新状态向量，考虑了测量值，预测值和噪声
# P = (I-KH)P' 更新不确定度，用于下一周期

###

class KalmanFilter(object):
    def __init__(self,x_in,F_in,P_in,Q_in,H_in,R_in):
        self.x_ = x_in # 状态向量
        self.F_ = F_in # 状态转移矩阵，也就是根据测量值转换成状态值的计算方式
        self.P_ = P_in # 状态协方差矩阵，表示状态的不确定性，根据实际情况填写，会随着系统更新而更新
        self.Q_ = Q_in # 过程噪声，就是外界无法估计的噪声，一般为单位矩阵
        self.H_ = H_in # 测量矩阵，将状态值转换成测量值
        self.R_ = R_in # R是测量噪声矩阵
        self.DT = 0.1
    def set_F(self,F_in):
        self.F_ = F_in

    def set_P(self,P_in):
        self.P_ = P_in
    
    def set_Q(self,Q_in):
        self.Q_ = Q_in

    def Prediction(self,u_in):

        self.B_ = np.array([[self.DT * math.cos(self.x_[2, 0]), 0],
                  [self.DT * math.sin(self.x_[2, 0]), 0],
                  [0.0, self.DT],
                  [1.0, 0.0]])
        self.x_ = self.F_ @ self.x_ + self.B_@u_in
        self.P_ = self.F_ @ self.P_ @ self.F_.T+self.Q_

    def MeasurementUpdate(self,z_in):
        y = z_in - self.H_ @ self.x_

        S = self.H_ @ self.P_ @ self.H_.T+self.R_
        K = self.P_ @ self.H_.T @ np.linalg.inv(S)

        self.x_ = self.x_ + K@y
        self.P_ = (np.eye(self.x_.shape[0])-K@self.H_)@self.P_



