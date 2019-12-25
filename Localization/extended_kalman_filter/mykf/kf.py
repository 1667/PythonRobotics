import numpy as np
import math
import matplotlib.pyplot as plt



###
# 卡尔曼滤波
# 根据上一时刻的状态估计，预测当前状态，将预测状态和测量值加权，求和后的结果就是当前的实际状态，而不止听信当前测量值

# 第一步：预测部分
# x' = Fx + u 根据输入值更新状态值,u表示另外的影响
# P' = FPFT+Q P 表示系统的不确定性
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
        self.F_ = F_in # 状态转移矩阵，也就是根据状态值预测状态值的计算方式
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

    def Prediction(self,u_in = None):

        # self.B_ = np.array([[self.DT * math.cos(self.x_[2, 0]), 0],
        #           [self.DT * math.sin(self.x_[2, 0]), 0],
        #           [0.0, self.DT],
        #           [1.0, 0.0]])
        # self.x_ = self.F_ @ self.x_ + self.B_@u_in

        # self.F_ = np.array([[1.0, 0, 0, math.cos(self.x_[2, 0])*self.DT,0],
        #                 [0, 1.0, 0, math.sin(self.x_[2, 0])*self.DT,0],
        #                 [0, 0, 1.0, 0, self.DT],
        #                 [0, 0, 0, 1, 0],
        #                 [0, 0, 0, 0, 1]])
        # self.F_ = np.array([[1.0,0.1],
        #             [0,1]])

        self.x_ = self.F_ @ self.x_
        self.P_ = self.F_ @ self.P_ @ self.F_.T+self.Q_

    def MeasurementUpdate(self,z_in):
        # print("ud",z_in)
        y = z_in - self.H_ @ self.x_

        S = self.H_ @ self.P_ @ self.H_.T+self.R_
        K = self.P_ @ self.H_.T @ np.linalg.inv(S)

        self.x_ = self.x_ + K@y
        self.P_ = (np.eye(self.x_.shape[0])-K@self.H_)@self.P_


if __name__ == "__main__":

    z = [i for i in range(100)]
    z_watch = np.mat(z)

    noise = np.round(np.random.normal(0,5,100),2)
    noise_mat = np.mat(noise)

    print(noise_mat.shape)

    z_mat = z_watch+noise_mat
    # print(z_mat.shape,z_mat[1,0])



    x_mat = np.mat([[0,], [0,]])
    # ¶¨Òå³õÊ¼×´Ì¬Ð­·½²î¾ØÕó
    p_mat = np.mat([[1, 0], [0, 1]])
    # ¶¨Òå×´Ì¬×ªÒÆ¾ØÕó£¬ÒòÎªÃ¿ÃëÖÓ²ÉÒ»´ÎÑù£¬ËùÒÔdelta_t = 1
    f_mat = np.mat([[1, 1], [0, 1]])
    # ¶¨Òå×´Ì¬×ªÒÆÐ­·½²î¾ØÕó£¬ÕâÀïÎÒÃÇ°ÑÐ­·½²îÉèÖÃµÄºÜÐ¡£¬ÒòÎª¾õµÃ×´Ì¬×ªÒÆ¾ØÕó×¼È·¶È¸ß
    q_mat = np.mat([[0.0001, 0], [0, 0.0001]])
    # ¶¨Òå¹Û²â¾ØÕó
    h_mat = np.mat([1, 0])
    # ¶¨Òå¹Û²âÔëÉùÐ­·½²î
    r_mat = np.mat([1])

    myk = KalmanFilter(x_mat,f_mat,p_mat,q_mat,h_mat,r_mat)

    for i in range(100):
        myk.Prediction()
        myk.MeasurementUpdate(z_mat[0,i])

        print(myk.x_[0,0])
        plt.plot(myk.x_[0, 0], myk.x_[1, 0], 'ro', markersize = 1)

    # plt.plot(z_watch,z_mat, "ro")
    plt.show()

    
