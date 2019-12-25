import numpy as np
import matplotlib.pyplot as plt
 
# ´´½¨Ò»¸ö0-99µÄÒ»Î¬¾ØÕó
z = [i for i in range(100)]
z_watch = np.mat(z)
#print(z_mat)
 
# ´´½¨Ò»¸ö·½²îÎª1µÄ¸ßË¹ÔëÉù£¬¾«È·µ½Ð¡ÊýµãºóÁ½Î»
noise = np.round(np.random.normal(0, 1, 100), 2)
noise_mat = np.mat(noise)
 
# ½«zµÄ¹Û²âÖµºÍÔëÉùÏà¼Ó
z_mat = z_watch + noise_mat
#print(z_watch)
 
# ¶¨ÒåxµÄ³õÊ¼×´Ì¬
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
 
for i in range(100):
    x_predict = f_mat * x_mat
    p_predict = f_mat * p_mat * f_mat.T + q_mat
    kalman = p_predict * h_mat.T / (h_mat * p_predict * h_mat.T + r_mat)
    x_mat = x_predict + kalman *(z_mat[0, i] - h_mat * x_predict)
    p_mat = (np.eye(2) - kalman * h_mat) * p_predict
    
    plt.plot(x_mat[0, 0], x_mat[1, 0], 'ro', markersize = 1)
    print(x_mat[0, 0])
plt.show()