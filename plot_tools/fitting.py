import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# 原始数据
data = np.array([
[6.171,0.547,0.446],
[6.125,0.579,0.497],
[5.610,0.523,0.363],
[4.429,0.571,0.548],
[3.866,0.666,0.507],
[4.401,0.610,0.546],
[6.507,0.618,0.480],
[5.562,0.555,0.426],
[4.293,0.935,0.793],
[6.737,0.531,0.369],
[4.241,0.579,0.471],
[5.290,0.618,0.499],
[5.442,0.769,0.553],
[5.157,0.682,0.605],
[5.235,0.523,0.517],
[5.933,0.586,0.490],
[6.426,0.563,0.322],
[4.288,0.610,0.468],
[3.869,0.856,0.737],
])

# 定义拟合函数
def fit_func(params, x):
    k = params[0]
    return (1 - k * x[0]) * x[1]

# 定义损失函数
def loss_func(params, x):
    c_fit = fit_func(params, x)
    c_actual = x[2]
    return (c_fit - c_actual) ** 2

# 初始化拟合系数
initial_params = [1.0]

# 最小化损失函数
res = minimize(loss_func, initial_params, args=(data[0]), method='Nelder-Mead')

# 打印拟合结果
print("拟合系数为:", res.x[0])
errors = []
for i in range(data.shape[0]):
	c_fit = fit_func(res.x, data[i,:])
	errors.append(c_fit - data[i,2])
print(errors)

plt.figure()

plt.plot(errors)
plt.show()