import math
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from itertools import product

w = 3.3
L = 4.998
g = 9.81
M = 2100

def compute_s(theta, v0):
    sample_time = 0.05
    t = np.arange(0, 10 + sample_time, sample_time)
    s = 0
    for ts in t:
        v_t = (2*v0*L+(L-w)*2*g*math.sin(theta)*ts) / (2*L-g*math.sin(theta)*ts*ts)
        s += v_t * sample_time
        if(v_t<0):
            break
    # print(f"theta:{theta *180 /math.pi}, v0:{v0}, s:{s}")
    return s

THETA = np.linspace(-10, -3, 50) / 180 * math.pi
V0 = np.linspace(0, 1, 50)

S = []
for theta, v0 in zip(THETA, V0):
    S.append(compute_s(theta, v0))
S = np.array(S)

# 定义要拟合的二元函数（在本例中为二次函数）
def func(data, a):
    x, y = data
    return a*y**2 / (9.8 *x)

# 执行曲面拟合
popt, pcov = curve_fit(func, (THETA, V0), S)


# 提取拟合参数
a = popt

print(f"a:{a}")

# Unpack x, y, z values for plotting
combinations = list(product(THETA, V0))
theta_values, v0_values = zip(*combinations)

s_values = [compute_s(theta, v0) for theta, v0 in combinations]

# 生成拟合曲面数据
Z_fit = np.array([a*v0**2 / (9.8 * theta) for theta, v0 in combinations])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(theta_values, v0_values, s_values)
ax.scatter(theta_values, v0_values, Z_fit)

theta = -10 /180 * math.pi
v0 = 0.8
print(f"theta:{theta * 180 / math.pi}, v0:{v0}, s:{compute_s(theta, v0)}")

plt.show()