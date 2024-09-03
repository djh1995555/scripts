import math
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from itertools import product

w = 3.3
a0 = 0.0
a_target = -0.5
m = 10
n = 0.2

def compute_s(theta, v0):
    jerk = -(m * theta + n)
    mid_t = int((a_target - a0)/jerk * 100) / 100	
    v_new = v0 + a0 * mid_t + 0.5 * jerk * mid_t * mid_t
    end_t = -v_new / a_target + mid_t
    s1 = v0 * mid_t + 0.5 * a0 * mid_t * mid_t + 1/6 * jerk * mid_t * mid_t * mid_t
    # s2 = s1 + v_new * (end_t-mid_t) + 0.5 * a_target * (end_t-mid_t) * (end_t-mid_t)
    s2 = s1 + (-v_new * v_new / (2 * a_target))
    print(f"theta:{(theta / math.pi * 180):.3f}, v0:{v0:.3f}, jerk:{jerk:.3f}, mid_t:{mid_t:.3f}, end_t:{end_t:.3f}, s2:{s2:.3f}")
    return s2

THETA = np.linspace(10, 2, 50) / 180 * math.pi
V0 = np.linspace(0.1, 0.8, 50)

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
ax.scatter(theta_values, v0_values, s_values, label = "original")
ax.scatter(theta_values, v0_values, Z_fit, label = "fitting")
plt.legend()

theta = -10 /180 * math.pi
v0 = 0.8
print(f"theta:{theta * 180 / math.pi}, v0:{v0}, s:{compute_s(theta, v0)}")

plt.show()