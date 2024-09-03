import math
import numpy as np
import matplotlib.pyplot as plt

w = 3.3
theta = 5 / 180 * math.pi
v0 = 0.875
a0 = 0.0
a_target = -0.5
m = 1
n = 1

def compute_s(theta, v0):
	jerk = -(m * theta + n)
	mid_t = int((a_target - a0)/jerk * 100) / 100	
	v_new = v0 + a0 * mid_t + 0.5 * jerk * mid_t * mid_t
	end_t = -v_new / a_target + mid_t
	s1 = v0 * mid_t + 0.5 * a0 * mid_t * mid_t + 1/6 * jerk * mid_t * mid_t * mid_t
	s2 = s1 + v_new * (end_t-mid_t) + 0.5 * a_target * (end_t-mid_t) * (end_t-mid_t)
	print(f"s:{s2}")

jerk = -(m * theta + n)
mid_t = int((a_target - a0)/jerk * 100) / 100

x1 = np.arange(0, mid_t+0.01, 0.01)
x2 = np.arange(mid_t, 5, 0.01)
v1 = v0 + a0 * x1 + 0.5 * jerk * x1 * x1
v_new = v0 + a0 * x1[-1] + 0.5 * jerk * x1[-1] * x1[-1]
v2 = v_new + a_target * (x2 - x1[-1])
end_t = -v_new / a_target + x1[-1]

x3 = np.arange(mid_t, end_t, 0.01)
s1 = v0 * x1 + 0.5 * a0 * x1 * x1 + 1/6 * jerk * x1 * x1 * x1
v_new = v0 + a0 * x1[-1] + 0.5 * jerk * x1[-1] * x1[-1]
s2 = s1[-1] + v_new * (x3-x1[-1]) + 0.5 * a_target * (x3-x1[-1]) * (x3-x1[-1])

compute_s(theta,v0)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(x1, s1)
ax.plot(x3, s2)
ax.plot(x1, v1)
ax.plot(x2, v2)

plt.title('Function Curve')
plt.grid(True)
plt.show()
