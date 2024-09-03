import math
import numpy as np
import matplotlib.pyplot as plt

w = 3.3
L = 4.998
g = 9.81
M = 2100

def plot_2d():
	theta = -5.1 / 180 * math.pi
	V0 = 0.8

	sample_time = 0.05
	t = np.arange(0, 10 + sample_time, sample_time)
	v = (2*V0*L+(L-w)*2*g*math.sin(theta)*t) / (2*L-g*math.sin(theta)*t*t)
	s = 0
	for ts in t:
		v_t = (2*V0*L+(L-w)*2*g*math.sin(theta)*ts) / (2*L-g*math.sin(theta)*ts*ts)
		s += v_t * sample_time
		if(v_t<0):
			break

	plt.plot(t, v)
	plt.xlabel('t')
	plt.ylabel('v')
	plt.title(f"theta:{theta *180 /math.pi}, v0:{V0}, s:{s}")
	plt.grid(True)
	plt.show()

def plot_3d():
	from itertools import product
	from mpl_toolkits.mplot3d import Axes3D
	THETA = np.arange(-10, -3, 0.1) / 180 * math.pi
	
	V0 = np.arange(0, 1, 0.05)

	combinations = list(product(THETA, V0))

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

	s_values = [compute_s(theta, v0) for theta, v0 in combinations]

	# Unpack x, y, z values for plotting
	theta_values, v0_values = zip(*combinations)

	# Create 3D plot
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	# Plot the values
	ax.scatter(theta_values, v0_values, s_values)
	# ax.plot_surface(theta_values, v0_values, s_values, cmap='viridis')

	# Set labels
	ax.set_xlabel('theta')
	ax.set_ylabel('v0')
	ax.set_zlabel('s')

	# Show the plot
	plt.show()

plot_3d()
# plot_2d()

