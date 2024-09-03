import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import math
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from itertools import product

w = 3.3
a0 = 0.0
a_target = -0.6
m = 10
n = 1.0

def compute_s(theta, v0, jerk_coef1, jerk_coef2, gain_coef1, gain_coef2):
    theta = theta / 180 * math.pi
    jerk = -(jerk_coef1 * theta + jerk_coef2)
    stop_distance_gain = gain_coef1 * theta + gain_coef2
    # speed_gain = max(1.0, min(-v0 + 1.5, 1.5))
    mid_t = int((a_target - a0)/jerk * 100) / 100
    s1 = v0 * mid_t + 0.5 * a0 * mid_t * mid_t + 1/6 * jerk * mid_t * mid_t * mid_t
    # s2 = s1 + v_new * (end_t-mid_t) + 0.5 * a_target * (end_t-mid_t) * (end_t-mid_t)
    s2 = s1 + stop_distance_gain  * (-v0 * v0 / (2 * a_target))
    return s2

v = 0.7
theta = 1.5
print(f"v = {v}, theta = {theta}, dist = {compute_s(theta, v, 10, 1.0, -3.8, 1.05)}")
print(f"v = {v}, theta = {theta}, dist = {compute_s(theta, v, 10, 1.0, -4.0, 1.0)}")
print(f"v = {v}, theta = {theta}, dist = {compute_s(theta, v, 10, 1.0, -5.0, 1.0)}")
print(f"v = {v}, theta = {theta}, dist = {compute_s(theta, v, 10, 1.1, -3.8, 1.05)}")
print(f"v = {v}, theta = {theta}, dist = {compute_s(theta, v, 11, 1.0, -3.8, 1.05)}")
print(f"v = {v}, theta = {theta}, dist = {compute_s(theta, v, 10, 1.0, -6.0, 1.00)}")

plot = False
if(plot): 
    THETA = np.linspace(10, 2, 50)
    V0 = np.linspace(0.3, 1.0, 50)
    DIST = np.linspace(0.0, 1.5, 50)

    # Unpack x, y, z values for plotting
    combinations = list(product(THETA, V0))
    theta_values, v0_values = zip(*combinations)

    s1_values = [compute_s(theta, v0, 10, 1.0, -3.8, 1.05) for theta, v0 in combinations]
    s2_values = [compute_s(theta, v0, 10, 1.0, -4.8, 1.0) for theta, v0 in combinations]

    min_dist = [0.3 for theta, v0 in combinations]


    combinations2 = list(product(THETA, DIST))
    theta_values2, dist_values2 = zip(*combinations2)
    min_speed = [0.3 for theta, dist in combinations2]


    # successed_df = pd.read_csv('/home/mi/debug/scripts/successed.csv')
    # successed_x = successed_df.iloc[:, 1]
    # successed_y = successed_df.iloc[:, 2]
    # successed_z = successed_df.iloc[:, 3]
    # successed_end_err = successed_df.iloc[:, 4]

    # failed_df = pd.read_csv('/home/mi/debug/scripts/failed.csv')
    # failed_x = failed_df.iloc[:, 1]
    # failed_y = failed_df.iloc[:, 2]
    # failed_z = failed_df.iloc[:, 3]
    # failed_end_err = failed_df.iloc[:, 4]

    # Create a 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_trisurf(theta_values, v0_values, s1_values, color = "gray", edgecolor='none')
    ax.plot_trisurf(theta_values, v0_values, s2_values, color = "b", edgecolor='none')
    ax.plot_trisurf(theta_values, v0_values, min_dist, color = "gray", edgecolor='none')

    # ax.plot_trisurf(theta_values2, min_speed, dist_values2,color = "gray", edgecolor='none')

    # ax.scatter(successed_x, successed_y, successed_z, color='green', label = "successed")
    # ax.scatter(failed_x, failed_y, failed_z, color='red', label = "failed")


    # all_df = pd.read_csv('/home/mi/debug/scripts/all.csv')
    # all_x = all_df.iloc[:, 1]
    # all_y = all_df.iloc[:, 2]
    # all_z = all_df.iloc[:, 3]
    # all_end_err = all_df.iloc[:, 4]
    # sc = ax.scatter(all_x, all_y, all_z, c=all_end_err)
    # plt.colorbar(sc)


    plt.legend()
    # Set labels and title
    ax.set_xlabel('slope')
    ax.set_ylabel('speed')
    ax.set_zlabel('distacne')
    plt.title('3D Scatter Plot of Columns 2, 3, and 4')

    # Show the plot
    plt.show()