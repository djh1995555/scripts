#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import sympy
from utils.signals import *
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel
from lon_params_estimator import LonParamsEstimator
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from functools import partial

class LonParamsEstimatorKF(LonParamsEstimator):
    def __init__(self, estimator_params, vehicle_params):
        super(LonParamsEstimatorKF, self).__init__(estimator_params, vehicle_params)
        self._state_num = 4
        self._measure_num = 1
        self._dt = 0.05
        self._kf = KalmanFilter(dim_x = self._state_num, dim_z = self._measure_num)
        # self._kf.Q = np.diag([0.01, 0.01, 0.01, 0.01])
        # self._kf.R = np.diag([0.1])
        self._kf.Q = np.diag([1.0, 0.01, 0.0001, 1000000.0])
        self._kf.R = np.diag([1.0, 1.0])
        self._kf.x = np.array([10, self._nominal_drag_coefficient, self._nominal_rolling_coefficient, self._nominal_vehicle_mass])
        self._kf.P = 1 * np.identity(4)

        self._rotation_equivalent_mass = (self._vehicle_params['inertia_wheels'] + self._vehicle_params['inertia_engine'] * self._vehicle_params['transmission_efficiency'] * self._vehicle_params['axle_drive_ratio']**2) / (self._vehicle_params['effective_tire_radius']**2)

    def update(self, controller_input):
        self._traction = controller_input[TRACTION]
        self._acc = controller_input[A_REPORT]
        self._v = controller_input[V]
        self._pitch = controller_input[PITCH]

        # self._kf.f = partial(self.nonlinear_state_transition_function, 
        #                       traction = self._traction, 
        #                       g = self._vehicle_params['gravity_acceleration'],
        #                       pitch = self._pitch,
        #                       mass_r = self._rotation_equivalent_mass)
        # self._kf.h = self.nonlinear_observation_function

        g = self._vehicle_params['gravity_acceleration']

        v, drag_coef, rolling_coef, mass = self._kf.x
        k = self._rotation_equivalent_mass / mass
        f_matrix = np.eye((self._state_num))
        f_matrix[0,0] += -2 * drag_coef * v * self._dt / (mass + self._rotation_equivalent_mass)
        f_matrix[0,1] += -v ** 2 * self._dt / (mass + self._rotation_equivalent_mass)
        f_matrix[0,2] += -mass * g * math.cos(self._pitch) * self._dt / (mass + self._rotation_equivalent_mass)
        f_matrix[0,3] += ((g * math.sin(self._pitch) - g * math.cos(self._pitch) * rolling_coef)-(self._traction - drag_coef * v ** 2)) * self._dt / (mass + self._rotation_equivalent_mass)**2

        # v, drag_coef, rolling_coef, mass = sympy.symbols('v, drag_coef, rolling_coef, mass')
        # state = sympy.Matrix([v, drag_coef, rolling_coef, mass])
        # F_sym = sympy.Matrix([[(self._traction - drag_coef * v ** 2 - mass * g * rolling_coef  * sympy.cos(self._pitch) + mass * g * sympy.sin(self._pitch))/(mass + self._rotation_equivalent_mass)],[0.0],[0.0],[0.0]])
        # F_j = F_sym.jacobian(state)
        # x_v, x_drag_coef, x_rolling_coef, x_mass = self._kf.x
        # f_matrix = np.eye(self._state_num) + np.array(F_j.evalf(subs = {
        #     v: float(x_v), 
        #     drag_coef: float(x_drag_coef), 
        #     rolling_coef: float(x_rolling_coef),
        #     mass: float(x_mass)
        # })).astype(float) * self._dt


        # print('shape:{}'.format(f_matrix.shape))
        # print('f_matrix type:{}'.format(type(f_matrix)))
        self._kf.F = f_matrix
        self._kf.predict()

        v, drag_coef, rolling_coef, mass = self._kf.x
        # self._kf.H = np.array([[1, 0, 0, 0],[drag_coef * v, v**2, mass * g * math.cos(self._pitch), g * math.cos(self._pitch) - g * math.sin(self._pitch) + self._acc]])
        # self.z = np.array([self._v, self._traction - self._rotation_equivalent_mass * self._acc])
        self._kf.H = np.array([[drag_coef * v, v**2, mass * g * math.cos(self._pitch), g * math.cos(self._pitch) - g * math.sin(self._pitch) + self._acc]])
        self.z = np.array([self._v, self._traction - self._rotation_equivalent_mass * self._acc])
        # self._kf.update(self.z)

        print('kf.x:{}'.format(self._kf.x))
        self._v_estimated = max(min(self._kf.x[0],40),0)
        self._drag_coefficient_estimated = max(min(self._kf.x[1],self._drag_coefficient_bound),0)
        self._rolling_coefficient_estimated = max(min(self._kf.x[2],self._rolling_coefficient_bound),0)
        self._vehicle_mass_estimated = max(min(self._kf.x[3],self._vehicle_mass_high_bound),self._vehicle_mass_low_bound)

