#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import sympy
from utils.signals import *
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel
from lon_params_estimator import LonParamsEstimator
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from functools import partial

class LonParamsEstimatorEKF(LonParamsEstimator):
    def __init__(self, estimator_params, vehicle_params):
        super(LonParamsEstimatorEKF, self).__init__(estimator_params, vehicle_params)
        self._state_num = 4
        self._measure_num = 2
        self._dt = 0.05
        self._ekf = ExtendedKalmanFilter(dim_x = 4, dim_z = 1)
        self._ekf.Q = np.diag([0.1, 0.01, 0.00001, 100000.0])
        self._ekf.R = np.diag([1.0, 1.0])
        self._ekf.x = np.matrix([10, self._nominal_drag_coefficient, self._nominal_rolling_coefficient, self._nominal_vehicle_mass]).T
        self._ekf.P = 1.0 * np.identity(4)

        self._rotation_equivalent_mass = (self._vehicle_params['inertia_wheels'] + self._vehicle_params['inertia_engine'] * self._vehicle_params['transmission_efficiency'] * self._vehicle_params['axle_drive_ratio']**2) / (self._vehicle_params['effective_tire_radius']**2)

    def update(self, controller_input):
        self._traction = controller_input[TRACTION]
        self._acc = controller_input[A_REPORT]
        self._v = controller_input[V]
        self._pitch = controller_input[PITCH]

        g = self._vehicle_params['gravity_acceleration']

        v, drag_coef, rolling_coef, mass = sympy.symbols('v, drag_coef, rolling_coef, mass')
        state = sympy.Matrix([v, drag_coef, rolling_coef, mass])
        F_sym = sympy.Matrix([[(self._traction - drag_coef * v ** 2 - mass * g * rolling_coef  * sympy.cos(self._pitch) + mass * g * sympy.sin(self._pitch))/(mass + self._rotation_equivalent_mass)],[0.0],[0.0],[0.0]])
        F_j = F_sym.jacobian(state)
        x_v, x_drag_coef, x_rolling_coef, x_mass = self._ekf.x

        self._ekf.F = np.eye(self._state_num) + np.matrix(F_j.evalf(subs = {
            v: float(x_v), 
            drag_coef: float(x_drag_coef), 
            rolling_coef: float(x_rolling_coef),
            mass: float(x_mass)
        })).astype(float) * self._dt

        self._ekf.predict()

        def H_jacobian(x):
            s_v, s_drag_coef, s_rolling_coef, s_mass = sympy.symbols('v, drag_coef, rolling_coef, mass')
            state = sympy.Matrix([s_v, s_drag_coef, s_rolling_coef, s_mass])
            H_sym = sympy.Matrix([[s_v],[s_drag_coef * s_v ** 2 + s_mass * g * s_rolling_coef * sympy.cos(self._pitch) + s_mass * (self._acc - g * sympy.sin(self._pitch))]])
            H_j = H_sym.jacobian(state)
            v, drag_coef, rolling_coef, mass = x

            H = np.matrix(H_j.evalf(subs = {
                s_v: float(v), 
                s_drag_coef: float(drag_coef), 
                s_rolling_coef: float(rolling_coef),
                s_mass: float(mass)
            })).astype(float)
            return H
        
        def Hx(x):
            v = x[0,0] 
            drag_coef = x[1,0] 
            rolling_coef = x[2,0] 
            mass = x[3,0] 
            Hx = np.array([[v],[drag_coef * v ** 2 + mass * g * rolling_coef * math.cos(self._pitch) + mass * (self._acc - g * math.sin(self._pitch))]], dtype='float')
            return Hx

        z = np.array([[self._v],[self._traction - self._rotation_equivalent_mass * self._acc]], dtype='float')
        print('shape:{}'.format(z.shape))
        self._ekf.update(z, H_jacobian, Hx)

        print('kf.x:{}'.format(self._ekf.x.T))
        self._v_estimated = max(min(self._ekf.x[0],40),0)
        self._drag_coefficient_estimated = max(min(self._ekf.x[1],self._drag_coefficient_bound),0)
        self._rolling_coefficient_estimated = max(min(self._ekf.x[2],self._rolling_coefficient_bound),0)
        self._vehicle_mass_estimated = max(min(self._ekf.x[3],self._vehicle_mass_high_bound),self._vehicle_mass_low_bound)

