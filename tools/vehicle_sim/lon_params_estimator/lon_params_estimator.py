#!/usr/bin/env python
import math
import queue
import numpy as np
import pandas as pd
from utils.signals import *
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel


class LonParamsEstimator(object):
    def __init__(self, estimator_params, vehicle_params):
        self._name = 'BaseLonParamsEstimator'

        self._traction_history = queue.Queue()
        self._v_history = queue.Queue()
        self._pitch_history = queue.Queue()
        self._acc = 0.0
        self._traction = 0.0
        self._v = 0.0
        self._pitch = 0.0
        self._delay_frames = vehicle_params['delay_time']/vehicle_params['sample_time']

        self._weight_scale = 1
        self._estimator_params = estimator_params
        self._vehicle_params = vehicle_params

        self._nominal_drag_coefficient = 3.0
        self._nominal_rolling_coefficient = 0.005
        self._nominal_vehicle_mass = 25000 / self._weight_scale

        self._vehicle_mass_high_bound = 50000 / self._weight_scale
        self._vehicle_mass_low_bound = 25000 / self._weight_scale
        self._drag_coefficient_bound = 10.0
        self._rolling_coefficient_bound = 0.01

        self._v_estimated = 0
        self._drag_coefficient_estimated = self._nominal_drag_coefficient
        self._rolling_coefficient_estimated = self._nominal_rolling_coefficient
        self._vehicle_mass_estimated = self._nominal_vehicle_mass

        self._err = 0.0

    def update(self, controller_input):
        pass

    def get_weight_scale(self):
        return self._weight_scale
    
    def get_v_estimated(self):
        return self._v_estimated
    
    def get_mass_estimated(self):
        return self._vehicle_mass_estimated

    def get_rolling_coefficient_estimated(self):
        return self._rolling_coefficient_estimated

    def get_drag_coefficient_estimated(self):
        return self._drag_coefficient_estimated

    def get_nominal_mass(self):
        return self._nominal_vehicle_mass

    def get_nominal_rolling_coefficient(self):
        return self._nominal_rolling_coefficient

    def get_nominal_drag_coefficient(self):
        return self._nominal_drag_coefficient
    
    def get_err(self):
        return float(self._err)

    def get_state_covariance(self):
        return self._P
    
    def get_K(self):
        return self._K
    
    def get_noise_covariance(self):
        return self._noise_covariance
    
    def get_phi(self):
        return self._phi