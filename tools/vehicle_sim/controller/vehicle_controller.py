#!/usr/bin/env python
import numpy as np
import pandas as pd
from controller.pid_controller import PIDController
from controller.sliding_mode_controller import SlidingModeController
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel
from utils.signals import *
from lon_params_estimator.lon_params_estimator_rls import LonParamsEstimatorRLS
from lon_params_estimator.lon_params_estimator_rls_2 import LonParamsEstimatorRLS2
from lon_params_estimator.lon_params_estimator_rls_3 import LonParamsEstimatorRLS3
from lon_params_estimator.lon_params_estimator_rls_4 import LonParamsEstimatorRLS4
from lon_params_estimator.lon_params_estimator_kf import LonParamsEstimatorKF
from lon_params_estimator.lon_params_estimator_ekf import LonParamsEstimatorEKF
class VehicleController():
    def __init__(self, config):
        self._controller_params = config['controller_params']
        self._controller = PIDController(self._controller_params['PID_args'])
        if(self._controller_params['lon_params_estimator_type'] == 'RLS'):
            self._lon_params_estimator = LonParamsEstimatorRLS(self._controller_params['lon_params_estimator_params'], config['vehicle_param'])
        elif(self._controller_params['lon_params_estimator_type'] == 'RLS2'):
            self._lon_params_estimator = LonParamsEstimatorRLS2(self._controller_params['lon_params_estimator_params'], config['vehicle_param'])
        elif(self._controller_params['lon_params_estimator_type'] == 'RLS3'):
            self._lon_params_estimator = LonParamsEstimatorRLS3(self._controller_params['lon_params_estimator_params'], config['vehicle_param'])
        elif(self._controller_params['lon_params_estimator_type'] == 'RLS4'):
            self._lon_params_estimator = LonParamsEstimatorRLS4(self._controller_params['lon_params_estimator_params'], config['vehicle_param'])
        elif(self._controller_params['lon_params_estimator_type'] == 'KF'):
            self._lon_params_estimator = LonParamsEstimatorKF(self._controller_params['lon_params_estimator_params'], config['vehicle_param'])
        elif(self._controller_params['lon_params_estimator_type'] == 'EKF'):
            self._lon_params_estimator = LonParamsEstimatorEKF(self._controller_params['lon_params_estimator_params'], config['vehicle_param'])
        else:
            self._lon_params_estimator = LonParamsEstimatorRLS(self._controller_params['lon_params_estimator_params'], config['vehicle_param'])
        self._vehicle_model = VehicleDynamicModel(config['vehicle_param'])

        self._vehicle_mass_estimated = config['vehicle_param']['vehicle_mass']
        self._drag_coefficient_estimated = config['vehicle_param']['drag_coefficient']
        self._rolling_friction_coefficient_estimated = config['vehicle_param']['rolling_friction_coefficient']

        self._vehicle_mass_estimated = self._lon_params_estimator.get_nominal_mass()
        self._drag_coefficient_estimated = self._lon_params_estimator.get_nominal_drag_coefficient()
        self._rolling_friction_coefficient_estimated = self._lon_params_estimator.get_nominal_rolling_coefficient()

        self._traction_bound = 30000

    def compute_control_cmd(self, ref_v, controller_input):
        self._target_acc = self._controller.compute_control_cmd(ref_v, controller_input)
        acc_bound = 1.0
        self._target_acc = min(max(self._target_acc,-acc_bound),acc_bound)
        self._lon_params_estimator.update(controller_input)
        if(self._controller_params['use_estimation_result']):
            self._vehicle_mass_estimated = self._lon_params_estimator.get_mass_estimated()
            self._drag_coefficient_estimated = self._lon_params_estimator.get_drag_coefficient_estimated()
            self._rolling_friction_coefficient_estimated = self._lon_params_estimator.get_rolling_coefficient_estimated()
        traction = self.compute_traction(self._target_acc,
                                     1,
                                     controller_input[V],
                                     controller_input[PITCH],
                                     self._vehicle_mass_estimated,
                                     self._drag_coefficient_estimated,
                                     self._rolling_friction_coefficient_estimated)
        # return traction
        return max(min(traction, self._traction_bound), -self._traction_bound)
    
    def compute_traction(self, target_acc, gear_ratio, v, pitch, vehicle_mass = None, drag_coefficient = None, rolling_friction_coefficient = None):
        return self._vehicle_model.compute_traction(target_acc, gear_ratio, v, pitch, vehicle_mass, drag_coefficient, rolling_friction_coefficient)

    def get_target_acc(self):
        return self._target_acc
    
    def get_v_estimated(self):
        return self._lon_params_estimator.get_v_estimated()
    
    def get_mass_estimated(self):
        return self._lon_params_estimator.get_mass_estimated()
    
    def get_rolling_coefficient_estimated(self):
        return self._lon_params_estimator.get_rolling_coefficient_estimated()

    def get_drag_coefficient_estimated(self):
        return self._lon_params_estimator.get_drag_coefficient_estimated()
    
    def get_err(self):
        return self._lon_params_estimator.get_err()
    
    def get_state_covariance(self):
        return self._lon_params_estimator.get_state_covariance()
    
    def get_K(self):
        return self._lon_params_estimator.get_K()
    
    def get_weight_scale(self):
        return self._lon_params_estimator.get_weight_scale()
    
    def get_noise_covariance(self):
        return self._lon_params_estimator.get_noise_covariance()
    
    def get_phi(self):
        return self._lon_params_estimator.get_phi()