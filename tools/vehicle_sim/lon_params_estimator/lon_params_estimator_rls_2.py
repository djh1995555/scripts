#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel
from lon_params_estimator import LonParamsEstimator
from rls_estimator import RLSEstimator
from mfrls_estimator import MFRLSEstimator
from rawrls_estimator import RAWRLSEstimator
from rawrgtls_estimator import RAWRGTLSEstimator
from rgtls_estimator import RGTLSEstimator
from mfrawrls_estimator import MFRAWRLSEstimator
from noise_covariance_estimator import NoiseCovarianceEstimator

class LonParamsEstimatorRLS2(LonParamsEstimator):
    def __init__(self, estimator_params, vehicle_params):
        super(LonParamsEstimatorRLS2, self).__init__(estimator_params, vehicle_params)
        poly_order = estimator_params['noise_covariance_estimator_params']['poly_order']
        wl = estimator_params['noise_covariance_estimator_params']['wl']
        wr = estimator_params['noise_covariance_estimator_params']['wr']
        forget_factor = estimator_params['noise_covariance_estimator_params']['forget_factor']
        init_state_1 = np.array([[self._nominal_vehicle_mass, 0.0]])
        init_state_1 = init_state_1.T
        self._noise_covariance_estimator = NoiseCovarianceEstimator(init_state_1.shape[1]+1, poly_order, wl, wr, forget_factor)
        init_P_1 = 0.1 * np.identity(init_state_1.shape[0])
        init_state_2 = np.array([[self._nominal_drag_coefficient, self._nominal_rolling_coefficient]])
        init_state_2 = init_state_2.T
        init_P_2 = 0.1 * np.identity(init_state_2.shape[0])
        if(estimator_params['rls_type'] == 'RLS'):
            self._first_rls = RLSEstimator(init_state_1, init_P_1, estimator_params['RLS_params'])
            self._second_rls = RLSEstimator(init_state_2, init_P_2, estimator_params['RLS_params'])
        elif(estimator_params['rls_type'] == 'MFRLS'):
            self._first_rls = MFRLSEstimator(init_state_1, init_P_1, estimator_params['MFRLS_params'])
            self._second_rls = MFRLSEstimator(init_state_2, init_P_2, estimator_params['MFRLS_params'])
        elif(estimator_params['rls_type'] == 'RAWRLS'):
            self._first_rls = RAWRLSEstimator(init_state_1, init_P_1, estimator_params['RAWRLS_params'])
            self._second_rls = RAWRLSEstimator(init_state_2, init_P_2, estimator_params['RAWRLS_params'])
        elif(estimator_params['rls_type'] == 'RGTLS'):
            init_P_1 = 1 * np.identity(init_state_1.shape[0]+1)
            init_P_2 = 1 * np.identity(init_state_2.shape[0]+1)
            self._first_rls = RGTLSEstimator(init_state_1, init_P_1, estimator_params['RGTLS_params'])
            self._second_rls = RGTLSEstimator(init_state_2, init_P_2, estimator_params['RGTLS_params'])
        elif(estimator_params['rls_type'] == 'RAWRGTLS'):
            init_P_1 = 1 * np.identity(init_state_1.shape[0]+1)
            init_P_2 = 1 * np.identity(init_state_2.shape[0]+1)
            self._first_rls = RAWRGTLSEstimator(init_state_1, init_P_1, estimator_params['RAWRGTLS_params']) 
            self._second_rls = RAWRGTLSEstimator(init_state_2, init_P_2, estimator_params['RAWRGTLS_params'])       
        elif(estimator_params['rls_type'] == 'MFRAWRLS'):
            self._first_rls = MFRAWRLSEstimator(init_state_1, init_P_1, estimator_params['MFRAWRLS_params'])
            self._second_rls = MFRAWRLSEstimator(init_state_2, init_P_2, estimator_params['MFRAWRLS_params'])
        else:
            self._first_rls = RLSEstimator(init_state_1, init_P_1, estimator_params['RLS_params'])
            self._second_rls = RLSEstimator(init_state_2, init_P_2, estimator_params['RLS_params'])

    def update(self, controller_input):
        self._traction_history.put(controller_input[TRACTION])
        self._v_history.put(controller_input[V])
        self._pitch_history.put(controller_input[PITCH])
        if(self._vehicle_params['delay_correction']):
            if(self._traction_history.qsize() >= int(self._delay_frames)):
                self._traction = self._traction_history.get()
                self._v = self._v_history.get()
                self._pitch = self._pitch_history.get()
        else:
            self._traction = controller_input[TRACTION]
            self._v = controller_input[V]
            self._pitch = controller_input[PITCH]

        self._acc = controller_input[A_REPORT]
        rotation_equivalent_mass = (self._vehicle_params['inertia_wheels'] + self._vehicle_params['inertia_engine'] * self._vehicle_params['transmission_efficiency'] * self._vehicle_params['axle_drive_ratio']**2) / (self._vehicle_params['effective_tire_radius']**2)
    
        y = self._traction - rotation_equivalent_mass * self._acc - self._nominal_drag_coefficient * self._v ** 2
        g = self._vehicle_params['gravity_acceleration']
        phi = np.array([[self._acc - g * math.sin(self._pitch) + g * math.cos(self._pitch) * self._nominal_rolling_coefficient, 
                         -1]])
        
        self._noise_covariance = self._noise_covariance_estimator.update([phi[0,0], phi[0,1], y])
        self._first_rls.set_noise_covariance(self._noise_covariance)
        self._phi = phi
        self._first_rls.set_phi(phi)
        self._first_rls.update(y)
        first_state = self._first_rls.get_state()
        self._err = self._first_rls.get_err()
        self._P = self._first_rls.get_state_covariance()
        self._K = self._first_rls.get_K()

        self._vehicle_mass_estimated = max(min(first_state[0,0],self._vehicle_mass_high_bound),self._vehicle_mass_low_bound)
        self._disturbance = max(min(first_state[1,0],10000),-10000)

        y = -first_state[1,0] + self._nominal_drag_coefficient * self._v ** 2 + g * math.cos(self._pitch) * self._nominal_rolling_coefficient * first_state[0,0]
        phi = np.array([[self._v ** 2, first_state[0,0] * g * math.cos(self._pitch)]])
        self._second_rls.set_phi(phi)
        self._second_rls.update(y)
        second_state = self._second_rls.get_state()

        self._drag_coefficient_estimated = max(min(second_state[0,0],self._drag_coefficient_bound),0)
        self._rolling_coefficient_estimated = max(min(second_state[1,0],self._rolling_coefficient_bound),0)
        # print('_vehicle_mass_estimated:{}\n_drag_coefficient_estimated:{}\n_rolling_coefficient_estimated:{}'.format(self._vehicle_mass_estimated, self._drag_coefficient_estimated, self._rolling_coefficient_estimated))