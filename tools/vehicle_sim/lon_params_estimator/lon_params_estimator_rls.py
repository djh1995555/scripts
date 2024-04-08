#!/usr/bin/env python

import math
import numpy as np
import pandas as pd
from utils.signals import *
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel
from lon_params_estimator import LonParamsEstimator
from rls_estimator import RLSEstimator
from ef1rls_estimator import EF1RLSEstimator
from ef2rls_estimator import EF2RLSEstimator
from df1rls_estimator import DF1RLSEstimator
from df2rls_estimator import DF2RLSEstimator
from mfrls_estimator import MFRLSEstimator
from rawrls_estimator import RAWRLSEstimator
from rawrgtls_estimator import RAWRGTLSEstimator
from rgtls_estimator import RGTLSEstimator
from rgef1tls_estimator import RGEF1TLSEstimator
from rgef2tls_estimator import RGEF2TLSEstimator
from rgdf1tls_estimator import RGDF1TLSEstimator
from rgdf2tls_estimator import RGDF2TLSEstimator
from mfrawrls_estimator import MFRAWRLSEstimator
from noise_covariance_estimator import NoiseCovarianceEstimator

class LonParamsEstimatorRLS(LonParamsEstimator):
    def __init__(self, estimator_params, vehicle_params):
        super(LonParamsEstimatorRLS, self).__init__(estimator_params, vehicle_params)
        poly_order = estimator_params['noise_covariance_estimator_params']['poly_order']
        wl = estimator_params['noise_covariance_estimator_params']['wl']
        wr = estimator_params['noise_covariance_estimator_params']['wr']
        forget_factor = estimator_params['noise_covariance_estimator_params']['forget_factor']
        init_state = np.array([[self._nominal_drag_coefficient, 
                                 self._nominal_rolling_coefficient * self._nominal_vehicle_mass, 
                                 self._nominal_vehicle_mass]])
        self._noise_covariance_estimator = NoiseCovarianceEstimator(init_state.shape[1]+1, poly_order, wl, wr, forget_factor)
        init_state = init_state.T
        init_P = 1 * np.identity(init_state.shape[0])
        # init_P[0,0] = 100
        # init_P[1,1] = 100
        # init_P[2,2] = 100
        
        self._scale_T =  np.identity(init_state.shape[0])
        acc_max = 0.5
        v_max = 30.0
        self._scale_T[0,0] = 1 / (v_max ** 2)
        self._scale_T[1,1] = 1 / (self._vehicle_params['gravity_acceleration'])
        self._scale_T[2,2] = 1 / (acc_max)


        # delta_ca = 0.1
        # delta_mass = 500
        # delta_cr = 0.0005

        # delta_cr_mass = delta_cr * delta_mass + delta_cr * self._nominal_vehicle_mass + delta_mass * self._nominal_rolling_coefficient
        # self._scale_T[0,0] = 1
        # self._scale_T[1,1] = delta_ca / delta_cr_mass
        # self._scale_T[2,2] = delta_ca / delta_mass
        # self._scale_T *= 500

        if(estimator_params['rls_type'] == 'RLS'):
            self._rls = RLSEstimator(init_state, init_P, estimator_params['RLS_params'])
        elif(estimator_params['rls_type'] == 'MFRLS'):
            self._rls = MFRLSEstimator(init_state, init_P, estimator_params['MFRLS_params'])
        elif(estimator_params['rls_type'] == 'RAWRLS'):
            self._rls = RAWRLSEstimator(init_state, init_P, estimator_params['RAWRLS_params'])
        elif(estimator_params['rls_type'] == 'EF1RLS'):
            self._rls = EF1RLSEstimator(init_state, init_P, estimator_params['EF1RLS_params'])
        elif(estimator_params['rls_type'] == 'EF2RLS'):
            self._rls = EF2RLSEstimator(init_state, init_P, estimator_params['EF2RLS_params'])
        elif(estimator_params['rls_type'] == 'DF1RLS'):
            self._rls = DF1RLSEstimator(init_state, init_P, estimator_params['DF1RLS_params'])
        elif(estimator_params['rls_type'] == 'DF2RLS'):
            self._rls = DF2RLSEstimator(init_state, init_P, estimator_params['DF2RLS_params'])
        elif(estimator_params['rls_type'] == 'RGTLS'):
            init_P = 1 * np.identity(init_state.shape[0]+1)
            self._rls = RGTLSEstimator(init_state, init_P, estimator_params['RGTLS_params'])
        elif(estimator_params['rls_type'] == 'RGEF1TLS'):
            init_P = 1 * np.identity(init_state.shape[0]+1)
            self._rls = RGEF1TLSEstimator(init_state, init_P, estimator_params['RGEF1TLS_params'])
        elif(estimator_params['rls_type'] == 'RGEF2TLS'):
            init_P = 1 * np.identity(init_state.shape[0]+1)
            self._rls = RGEF2TLSEstimator(init_state, init_P, estimator_params['RGEF2TLS_params'])
        elif(estimator_params['rls_type'] == 'RGDF1TLS'):
            init_P = 1 * np.identity(init_state.shape[0]+1)
            self._rls = RGDF1TLSEstimator(init_state, init_P, estimator_params['RGDF1TLS_params'])
        elif(estimator_params['rls_type'] == 'RGDF2TLS'):
            init_P = 1 * np.identity(init_state.shape[0]+1)
            self._rls = RGDF2TLSEstimator(init_state, init_P, estimator_params['RGDF2TLS_params'])
        elif(estimator_params['rls_type'] == 'RAWRGTLS'):
            init_P = 1 * np.identity(init_state.shape[0]+1)
            self._rls = RAWRGTLSEstimator(init_state, init_P, estimator_params['RAWRGTLS_params'])        
        elif(estimator_params['rls_type'] == 'MFRAWRLS'):
            self._rls = MFRAWRLSEstimator(init_state, init_P, estimator_params['MFRAWRLS_params'])
        else:
            self._rls = RLSEstimator(init_state, init_P, estimator_params['RLS_params'])
        

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

        y = self._traction - rotation_equivalent_mass * self._acc
        g = self._vehicle_params['gravity_acceleration']
        phi = np.array([[self._v**2, 
                         g * math.cos(self._pitch), 
                         (self._acc - g * math.sin(self._pitch))]])
        
        self._noise_covariance = self._noise_covariance_estimator.update([phi[0,0], phi[0,1], phi[0,2], y])
        self._rls.set_noise_covariance(self._noise_covariance)

        # phi = np.dot(phi, self._scale_T)
        self._phi = phi
        self._rls.set_phi(phi)
        self._rls.update(y)
        state = self._rls.get_state()
        # state = np.dot(self._scale_T, state)
        self._err = self._rls.get_err()
        self._P = self._rls.get_state_covariance()
        self._K = self._rls.get_K()
        
        self._drag_coefficient_estimated = max(min(state[0,0],self._drag_coefficient_bound),0)
        self._rolling_coefficient_estimated = max(min(state[1,0]/state[2,0],self._rolling_coefficient_bound),0)
        self._vehicle_mass_estimated = max(min(state[2,0],self._vehicle_mass_high_bound),self._vehicle_mass_low_bound)

        # print('_vehicle_mass_estimated:{}\n_drag_coefficient_estimated:{}\n_rolling_coefficient_estimated:{}'.format(self._vehicle_mass_estimated, self._drag_coefficient_estimated, self._rolling_coefficient_estimated))

