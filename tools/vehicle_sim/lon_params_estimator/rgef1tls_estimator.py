#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class RGEF1TLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(RGEF1TLSEstimator, self).__init__(init_state, init_P, config)
        self._signal_smoother_list = []
    
    def update(self, y):
        state_num = self._state.shape[0]
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat

        z = np.append(self._phi, y)
        z = z.T
        z = z.reshape(state_num+1,1)

        # self._K = np.dot(self._P, z) / (self._forget_factor + np.linalg.multi_dot([z.T, self._P, z]))
        # self._P = (self._P - np.linalg.multi_dot([self._K, z.T, self._P])) / self._forget_factor
        self._K = np.dot(self._P, z) / (self._forget_factor + np.linalg.multi_dot([z.T, self._P, z]))


        F = self._forget_factor * np.identity(self._state_num+1)
        last_P = np.dot(self._P, np.linalg.inv(F))
        self._P = last_P - np.linalg.multi_dot([last_P, z, z.T, last_P]) / (1 + np.linalg.multi_dot([z.T, last_P, z]))

        V = np.append(self._state.T, -1)
        V = V.T
        V = V.reshape(state_num+1,1)
        P_noise = np.identity(state_num + 1)
        if(self._config['use_noise_covariance_estimated']):
            P_noise = self._noise_covariance
        V = np.linalg.multi_dot([self._P, P_noise, V])
        self._state = -V[0:state_num] / V[-1]

        # print('self.phi:{}'.format(self._phi))
        # print('self.K_num:{}'.format(np.dot(self._P, self._phi.T)/self._forget_factor))
        # print('self.K_dem:{}'.format(np.linalg.multi_dot([self._phi, self._P, self._phi.T])/self._forget_factor))
        # print('self.self._K:{}'.format(self._K))
        # print('self.P:{}'.format(self._P))
        # print('self._state:{}'.format(self._state))
        # print('===================================')