#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class RAWRGTLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(RAWRGTLSEstimator, self).__init__(init_state, init_P, config)

    def update(self, y):
        print('=======RAWRGTLS=========')
        state_num = self._state.shape[0]
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat

        w = 1    
        self._Pd = np.identity(state_num+1)

        pd_covariance = 10e-12
        self._Pd[0,0] = pd_covariance
        self._Pd[1,1] = pd_covariance
        self._Pd[2,2] = pd_covariance
        self._Pd[3,3] = pd_covariance
        kesi = self._config['kesi']
        if(abs(self._err) < kesi):
            w = 1
        else:
            w = kesi / abs(self._err)

        # sigma = 100
        # v = 10
        # w = 2 / (v * sigma*2 + self._err**2)  

        w = 1

        z = np.append(self._phi, y)
        z = z.T
        z = z.reshape(state_num+1,1)
        self._K = w * np.dot(self._P, z) / (self._forget_factor + w * np.linalg.multi_dot([z.T, self._P, z]))
        Q = w * np.linalg.multi_dot([self._Pd, z, z.T, self._Pd]) / (self._forget_factor + w * np.linalg.multi_dot([z.T, self._Pd, z]))
        self._P = (self._P - np.linalg.multi_dot([self._K, z.T, self._P]) + Q) / self._forget_factor
        # self._P = (self._P - np.linalg.multi_dot([self._K, z.T, self._P])) / self._forget_factor

        V = np.append(self._state.T, -1)
        V = V.T
        V = V.reshape(state_num+1,1)
        P_noise = 100 * np.identity(state_num + 1)
        V = np.linalg.multi_dot([self._P, P_noise, V])
        self._state = -V[0:state_num] / V[-1]
        print('==================================')