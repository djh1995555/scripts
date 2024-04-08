#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class MFRAWRLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(MFRAWRLSEstimator, self).__init__(init_state, init_P, config)

    def update(self, y):
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat

        self._Pd = np.identity(self._state.shape[0])
        self._Pd[0,0] = 0.001
        self._Pd[1,1] = 0.0001
        self._Pd[2,2] = 0.01
        kesi = self._config['kesi']
        if(abs(self._err) < kesi):
            w = 1
        else:
            w = kesi / abs(self._err)

        state_num = len(self._forget_factor_list)
        P_sum = 0.0
        Pd_sum = 0.0
        
        for i in range(state_num):
            for j in range(state_num):
                P_sum += self._P[i,j] * self._phi[0,i] * self._phi[0,j] / math.sqrt(self._forget_factor_list[i] * self._forget_factor_list[j])
                Pd_sum += self._Pd[i,j] * self._phi[0,i] * self._phi[0,j] / math.sqrt(self._forget_factor_list[i] * self._forget_factor_list[j])

        self._K = np.zeros((state_num,1))
        
        for i in range(state_num):
            self._K[i,0] = w * np.dot(self._P[i,:], self._phi.T) / self._forget_factor_list[i] / (1 + w*P_sum)
        self._state = self._state + self._K * self._err

        Q = np.zeros_like(self._P)
        phi_Pd = np.dot(self._phi, self._Pd)
        for i in range(state_num):
            Q[i,:] = w * np.linalg.multi_dot([self._Pd[i,:], self._phi.T, phi_Pd]) / self._forget_factor_list[i] / (1 + w*Pd_sum)
        
        Q_P = Q / self._P
        current_P = np.identity(state_num)
        I = np.identity(state_num)
        for i in range(state_num):
            for j in range(state_num): 
                current_P[i,j] = (I[i,j] - self._K[i,0] * self._phi[0,j]) / self._forget_factor_list[i]
        self._P = np.dot(current_P, self._P)

        # print('self.phi:{}'.format(self._phi))
        # print('self.K_num:{}'.format(self._K * (1+sum)))
        # print('self.K_dem:{}'.format(sum))

        # print('self.self._K:{}'.format(self._K))
        # print('self.P:{}'.format(self._P))
        # print('===================================')