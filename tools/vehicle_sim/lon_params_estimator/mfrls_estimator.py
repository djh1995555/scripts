#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class MFRLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(MFRLSEstimator, self).__init__(init_state, init_P, config)

    def update(self, y):
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat

        state_num = len(self._forget_factor_list)
        sum = 0.0
        
        for i in range(state_num):
            for j in range(state_num):
                sum += self._P[i,j] * self._phi[0,i] * self._phi[0,j] / math.sqrt(self._forget_factor_list[i] * self._forget_factor_list[j])

        self._K = np.zeros((state_num,1))
        for i in range(state_num):
            self._K[i,0] = np.dot(self._P[i,:], self._phi.T) / self._forget_factor_list[i] / (1 + sum)

        self._state = self._state + self._K * self._err
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