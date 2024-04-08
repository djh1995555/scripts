#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class DF2RLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(DF2RLSEstimator, self).__init__(init_state, init_P, config)
    
    def update(self, y):
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat
        
        self._K = np.dot(self._P, self._phi.T)
        self._state = self._state + self._K * self._err

        P_inv = np.linalg.inv(self._P)
        kesi = 0.01
        max_phi = np.max(np.abs(self._phi))
        if(max_phi<kesi):
            last_P = self._P
        else:
            last_P = self._P +  (1 - self._forget_factor) / self._forget_factor / np.linalg.multi_dot([self._phi, P_inv, self._phi.T]) * np.dot(self._phi.T, self._phi)
            # last_P = np.identity(self._state_num) -  (1 - self._forget_factor) / self._forget_factor * np.linalg.multi_dot([P_inv, self._phi.T, self._phi]) / np.linalg.multi_dot([self._phi, P_inv, self._phi.T]) 
        
        self._P = last_P - np.linalg.multi_dot([last_P, self._phi.T, self._phi, last_P]) / (1 + np.linalg.multi_dot([self._phi, last_P, self._phi.T]))

        # print('_phi:{}'.format(self._phi))
        # print('P:{}'.format(self._P))
        # print('=======================')