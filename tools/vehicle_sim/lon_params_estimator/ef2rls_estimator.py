#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class EF2RLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(EF2RLSEstimator, self).__init__(init_state, init_P, config)
    
    def update(self, y):
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat
        
        self._K = np.dot(self._P, self._phi.T)
        self._state = self._state + self._K * self._err

        delta = 0.01
        F = self._forget_factor * np.identity(self._state_num) + delta * self._P
        last_P = np.dot(self._P, np.linalg.inv(F))
        self._P = last_P - np.linalg.multi_dot([last_P, self._phi.T, self._phi, last_P]) / (1 + np.linalg.multi_dot([self._phi, last_P, self._phi.T]))

        # print('_phi:{}'.format(self._phi))
        # print('P:{}'.format(self._P))
        # print('=======================')