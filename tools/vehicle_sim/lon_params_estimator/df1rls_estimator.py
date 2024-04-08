#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class DF1RLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(DF1RLSEstimator, self).__init__(init_state, init_P, config)
        self._r = config['r']
        # self._u = config['u']
        # self._g = config['g']
    
    def update(self, y):
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat
        
        self._K = np.dot(self._P, self._phi.T) / (1 + np.linalg.multi_dot([self._phi, self._P, self._phi.T]))
        self._state = self._state + self._K * self._err

        beta = self._forget_factor - (1 - self._forget_factor) / np.linalg.multi_dot([self._phi, self._P, self._phi.T])
        F = np.identity(self._state_num) - (1 - beta) * np.linalg.multi_dot([self._phi.T, self._phi, self._P])
        last_P = np.dot(self._P, np.linalg.inv(F))
        self._P = last_P - np.linalg.multi_dot([last_P, self._phi.T, self._phi, last_P]) / (1 + np.linalg.multi_dot([self._phi, last_P, self._phi.T]))

        # print('_phi:{}'.format(self._phi))
        # print('P:{}'.format(self._P))
        # print('=======================')