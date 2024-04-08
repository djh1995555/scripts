#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class RLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(RLSEstimator, self).__init__(init_state, init_P, config)
    
    def update(self, y):
        
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat
        
        self._K = np.dot(self._P, self._phi.T) / (self._forget_factor + np.linalg.multi_dot([self._phi, self._P, self._phi.T]))
        self._P = (self._P - np.linalg.multi_dot([self._K, self._phi, self._P])) / self._forget_factor
        self._state = self._state + self._K * self._err

        
        # self._P = self._P - (np.linalg.multi_dot([self._P, self._phi.T, self._phi, self._P])) / (self._forget_factor + np.linalg.multi_dot([self._phi, self._P, self._phi.T]))
        # self._K = np.dot(self._P, self._phi.T) / self._forget_factor
        # self._state = self._state + self._K * self._err

        
        # print('self.phi:{}'.format(self._phi))
        # print('self.K_num:{}'.format(np.dot(self._P, self._phi.T)/self._forget_factor))
        # print('self.K_dem:{}'.format(np.linalg.multi_dot([self._phi, self._P, self._phi.T])/self._forget_factor))
        # print('self.self._K:{}'.format(self._K))
        # print('self.P:{}'.format(self._P))
        # print('self._state:{}'.format(self._state))
        # print('===================================')