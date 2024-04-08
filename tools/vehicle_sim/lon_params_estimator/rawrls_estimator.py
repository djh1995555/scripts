#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *
from base_rls import BaseRLSEstimator

class RAWRLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P, config):
        super(RAWRLSEstimator, self).__init__(init_state, init_P, config)

    
    def update(self, y):
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat

        w = 1    
        self._Pd = 0.001 * np.identity(self._state.shape[0])

        # self._Pd[0,0] = 0.001
        # self._Pd[1,1] = 0.0001
        # self._Pd[2,2] = 0.01
        kesi = self._config['kesi']
        if(abs(self._err) < kesi):
            w = 1
        else:
            w = kesi / abs(self._err)

        # sigma = 100
        # v = 10
        # w = 2 / (v * sigma*2 + self._err**2) 

        # print('w:{}'.format(w))
        self._K = w * np.dot(self._P, self._phi.T) / (self._forget_factor + w * np.linalg.multi_dot([self._phi, self._P, self._phi.T]))
        self._state = self._state + self._K * self._err
        Q = w * np.linalg.multi_dot([self._Pd, self._phi.T, self._phi, self._Pd]) / (self._forget_factor + w * np.linalg.multi_dot([self._phi, self._Pd, self._phi.T]))
        # print('Q:{}'.format(Q))
        self._P = (self._P -  np.linalg.multi_dot([self._K, self._phi, self._P]) + Q) / self._forget_factor