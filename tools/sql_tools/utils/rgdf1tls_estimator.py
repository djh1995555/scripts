#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from base_rls import BaseRLSEstimator

class RGDF1TLSEstimator(BaseRLSEstimator):
    def __init__(self, init_state, init_P):
        super(RGDF1TLSEstimator, self).__init__(init_state, init_P)
        self._signal_smoother_list = []
    
    def update(self, y):
        state_num = self._state.shape[0]
        y_hat = np.dot(self._phi, self._state)
        self._err = y - y_hat

        z = np.append(self._phi, y)
        z = z.T
        z = z.reshape(state_num+1,1)

        self._K = np.dot(self._P, z) / (1 + np.linalg.multi_dot([z.T, self._P, z]))
        beta = self._forget_factor - (1 - self._forget_factor) / np.linalg.multi_dot([z.T, self._P, z])
        F = np.identity(self._state_num+1) - (1 - beta) * np.linalg.multi_dot([z, z.T, self._P])
        last_P = np.dot(self._P, np.linalg.inv(F))
        self._P = last_P - np.linalg.multi_dot([last_P, z, z.T, last_P]) / (1 + np.linalg.multi_dot([z.T, last_P, z]))

        V = np.append(self._state.T, -1)
        V = V.T
        V = V.reshape(state_num+1,1)
        P_noise = np.identity(state_num + 1)
        V = np.linalg.multi_dot([self._P, P_noise, V])
        self._state = -V[0:state_num] / V[-1]