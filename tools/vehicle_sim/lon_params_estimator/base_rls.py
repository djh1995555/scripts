#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.signals import *

class BaseRLSEstimator(object):
    def __init__(self, init_state, init_P, config):
        self._state = init_state
        self._state_num = init_state.shape[0]
        self._P = init_P
        self._P_inv = np.linalg.inv(init_P)
        self._config = config
        forget_factor = config['forget_factor']
        if(len(forget_factor) == 1):
            self._forget_factor = forget_factor[0]
        else:
            self._forget_factor_list = forget_factor

    def set_noise_covariance(self, noise_covariance):
        self._noise_covariance = noise_covariance
        
    def set_phi(self, phi):
        self._phi = phi
    
    def get_state(self):
        return self._state

    def get_err(self):
        return self._err
    
    def update(self, y):
        pass

    def get_state_covariance(self):
        return self._P

    def get_K(self):
        return self._K