#!/usr/bin/env python
import os
import shutil
import numpy as np
import math
import queue

class PolynomialKalmanSmoother(object):
    def __init__(self, poly_order, wl, wr, forget_factor):
        self._state = np.zeros((poly_order+1,1))
        self._P = np.identity(poly_order+1)
        self._forget_factor = forget_factor
        self._smoothed_y = []
        self._delayed_y = []
        self._delay_frames = wr
        self._y = queue.Queue()
        self._A = np.array([
            [1,1,1,1,1],
            [0,1,2,3,4],
            [0,0,1,3,6],
            [0,0,0,1,4],
            [0,0,0,0,1]
            ])
        c_element = wl + 1 + wr
        smoothed_vector_element = wl + 1

        self._C = np.zeros((1,poly_order+1))
        self._smoothed_vector = np.zeros((1,poly_order+1))

        for i in range(poly_order+1):
            self._C[0,i] = c_element ** i
            self._smoothed_vector[0,i] = smoothed_vector_element ** i

    def update_kalman(self, y):
        predict_state = np.dot(self._A, self._state)
        predict_P = np.linalg.multi_dot([self._A, self._P, self._A.T])
        err = y - np.dot(self._C,self._state)
        K = np.dot(predict_P, self._C.T) / (self._forget_factor + np.linalg.multi_dot([self._C, predict_P, self._C.T]))
        self._P = (predict_P - np.linalg.multi_dot([K, self._C, predict_P])) / self._forget_factor
        self._state = predict_state + K * err

    def compute_noise_estimated(self, y):
        self._y.put(y)
        self.update_kalman(y)
        if(self._y.qsize() <= int(self._delay_frames)):
            return 0
        smoothed_y = np.dot(self._smoothed_vector, self._state)
        self._smoothed_y.append(smoothed_y[0,0])
        delayed_y = self._y.get()
        self._delayed_y.append(delayed_y)
        return delayed_y - smoothed_y

    def get_smoothed_y(self):
        return self._smoothed_y

    def get_delayed_y(self):
        return self._delayed_y    

class NoiseCovarianceEstimator(object):
    def __init__(self, noise_num, poly_order, wl, wr, forget_factor):
        self._pks_list = []
        for i in range(noise_num):
            self._pks_list.append(PolynomialKalmanSmoother(poly_order, wl, wr, forget_factor))
        
        self._noise_P = 0.3 * np.identity(noise_num)
        self._forget_factor = forget_factor

    def update(self, noise_list):
        z = np.zeros((1, len(noise_list)))
        
        for i in range(len(self._pks_list)):
            z[0,i] = self._pks_list[i].compute_noise_estimated(noise_list[i])

        self._noise_P = self._forget_factor * self._noise_P + (1-self._forget_factor) * np.dot(z.T,z)
        return self._noise_P
    
    def get_kalman_smoothed_y(self):
        smoothed_y_list = []
        for i in range(len(self._pks_list)):
            smoothed_y_list.append(self._pks_list[i].get_smoothed_y())
        return smoothed_y_list

    def get_kalman_delayed_y(self):
        delayed_y_list = []
        for i in range(len(self._pks_list)):
            delayed_y_list.append(self._pks_list[i].get_delayed_y())
        return delayed_y_list
