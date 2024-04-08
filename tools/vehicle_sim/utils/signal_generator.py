#!/usr/bin/env python
import numpy as np
import pandas as pd
import math
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel
from utils.signals import *

class SignalGenerator():
    def __init__(self, simulation_time, sample_time, init_v, ref_length, vehicle_params):
        
        self._ref_length = ref_length
        self._ref_v = []
        self._pitch = []
        self._ca = []
        self._cr = []
        self._total_length = int(simulation_time / sample_time)
        t = [sample_time * i for i in range(self._total_length)]

        self._ca = vehicle_params['ca_magnitude'] * np.sin(np.dot(2 * math.pi / 1000, t) - math.pi/4) + vehicle_params['drag_coefficient']
        self._cr = vehicle_params['cr_magnitude'] * np.sin(np.dot(2 * math.pi / 5000, t) - math.pi*3/4) + vehicle_params['rolling_friction_coefficient']

        if(vehicle_params['const_ref_v']):
            ref_v = [init_v] * self._total_length
            self._ref_v = np.array(ref_v)   
        else:
            self._ref_v = 6 * np.sin(np.dot(2 * math.pi / 100, t)) + 2 * np.sin(np.dot(2 * math.pi / 160, t)) + init_v

        if(vehicle_params['no_ramp']):
            self._pitch = np.array([0] * self._total_length)   
        else:
            self._pitch = -(0.4 * np.sin(np.dot(2 * math.pi / 100, t) - math.pi/4) + \
                                    0.1 * np.sin(np.dot(2 * math.pi / 200, t) - math.pi/4)) / 20
    def get_ref_v(self, i):
        return self._ref_v[i : min(i + self._ref_length, self._total_length)]

    def get_pitch(self, i):
        return self._pitch[i]
    
    def get_ca(self, i):
        return self._ca[i]
    
    def get_cr(self, i):
        return self._cr[i]
    
    def get_computed_traction(self,i):
        return self._computed_traction[i]