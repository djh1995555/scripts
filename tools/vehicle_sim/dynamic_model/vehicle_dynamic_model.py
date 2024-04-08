#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import queue
from utils.signals import *

class NoiseGenerator(object):
    def __init__(self, deviation):
        self._length = 1000
        self._noise = np.random.normal(0, deviation, size=(self._length,))
        self._idx = 0

    def get_noise(self):
        current_noise = self._noise[self._idx%self._length]
        self._idx += 1
        return current_noise
    
class VehicleDynamicModel(object):
    def __init__(self, config, init_pitch = 0.0, generate_mode = False):
        self._config = config
        self._generate_mode = generate_mode
        self.reset(config['init_v'], init_pitch)
        self._acc_noise = NoiseGenerator(self._config['acc_noise_deviation'])
        self._pitch_noise = NoiseGenerator(self._config['pitch_noise_deviation'])
        self._traction_noise = NoiseGenerator(self._config['traction_noise_deviation'])
        self._v_noise = NoiseGenerator(self._config['v_noise_deviation'])
        
    def reset(self, init_v = 0.0, init_pitch = 0.0):
        self._v = init_v
        self._traction = 0.0
        self._pitch = init_pitch
        self._acc_history = queue.Queue()
        self._acc_signal = 0.0
    
    def get_state(self):
        if(self._config['with_noise'] and not self._generate_mode):
            traction_measurement = self._traction + self._traction_noise.get_noise()
            v_measurement = self._v + self._v_noise.get_noise()
            acc_measurement = self._acc_signal + self._acc_noise.get_noise()
            picth_measurement = self._pitch + self._pitch_noise.get_noise()
            yaw_rate_measurement = 0.0
            steering_angle_measurement = 0.0
        else:
            traction_measurement = self._traction
            v_measurement = self._v
            acc_measurement = self._acc_signal
            picth_measurement = self._pitch
            yaw_rate_measurement = 0.0
            steering_angle_measurement = 0.0
        data = {
            TRACTION: traction_measurement,
            V: v_measurement,
            A_REPORT: acc_measurement,
            PITCH: picth_measurement,
            YAW_RATE: yaw_rate_measurement,
            STEERING_ANGLE: steering_angle_measurement,
        }
        return data
    
    def update(self, traction, pitch, vehicle_mass = None, drag_coefficient = None, rolling_friction_coefficient = None):
        self._traction = traction
        self._pitch = pitch
        self._gear_ratio = 1.0
        acc = self.compute_acc(self._config['traction_ratio'] * self._traction, 
                               self._gear_ratio, 
                               self._config['v_ratio'] * self._v, 
                               self._pitch,
                               vehicle_mass,
                               drag_coefficient,
                               rolling_friction_coefficient
                               )
        self._acc_history.put(acc)
        delay_frames = self._config['delay_time']/self._config['sample_time']
        if(self._acc_history.qsize() >= int(delay_frames)):
            self._acc_signal = self._acc_history.get()
        self._v += acc *  self._config['sample_time']
        if(self._v < 0.0):
            self._v = 0.0  

    def compute_component(self, gear_ratio, v, pitch, vehicle_mass = None, drag_coefficient = None, rolling_friction_coefficient = None):
        if(vehicle_mass == None):
            self._vehicle_mass = self._config['vehicle_mass']
        else:
            self._vehicle_mass = vehicle_mass

        if(drag_coefficient == None):
            self._drag_coefficient = self._config['drag_coefficient']
        else:
            self._drag_coefficient = drag_coefficient

        if(rolling_friction_coefficient == None):
            self._rolling_friction_coefficient = self._config['rolling_friction_coefficient']
        else:
            self._rolling_friction_coefficient = rolling_friction_coefficient

        self._overall_ratio = self.compute_overall_ratio(gear_ratio)
        self._M_eq = self.compute_equivalent_mass(self._vehicle_mass, self._overall_ratio)
        self._F_aero = self.compute_aero_drag(v)
        self._F_roll = self.compute_rolling_resis(self._vehicle_mass, pitch, v)
        self._F_grav = self.compute_gravity_load(self._vehicle_mass, pitch)

    def compute_traction(self, target_acc, gear_ratio, v, pitch, vehicle_mass = None, drag_coefficient = None, rolling_friction_coefficient = None):
        self.compute_component(gear_ratio, v, pitch, vehicle_mass, drag_coefficient, rolling_friction_coefficient)
        return self._M_eq * target_acc + self._F_aero + self._F_roll + self._F_grav
    
    def compute_acc(self, F_traction, gear_ratio, v, pitch, vehicle_mass = None, drag_coefficient = None, rolling_friction_coefficient = None):
        self.compute_component(gear_ratio, v, pitch, vehicle_mass, drag_coefficient, rolling_friction_coefficient)
        return (F_traction - self._F_aero - self._F_roll + self._F_grav) / self._M_eq
    
    def compute_overall_ratio(self, gear_ratio):
        overall_ratio = gear_ratio * self._config['axle_drive_ratio']
        if(type(overall_ratio) != float):
            overall_ratio = overall_ratio.fillna(0)
        return overall_ratio

    def compute_equivalent_mass(self, vehicle_mass, overall_ratio):
        rotation_equivalent_mass = (self._config['inertia_wheels'] + self._config['inertia_engine'] * self._config['transmission_efficiency'] * overall_ratio**2) / (self._config['effective_tire_radius']**2)
        return vehicle_mass + rotation_equivalent_mass
    
    def compute_aero_drag(self, v):
        v_wind = self._config['wind_speed']
        return self._drag_coefficient * (v-v_wind) ** 2

    def compute_rolling_resis(self, vehicle_mass, pitch, v = 0.0):
        return self._rolling_friction_coefficient * vehicle_mass * self._config['gravity_acceleration'] * math.cos(pitch)
        # return (self._rolling_friction_coefficient +  0.0001 * v) * vehicle_mass * self._config['gravity_acceleration'] * math.cos(pitch)
    def compute_gravity_load(self, vehicle_mass, pitch):
        return vehicle_mass * self._config['gravity_acceleration'] * math.sin(pitch)
