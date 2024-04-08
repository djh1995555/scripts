#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import math

friction_torque = 0.0
throttle = 0.0

v = 85/3.6
pitch = 1.1
acc = 0.08

pitch = pitch / 57.3
aero_coeff = 0.68
rolling_coeff = 0.006
air_density = 1.225
g = 9.8

axle_drive_ratio= 2.714
max_throttle_engine_torque= 3125
transmission_efficiency= 0.95
effective_tire_radius= 0.5
inertia_wheels= 60.0
inertia_engine= 4.0
vehicle_frontal_area= 10.0
rotation_equivalent_mass = (inertia_wheels + inertia_engine * transmission_efficiency * axle_drive_ratio**2) / (effective_tire_radius**2)

fdrive = ((throttle - friction_torque) * max_throttle_engine_torque  * transmission_efficiency / effective_tire_radius)
a = fdrive - rotation_equivalent_mass * acc - 0.5 * aero_coeff * air_density * vehicle_frontal_area * v ** 2
b = acc + g * math.cos(pitch) * rolling_coeff - g * math.sin(pitch)
weight = a/b

print('fdrive:{}'.format(fdrive))
print('mra:{}'.format(rotation_equivalent_mass * acc))
print('aero_drag:{}'.format(0.5 * aero_coeff * air_density * vehicle_frontal_area * v ** 2))
print('a:{}'.format(a))
print('b:{}'.format(b))
print('weight:{}'.format(weight))
