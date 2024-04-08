#!/usr/bin/env python
TRACTION = 'traction'
A_REPORT = 'a_report'
PITCH = 'pitch_pose'
YAW_RATE = 'yaw_rate'
STEERING_ANGLE = 'steering_angle'
V = 'v_current_dbw'
BENCHMARK = 'benchmark'

THROTTLE = 'throttle'
GEAR_RATIO = 'gear_ratio'
FRICTION = 'friction'
MASS = 'mass'
DRAG_COEFFICIENT = 'drag_coefficient'
ROLLING_COEFFICIENT = 'rolling_coefficient'
RAW_INPUT = [
    THROTTLE,
    GEAR_RATIO,
    FRICTION,
    MASS,
    PITCH,
    DRAG_COEFFICIENT,
    ROLLING_COEFFICIENT
]

OUTPUT_DATA = [
    A_REPORT,
    TRACTION,
    PITCH,
    V,
    YAW_RATE,
    STEERING_ANGLE,
    BENCHMARK
]