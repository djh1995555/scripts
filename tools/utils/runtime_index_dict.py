#!/usr/bin/env python
from collections import OrderedDict 
from topics_and_signals import *
COLOR_LIST = ['r-','g-','b-','c-','k-','y-','m-','orange','indigo','gray','purple','pink','brown','teal', 'turquoise', 'tan','aqua', 'lavender','violet']

RUNTIME_INDEX_DICT = OrderedDict()
RUNTIME_INDEX_DICT['throttle'] = [
                            # THROTTLE_GSP0,
                            # THROTTLE_GSP1,
                            # THROTTLE_GSP2,
                            # THROTTLE_GSP_WEIGHTED,
                            # THROTTLE_FROM_CONTROL,
                            THROTTLE_CMD,
                            THROTTLE_INPUT,
                            THROTTLE_OUTPUT,
                            THROTTLE_IDLE,
                            COASTING_FLAG,
                            ]

RUNTIME_INDEX_DICT['coasting acc estimation'] = [
                            COASTING_ACC_EST,
                            COASTING_ACC,
                            ACC_REPORT,
                            ACC_DIFFERENTIAL,
                        ]
RUNTIME_INDEX_DICT['StateCovariance'] = [
                            LON_ESTIMATION_P0, 
                            LON_ESTIMATION_P1,
                            LON_ESTIMATION_P2,
                            ]
RUNTIME_INDEX_DICT['weight'] = [TOTAL_WEIGHT,
                            TOTAL_WEIGHT_ESTIMATE,
                            WEIGHT_ESTIMATED, 
                            V_ESTIMATED,
                            ]
RUNTIME_INDEX_DICT['Ca'] = [Ca_ESTIMATED
                            ]
RUNTIME_INDEX_DICT['Cr'] = [Cr_ESTIMATED,
                            ]
RUNTIME_INDEX_DICT['velocity'] = [
                            # V_GSP0,
                            # V_GSP1,
                            # V_GSP2,
                            # V_ERROR,
                            # V_TARGER_FINAL,
                            # V_TARGET,
                            V_CURRENT_DBW,
                            V_ESTIMATED,
                            # V_IDLE,
                            # V_LEAD,
                            # KALMAN_V,
                            
                            # CRUISE_SPEED,
                            # SPEED_LIMIT,
                            # SPEED_LIMIT_FROM_FE,
                            # TARGET_SPEED_SOURCE,
                            # SPEED_LIMIT_FROM_CURVATURE,
                            # SPEED_LIMIT_FROM_MAP
                            ]                            
RUNTIME_INDEX_DICT['acceleration'] = [
                                # ACC_GSP0,
                                # ACC_GSP1,
                                # ACC_GSP2,
                                ACC_PLANNED,
                                ACC_TARGET_MPC,
                                ACC_TARGET_WITH_MARC,
                                ACC_TARGET_WITH_COMPENSATION,
                                ACC_TARGET,
                                ACC_REPORT,
                                A_IMU,
                                A_CHASSIS,
                                A_RAW_IMU,
                                A_FILTERED_IMU_1,
                                A_JERK_IMU_1,
                                A_FILTERED_IMU_2,
                                A_JERK_IMU_2,
                                REF_ACC_IN_PLANNING,
                                MAX_ACC,
                                # AZ_1,
                                # AZ_JERK_1,
                                # AZ_2,
                                # AZ_JERK_2,
                                # CMD_CONSTRAINTS,
                                ACC_DIFFERENTIAL,
                                ACC_PITCH,
                                # KALMAN_ACC,
                                # COASTING_FLAG,
                                ACC_LEVEL,
                                # DBW_ENABLED,
                                # COASTING_ACC,
                                ]
RUNTIME_INDEX_DICT['brake'] = [
                        # BRAKE_GSP0,
                        # BRAKE_GSP1,
                        # BRAKE_GSP2,
                        BRAKE_CMD,
                        BRAKE_INPUT,
                        BRAKE_OUTPUT,
                        ]
RUNTIME_INDEX_DICT['elevation'] = [ELEVATION]
RUNTIME_INDEX_DICT['pitch'] = [PITCH_MAP,
                        PITCH_CONTROL,
                        PITCH_SMOOTHED_POSE,
                        PITCH_POSE,
                        PITCH_Z,
                        ]
RUNTIME_INDEX_DICT['resistance param estimation'] = [
                            DISTURBANCE_ESTIMATED, 
                        ]

RUNTIME_INDEX_DICT['coasting acc estimation'] = [
                            COASTING_ACC_FORMULA,
                            COASTING_ACC_EST,
                            ACC_REPORT,
                        ]
RUNTIME_INDEX_DICT['weight'] = [TOTAL_WEIGHT,
                            TOTAL_WEIGHT_ESTIMATE,
                            WEIGHT_ESTIMATED, 
                            ]
RUNTIME_INDEX_DICT['lead info'] = [ LEAD_DISTANCE ]
RUNTIME_INDEX_DICT['gear'] = [GEAR_CMD,
                        GEAR_REPORT,
                        GEAR_CMD_CONTROL,
                        ]

RUNTIME_INDEX_DICT['fuel'] = [ENGINE_FUEL_RATE,
                        TOTAL_FUEL_USED,
                        ]

                                    

RUNTIME_INDEX_DICT['mrac throttle'] = [REF_MODEL_THROTTLE,
                                KC_THROTTLE,
                                KS_THROTTLE,
                                THETA_THROTTLE,
                                U_REFERENCE_THROTTLE,
                                U_STATE_THROTTLE,
                                U_DISTURBANCE_THROTTLE,
                                FEEDBACK_CMD_THROTTLE,
                                THROTTLE_ERR
                                ]

RUNTIME_INDEX_DICT['mrac brake'] = [REF_MODEL_BRAKE,
                                KC_BRAKE,
                                KS_BRAKE,
                                THETA_BRAKE,
                                U_REFERENCE_BRAKE,
                                U_STATE_BRAKE,
                                U_DISTURBANCE_BRAKE,
                                FEEDBACK_CMD_BRAKE,
                                BRAKE_ERR
                                ]



RUNTIME_INDEX_DICT['flag'] = [ DBW_ENABLED,
                        AEB_ENABLED,
                        ABS_ACTIVE,
                        ABS_ENABLED,
                        STAB_ACTIVE,
                        STAB_ENABLED,
                        ACTION_TYPE,
                        ]

RUNTIME_INDEX_DICT['powertrain'] = [
                            CLUTCH_SLIP,
                            ENGINE_RPM,
                            ]  
RUNTIME_INDEX_DICT['lateral info'] = [LAT_ERROR,
                        HEADING_ERROR,
                        CURVATURE,
                        YAW_RATE
                        ]  
RUNTIME_INDEX_DICT['heading info'] = [
                            HEADING_ANGLE,
                            HEADING_REF,
                            ]  
RUNTIME_INDEX_DICT['steering'] = [
                            STEERING_ANGLE
                            ]  