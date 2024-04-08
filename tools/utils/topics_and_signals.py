#!/usr/bin/env python
# throttle dict
THROTTLE_GSP0 = 'throttle_gsp_0'
THROTTLE_GSP1 = 'throttle_gsp_1'
THROTTLE_GSP2 = 'throttle_gsp_2'
THROTTLE_GSP_WEIGHTED = 'throttle_gsp_weighted'
THROTTLE_FROM_CONTROL = 'throttle_from_control'
THROTTLE_CMD = 'throttle_cmd'
THROTTLE_INPUT = 'throttle_input'
THROTTLE_OUTPUT = 'throttle_output'
THROTTLE_IDLE = 'throttle_idle'
GSP_THROTTLE = 'gsp_throttle'
COASTING_FLAG = 'coasting_flag'

# brake dict
BRAKE_GSP0 = 'brake_gsp_0'
BRAKE_GSP1 = 'brake_gsp_1'
BRAKE_GSP2 = 'brake_gsp_2'
BRAKE_CMD = 'brake_cmd'
BRAKE_INPUT = 'brake_input'
BRAKE_OUTPUT = 'brake_output'
ENGINE_BRAKE = 'engine_brake'
RETARDER = 'retarder'

# acc dict
ACC_GSP0 = 'a_gsp_0'
ACC_GSP1 = 'a_gsp_1'
ACC_GSP2 = 'a_gsp_2'
ACC_PLANNED = 'a_planned'
ACC_TARGET_MPC = 'a_target_mpc'
ACC_TARGET_WITH_MARC = 'a_taregt_with_mrac'
ACC_TARGET_WITH_COMPENSATION = 'a_taregt_with_compensation'
ACC_TARGET = 'a_target'
ACC_REPORT = 'a_report'
ACC_FROM_IMU_FILTERED = 'acc_from_imu_filtered'
ACC_FROM_KALMAN_DIFFERENTIATOR = 'acc_from_kalman_differentiator'
A_IMU = 'a_imu'
A_CHASSIS = 'a_chassis'
A_RAW_IMU = 'a_raw_imu'
A_FILTERED_IMU_1 = 'a_filtered_imu_1'
A_JERK_IMU_1 = 'a_jerk_from_imu_1'
A_FILTERED_IMU_2 = 'a_filtered_imu_2'
A_JERK_IMU_2 = 'a_jerk_from_imu_2'
ACC_PITCH = 'acc_pitch'
ACC_DIFFERENTIAL = 'acc differential'
ACC_LEVEL = 'acc_level'
AZ_RAW = 'az_raw'
AZ_1 = 'az_1'
AZ_JERK_1 = 'az_jerk_1'
AZ_2 = 'az_2'
AZ_JERK_2 = 'az_jerk_2'
CMD_CONSTRAINTS = 'cmd_constraints'
MAX_ACC = 'max_acc'
REF_ACC_IN_PLANNING = 'ref_acc_in_planning'

ACC_DELAY_COMPENSATOR_DICT = 'acc_delay_compensator_dict'
ACC_DELAY_COMPENSATOR_V = 'acc_delay_compensator_v'
ACC_DELAY_COMPENSATOR_V_ERR = 'acc_delay_compensator_v_err'
ACC_DELAY_COMPENSATOR_XI = 'acc_delay_compensator_xi'
ACC_DELAY_COMPENSATOR_U = 'acc_delay_compensator_u'

RESISTANCE = 'resistance'
ACC_COMPUTED = 'a_computed'
ACC_EST = 'acc_est'

COASTING_ACC = 'coasting_acc'
COASTING_ACC_FORMULA = 'coasting_acc_formula'
COASTING_ACC_EST = 'coasting_acc_est'
COASTING_ACC_EST_LOW_SPEED = 'coasting_acc_est_low_speed'
COASTING_ACC_EST_FLAG = 'coasting_acc_est_flag'

COASTING_ACC_V_OBSERVED = 'coasting_acc_v_observed' 
COASTING_ACC_CA_OBSERVED = 'coasting_acc_ca_observed' 

COASTING_ACC_CA_EST = 'coasting_acc_ca_estimated'
COASTING_ACC_CD_EST = 'coasting_acc_disturbance_estimated'
COASTING_ACC_CONFI = 'coasting_acc_confi'

WEIGHT_ESTIMATED = "weight_estimated"
WEIGHT_CALIBRATED = "weight_calibrated"
DISTURBANCE_ESTIMATED = "disturbance_estimated"
Cr_ESTIMATED = "cr_estimated"
Ca_ESTIMATED = "ca_estimated"
WEIGHT_ESTIMATION_COVARIANCE = "weight_estimation_covariance"
CONDITION_MET_FLAG = "condition_met_flag"
WEIGHT_ESTIMATION_VALID = "weight_estimation_valid"

V_GSP0 = 'v_gsp_0'
V_GSP1 = 'v_gsp_1'
V_GSP2 = 'v_gsp_2'
V_TARGET = 'v_target'
V_TARGER_FINAL = 'v_target_final'
V_CURRENT_WHEEL = 'v_current_wheel'
V_CURRENT_DBW = 'v_current_dbw'
V_LEAD = 'v_lead'
V_ERROR = 'v_error'
V_ESTIMATION_ERR = 'v_estimation_err'
V_ESTIMATION = 'v_estimation'
V_IDLE = 'v_idle'
V_KALMAN = 'v_kalman'
V_ESTIMATED = 'v_estimated'
CRUISE_SPEED = 'cruise_speed'
SPEED_LIMIT = 'speed_limit'
SPEED_LIMIT_FROM_FE = 'speed_limit_from_fe'
SPEED_LIMIT_FROM_CURVATURE = 'speed_limit_from_curvature'
SPEED_LIMIT_FROM_MAP = 'speed_limit_from_map'
TARGET_SPEED_SOURCE = 'taget_speed_source'

# mrac throttle
MRAC_THROTTLE_DICT = 'mrac throttle'
REF_MODEL_THROTTLE = 'ref_model_throttle'
FEEDBACK_CMD_THROTTLE = 'feedback_cmd_throttle'
KC_THROTTLE = 'kc_throttle'
KS_THROTTLE = 'ks_throttle'
THETA_THROTTLE = 'theta_throttle'
U_REFERENCE_THROTTLE = 'u_reference_throttle'
U_STATE_THROTTLE = 'u_state_throttle'
U_DISTURBANCE_THROTTLE = 'u_disturbance_throttle'
THROTTLE_ERR = 'throttle_error'

# mrac brake
MRAC_BRAKE_DICT = 'mrac brake'
REF_MODEL_BRAKE = 'ref_model_brake'
FEEDBACK_CMD_BRAKE = 'feedback_cmd_brake'
KC_BRAKE = 'kc_brake'
KS_BRAKE = 'ks_brake'
THETA_BRAKE = 'theta_brake'
U_REFERENCE_BRAKE = 'u_reference_brake'
U_STATE_BRAKE = 'u_state_brake'
U_DISTURBANCE_BRAKE = 'u_disturbance_brake'
BRAKE_ERR = 'brake_error'

# gear dict
GEAR_CMD_CONTROL = 'gear_cmd_control'
GEAR_CMD = 'gear_cmd'
GEAR_REPORT = 'gear_report'
GEAR_RATIO = 'gear_ratio'

# fuel dict 
ENGINE_FUEL_RATE = 'engine_fuel_rate'
TOTAL_FUEL_USED = 'total_fuel_used'

# elevation dict
ELEVATION = 'elevation'
Z_FORM_Z_DIFFERENTIATOR = 'z_from_z_differentiator'

# lon mpc solve status
FIRST_RUN_SOLVED = 'first_run_solved'
HOT_RUN_SOLVED = 'hot_run_solved'
OSQP_SOLVE_STATUS = 'osqp_solve_status'
SWITCH_CONTROLLER_FLAG = 'switch_controller_flag'
FALLBACK_CONTROL_CMD ='fallback_lon_controller_cmd'

# pitch dict
PITCH_CONTROL = 'pitch_control'                 # the pitch used finally
PITCH_MAP = 'pitch_map'                         # the pitch from map
PITCH_SMOOTHED_POSE = 'pitch_smoothed_pose'     # the pitch of pose from odom after filter
PITCH_POSE = 'pitch_pose'                       # the pitch of pose from odom without filter
PITCH_Z = 'pitch_z'                             # the pitch calculated from z of localization
PITCH_DELAY = 'pitch_delay'                     # the pitch of current timestamp using curve fitting (using smoothed_pose and map pitch)
PITCH_CURRENT = 'pitch_current'                 # the pitch considering delay time using curve fitting
PITCH_LOOKAHEAD = 'pitch_lookahead'             # the pitch with lookahead time using curve fitting
PITCH_Z_DIFFERENTIATOR = 'pitch_z_differentiator'




# weight dict 
TOTAL_WEIGHT = 'total_weight'
TOTAL_WEIGHT_ESTIMATE = 'total_weight_estimate'

# flag dict
DBW_ENABLED = 'dbw_enabled'
AEB_ENABLED = 'aeb_enabled'
ABS_ACTIVE = 'abs_active'
ABS_ENABLED = 'abs_enabled'
STAB_ACTIVE = 'stab_active'
STAB_ENABLED = 'stab_enabled'

# other
LEAD_DISTANCE = 'lead_distance'
TOTAL_VEHICLE_DISTANCE = 'total_vehicle_distance'
ACTION_TYPE = 'action_type'
CLUTCH_SLIP = 'clutch_slip'
ENGINE_FRICTION_TORQUE = 'engine_friction_torque'
ENIGIN_OUTPUT_TORQUE = 'engine_output_torque'
ENGINE_RPM = 'engine_rpm'

# lat dict
LAT_ERROR = 'lat_err'
HEADING_ERROR = 'heading_err'
CURVATURE = 'curvature'
YAW_RATE = 'yaw_rate'

# heading dict
HEADING_ANGLE = 'heading angle'
HEADING_REF = 'heading_ref'

# steering dict
STEERING_ANGLE = 'steering_angle'
STEERING_ANGLE_CMD = 'steering_angle_cmd'


TOPIC_INFO = {}
# TOPIC_INFO['planning'] = ( topic_name, callback_function, frequency, [variables])
TOPIC_INFO['planning'] = ( '/planning/trajectory', 'planning_online_callback', 'planning_offline_callback',10,
                          [THROTTLE_GSP0, THROTTLE_GSP1, THROTTLE_GSP2, THROTTLE_GSP_WEIGHTED, THROTTLE_FROM_CONTROL, COASTING_FLAG, 
                           BRAKE_GSP0, BRAKE_GSP1, BRAKE_GSP2, REF_ACC_IN_PLANNING, MAX_ACC,ACC_GSP0, ACC_GSP1,ACC_GSP2, 
                           V_GSP0, V_GSP1, V_GSP2, CRUISE_SPEED, SPEED_LIMIT, SPEED_LIMIT_FROM_FE, 
                           SPEED_LIMIT_FROM_CURVATURE, SPEED_LIMIT_FROM_MAP, TARGET_SPEED_SOURCE,GSP_THROTTLE])
TOPIC_INFO['lead_info'] = ( '/planning/lead_info', 'lead_online_callback', 'lead_offline_callback', 10,[V_LEAD, LEAD_DISTANCE])
TOPIC_INFO['control_cmd'] = ( '/vehicle/control_cmd', 'control_online_callback', 'control_offline_callback', 20,
                          [THROTTLE_CMD, BRAKE_CMD, ACC_PLANNED, ACC_TARGET_MPC, ACC_TARGET_WITH_MARC, 
                           COASTING_ACC_V_OBSERVED, COASTING_ACC_CA_OBSERVED,
                           ACC_TARGET_WITH_COMPENSATION, ACC_TARGET, ACC_REPORT, ACC_COMPUTED, A_IMU, A_CHASSIS, ACC_PITCH, 
                           COASTING_ACC_EST, COASTING_ACC_EST_LOW_SPEED, ACC_EST, COASTING_ACC_CA_EST, COASTING_ACC_CD_EST, 
                           COASTING_ACC_CONFI, ACC_DIFFERENTIAL, ACC_LEVEL, ACC_DELAY_COMPENSATOR_V, 
                           ACC_DELAY_COMPENSATOR_V_ERR, ACC_DELAY_COMPENSATOR_XI, ACC_DELAY_COMPENSATOR_U, CMD_CONSTRAINTS, 
                           V_TARGET, V_TARGER_FINAL, V_ERROR, V_ESTIMATION_ERR, V_ESTIMATION, V_IDLE, THROTTLE_IDLE, 
                           REF_MODEL_THROTTLE, FEEDBACK_CMD_THROTTLE, KC_THROTTLE, KS_THROTTLE, THETA_THROTTLE, 
                           U_REFERENCE_THROTTLE, U_STATE_THROTTLE, U_DISTURBANCE_THROTTLE, THROTTLE_ERR, REF_MODEL_BRAKE, 
                           FEEDBACK_CMD_BRAKE, KC_BRAKE, KS_BRAKE, THETA_BRAKE, U_REFERENCE_BRAKE, U_STATE_BRAKE, 
                           U_DISTURBANCE_BRAKE, BRAKE_ERR, GEAR_CMD_CONTROL, TOTAL_WEIGHT, TOTAL_WEIGHT_ESTIMATE, 
                           LAT_ERROR, HEADING_ERROR, CURVATURE, YAW_RATE, HEADING_ANGLE, HEADING_REF, STEERING_ANGLE_CMD,
                           V_KALMAN, V_ESTIMATED, COASTING_ACC_EST_FLAG, Z_FORM_Z_DIFFERENTIATOR, WEIGHT_ESTIMATED, DISTURBANCE_ESTIMATED, 
                           CONDITION_MET_FLAG, RESISTANCE, FIRST_RUN_SOLVED, HOT_RUN_SOLVED, Cr_ESTIMATED, Ca_ESTIMATED,WEIGHT_ESTIMATION_COVARIANCE,
                           OSQP_SOLVE_STATUS, SWITCH_CONTROLLER_FLAG, FALLBACK_CONTROL_CMD, WEIGHT_CALIBRATED, WEIGHT_ESTIMATION_VALID])
TOPIC_INFO['dbw_reports'] = ( '/vehicle/dbw_reports', 'dbw_online_callback', 'dbw_offline_callback', 20,
                          [THROTTLE_INPUT, THROTTLE_OUTPUT, BRAKE_INPUT, BRAKE_OUTPUT, V_CURRENT_DBW, GEAR_CMD, GEAR_REPORT, 
                           ENGINE_FUEL_RATE, TOTAL_FUEL_USED, DBW_ENABLED, AEB_ENABLED, ABS_ACTIVE, ABS_ENABLED, STAB_ACTIVE, 
                           STAB_ENABLED, ACTION_TYPE, ENGINE_FRICTION_TORQUE, ENIGIN_OUTPUT_TORQUE, ENGINE_RPM, 
                           TOTAL_VEHICLE_DISTANCE, CLUTCH_SLIP, STEERING_ANGLE, RETARDER, ENGINE_BRAKE])
TOPIC_INFO['vehicle_state'] = ( '/vehicle/status', 'vehicle_state_online_callback', 'vehicle_state_offline_callback',20,
                          [COASTING_ACC_FORMULA, COASTING_ACC, COASTING_ACC_EST, PITCH_MAP, PITCH_CONTROL, PITCH_CONTROL, 
                           PITCH_MAP, PITCH_SMOOTHED_POSE, PITCH_POSE, PITCH_Z, PITCH_DELAY, PITCH_CURRENT, PITCH_LOOKAHEAD,
                           ACC_FROM_IMU_FILTERED, ACC_FROM_KALMAN_DIFFERENTIATOR, PITCH_Z_DIFFERENTIATOR, V_CURRENT_WHEEL,
                           GEAR_RATIO])
TOPIC_INFO['localization'] = ( '/navsat/odom', 'localization_online_callback', 'localization_offline_callback', 20,
                          [ELEVATION])
TOPIC_INFO['imu'] = ( '/imu/data', 'imu_online_callback', 'imu_offline_callback', 100,
                          [A_RAW_IMU, A_FILTERED_IMU_1, A_JERK_IMU_1, A_FILTERED_IMU_2, A_JERK_IMU_2, AZ_RAW, 
                           AZ_JERK_1, AZ_1, AZ_JERK_2, AZ_2])
