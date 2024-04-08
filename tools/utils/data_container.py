#!/usr/bin/env python
import time
import math
import copy
import threading
import logging
import jerk_utils
import rospy
from control import vehicle_state_pb2
from control import control_command_pb2
from control import dbw_reports_pb2
from localization import localization_pb2
from planning import planning_trajectory_pb2
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from scipy import signal
from topics_and_signals import *


_G = 9.81
_MAX_ENGINE_TORQUE = 2580

logger = logging.getLogger(__name__)

class DataList:
    def __init__(self, freq):
        self.data = list()
        self.freq = freq
        
class DataContainer:
    def __init__(self,online=False):
        self._online = online
        if(self._online):
            self.online_setup()
        else:
            self.offline_setup()
        self.init_msg()
        self.set_fir_filter_param()
        self._lock = threading.Lock()
        self._vehicle_state_freqency = TOPIC_INFO['vehicle_state'][3]
        self._dbw_report_freqency = TOPIC_INFO['dbw_reports'][3]
        self._subscribe_count = 0
        self._last_fuel_used = 0
        self._last_total_vehicle_distance = 0
        self._longitudinal_imu_filter = jerk_utils.TrackingDifferentiator(tracking_factor=5, order=2)
    
    def offline_setup(self):
        self._data_dict = {}
        self._topic_list = []
        self._topic_handlers = {}
        for topic_name, topic_info in TOPIC_INFO.items():
            topic = topic_info[0]
            offline_callback = topic_info[2]
            self._topic_handlers[topic] = getattr(self,offline_callback)
            self._topic_list.append(topic)
            variables_in_topic = topic_info[4]
            topic_frequency = topic_info[3]
            for variable in variables_in_topic:
                self._data_dict[variable] = DataList(topic_frequency)    

    def online_setup(self):
        self._data_dict = {}
        for topic_name, topic_info in TOPIC_INFO.items():
            topic = topic_info[0]
            online_callback = topic_info[1]
            if(topic_name=='imu'):
                rospy.Subscriber(TOPIC_INFO['imu'][0], Imu, getattr(self,TOPIC_INFO['imu'][1]))  
            elif(topic_name=='localization') :
                rospy.Subscriber(TOPIC_INFO['localization'][0], Odometry, getattr(self,TOPIC_INFO['localization'][1])) 
            else:
                rospy.Subscriber(topic, String, getattr(self,online_callback))
            variables_in_topic = topic_info[4]
            topic_frequency = topic_info[3]
            for variable in variables_in_topic:
                self._data_dict[variable] = DataList(topic_frequency)  
        
    def reset_data_dict(self):
        self._data_dict = {}
        for topic_name, topic_info in TOPIC_INFO.items():
            variables_in_topic = topic_info[4]
            topic_frequency = topic_info[3]
            for variable in variables_in_topic:
                self._data_dict[variable] = DataList(topic_frequency)     
                                          
    def init_msg(self):
        self._planning_trajectory = planning_trajectory_pb2.PlanningTrajectory()
        self._lead_info = planning_trajectory_pb2.LeadInfo()
        self._control_cmd = control_command_pb2.ControlCommand()
        self._dbw_reports = dbw_reports_pb2.DbwReports()
        self._vehicle_status = vehicle_state_pb2.VehicleState()
        self._localization = Odometry()

    def set_fir_filter_param(self):
        self._acc_fir_init = False
        self._az_fir_init = False
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._times = []
        self._window_size = 50
        self._rate = 100
    
    def get_data_dict(self):
        return self._data_dict
           
    def get_topic_list(self):
        return self._topic_list
           
    def get_color_list(self):
        return self._color_list
    
    def get_subscribe_count(self):
        return self._subscribe_count
    
    def update_subscribe_count(self):
        self._subscribe_count += 1

    def imu_callback_util(self, msg, cur_callback_time):
         with self._lock:
            self.imu_data=msg.linear_acceleration
            self._data_dict[A_RAW_IMU].data.append((cur_callback_time, float(self.imu_data.y)))
            self._data_dict[AZ_RAW].data.append((cur_callback_time, float(self.imu_data.z)))

            if not self._acc_fir_init:
                self.acc_b = signal.firwin(30, 0.01)
                self.acc_z = signal.lfilter_zi(self.acc_b, 1)
                self._acc_fir_init=True
            else:
                imu_data_list = self._data_dict[A_RAW_IMU].data
                result,self.acc_z = signal.lfilter(self.acc_b, 1, [imu_data_list[-1][1]], zi=self.acc_z)
                self._data_dict[A_FILTERED_IMU_1].data.append((cur_callback_time,float(result)))
                filtered_imu_data_list = self._data_dict[A_FILTERED_IMU_1].data
                length = 2
                if len(filtered_imu_data_list)>length :
                    jerk = (filtered_imu_data_list[-1][1]-filtered_imu_data_list[-length][1]) / ((length-1)*0.01) #imu topic frequency is 100hz.
                    self._data_dict[A_JERK_IMU_1].data.append((cur_callback_time,float(jerk)))

            if not self._az_fir_init:
                self.az_b = signal.firwin(30, 0.01)
                self.az_z = signal.lfilter_zi(self.az_b, 1)
                self._az_fir_init=True
            else:
                imu_az_list = self._data_dict[AZ_RAW].data
                az_result,self.az_z = signal.lfilter(self.az_b, 1, [imu_az_list[-1][1]], zi=self.az_z)
                self._data_dict[AZ_1].data.append((cur_callback_time,float(az_result)))
                filtered_imu_az_list = self._data_dict[AZ_1].data
                length = 2
                if len(filtered_imu_az_list)>length :
                    jerk = (filtered_imu_az_list[-1][1]-filtered_imu_az_list[-length][1]) / ((length-1)*0.01) #imu topic frequency is 100hz.
                    self._data_dict[AZ_JERK_1].data.append((cur_callback_time,float(jerk)))              

            longitudinal_acc = self.imu_data.y
            # update imu filter
            if not self._longitudinal_imu_filter.is_activated:
                # initialize longitudinal jerk filter
                longitudinal_acc_filtered = longitudinal_acc
                longitudinal_jerk_filtered = 0
                self._longitudinal_imu_filter.reset( cur_callback_time, [longitudinal_acc_filtered, longitudinal_jerk_filtered])
            else:(longitudinal_acc_filtered,longitudinal_jerk_filtered,) = self._longitudinal_imu_filter.step(longitudinal_acc, cur_callback_time)
            
            self._data_dict[A_FILTERED_IMU_2].data.append((cur_callback_time,float(longitudinal_acc_filtered)))
            self._data_dict[A_JERK_IMU_2].data.append((cur_callback_time,float(longitudinal_jerk_filtered)))

    def imu_online_callback(self, msg):
        self.imu_callback_util(msg,time.time())
        
    def imu_offline_callback(self, msg, bag_time):
        self.imu_callback_util(msg,bag_time.to_sec())

    def planning_trajectory_callback_util(self, msg, cur_callback_time): 
        with self._lock:
            self.update_subscribe_count()
            self._planning_trajectory.ParseFromString(msg.data)
            # self._data_dict[REF_ACC_IN_PLANNING].data.append((cur_callback_time, float(self._planning_trajectory.ref_a_from_bsfc)))
            # self._data_dict[MAX_ACC].data.append((cur_callback_time, float(self._planning_trajectory.max_available_acc)))
            self._data_dict[CRUISE_SPEED].data.append((cur_callback_time, float(self._planning_trajectory.target_cruise_speed)))
            self._data_dict[SPEED_LIMIT].data.append((cur_callback_time, float(self._planning_trajectory.speed_limit_used)))
            self._data_dict[SPEED_LIMIT_FROM_FE].data.append((cur_callback_time, float(self._planning_trajectory.speed_limit_from_fe)))
            self._data_dict[SPEED_LIMIT_FROM_CURVATURE].data.append((cur_callback_time, float(self._planning_trajectory.speed_limit_from_curvature)))
            self._data_dict[SPEED_LIMIT_FROM_MAP].data.append((cur_callback_time, float(self._planning_trajectory.speed_limit_from_map)))           
            self._data_dict[COASTING_FLAG].data.append((cur_callback_time, float(self._planning_trajectory.coasting)))
            self._data_dict[TARGET_SPEED_SOURCE].data.append((cur_callback_time, float(self._planning_trajectory.target_speed_source)))
            # self._data_dict[GSP_THROTTLE].data.append((cur_callback_time, float(self._planning_trajectory.gspRecordThrottle)))
            
            num_of_point = len(self._planning_trajectory.trajectory_point)
            if (num_of_point > 0) :
                self._data_dict[V_TARGER_FINAL].data.append((cur_callback_time,float(self._planning_trajectory.trajectory_point[num_of_point-1].v)))

            # if(self._planning_trajectory.online_gsp_output.size>0):
            #     self._data_dict[THROTTLE_GSP0].data.append((cur_callback_time, float(self._planning_trajectory.online_gsp_output.online_gsp_output_unit[0].throttle)))
            #     self._data_dict[ACC_GSP0].data.append((cur_callback_time, float(self._planning_trajectory.online_gsp_output.online_gsp_output_unit[0].acc)))
            #     self._data_dict[V_GSP0].data.append((cur_callback_time, float(self._planning_trajectory.online_gsp_output.online_gsp_output_unit[0].velocity)))
                
            #     self._data_dict[THROTTLE_GSP1].data.append((cur_callback_time, float(self._planning_trajectory.online_gsp_output.target_online_gsp_output_unit.throttle)))
            #     self._data_dict[ACC_GSP1].data.append((cur_callback_time, float(self._planning_trajectory.online_gsp_output.target_online_gsp_output_unit.acc)))
            #     self._data_dict[V_GSP1].data.append((cur_callback_time, float(self._planning_trajectory.online_gsp_output.target_online_gsp_output_unit.velocity)))
                
            #     self._data_dict[THROTTLE_GSP_WEIGHTED].data.append((cur_callback_time, float(self._planning_trajectory.online_gsp_output.weighted_gsp_throttle)))            

    def planning_online_callback(self, msg):
        self.planning_trajectory_callback_util(msg,time.time())
        
    def planning_offline_callback(self, msg, bag_time):
        self.planning_trajectory_callback_util(msg,bag_time.to_sec())

    def lead_callback_util(self, msg, cur_callback_time):
        with self._lock:
            self.update_subscribe_count()
            self._lead_info.ParseFromString(msg.data)
            self._data_dict[V_LEAD].data.append((cur_callback_time, float(self._lead_info.speed)))
            self._data_dict[LEAD_DISTANCE].data.append((cur_callback_time, float(self._lead_info.distance)))

    def lead_online_callback(self, msg):
        self.lead_callback_util(msg,time.time())
        
    def lead_offline_callback(self, msg, bag_time):
        self.lead_callback_util(msg,bag_time.to_sec())
    
    def control_callback_util(self, msg, cur_callback_time):
        with self._lock:
            self.update_subscribe_count()
            self._control_cmd.ParseFromString(msg.data)
            self._data_dict[THROTTLE_CMD].data.append((cur_callback_time, float(self._control_cmd.throttle_cmd.normalized_value)))
            self._data_dict[BRAKE_CMD].data.append((cur_callback_time, float(self._control_cmd.brake_cmd.normalized_value)))

            self._data_dict[ACC_PLANNED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.a_target_from_planning)))
            self._data_dict[ACC_TARGET] .data.append((cur_callback_time, float(self._control_cmd.debug_cmd.a_target)))
            self._data_dict[ACC_REPORT].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.a_report)))
            self._data_dict[ACC_TARGET_MPC].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.mpc_acceleration_cmd)))
            
            self._data_dict[V_TARGET].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.v_target)))
            # self._data_dict[V_TARGER_FINAL].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.final_reference_v)))
            
            self._data_dict[TOTAL_WEIGHT].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.total_weight)))
            self._data_dict[TOTAL_WEIGHT_ESTIMATE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.total_weight_estimation)))

            self._data_dict[V_ERROR].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.v_error)))
            self._data_dict[ACC_PITCH].data.append((cur_callback_time, _G*math.sin(-(float(self._control_cmd.debug_cmd.pitch_angle))*math.pi/180)))
          
            self._data_dict[GEAR_CMD_CONTROL].data.append((cur_callback_time, int(self._control_cmd.gear_cmd)))

            self._data_dict[V_IDLE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.idle_speed_current_gear)))
            self._data_dict[ACC_LEVEL].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.planning_required_precision_level_of_acc)))
            
            # self._data_dict[COASTING_ACC_V_OBSERVED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.coasting_acc_estimator_v_observed)))

            # ca_estimated = float(self._control_cmd.debug_cmd.a_coast_coeff_a_est)
            # if(ca_estimated != 0):
            #     ca_estimated -= 4
            # self._data_dict[COASTING_ACC_CA_EST].data.append((cur_callback_time, ca_estimated))
            # ca_observed = float(self._control_cmd.debug_cmd.coasting_acc_estimator_ca_observed)
            # if(ca_observed != 0):
            #     ca_observed -= 4            
            # self._data_dict[COASTING_ACC_CA_OBSERVED].data.append((cur_callback_time, ca_observed))
            # self._data_dict[COASTING_ACC_EST_FLAG].data.append((cur_callback_time, int(self._control_cmd.debug_cmd.coasting_acc_estimation_flag)))
            
            # self._data_dict[COASTING_ACC_CD_EST].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.a_caost_coeff_d_est)))           
            # self._data_dict[COASTING_ACC_CONFI].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.a_coast_confi))) 

            self._data_dict[ACC_DELAY_COMPENSATOR_XI].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.acc_delay_compensator_xi))) 
            self._data_dict[ACC_DELAY_COMPENSATOR_U].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.acc_delay_compensator_u)))
            
            GAIN = 30
            # mrac throttle 
            self._data_dict[REF_MODEL_THROTTLE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.mrac_state.mrac_ref_model_throttle)))
            self._data_dict[FEEDBACK_CMD_THROTTLE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.a_cmd_feedback_mrac_throttle)))
            self._data_dict[KC_THROTTLE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_kc_throttle)))
            self._data_dict[KS_THROTTLE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_ks_throttle)))
            self._data_dict[THETA_THROTTLE].data.append((cur_callback_time, 10*float(self._control_cmd.debug_cmd.mrac_state.mrac_theta_throttle)))
            self._data_dict[U_REFERENCE_THROTTLE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_u_reference_throttle)))
            self._data_dict[U_STATE_THROTTLE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_u_state_throttle)))
            self._data_dict[U_DISTURBANCE_THROTTLE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_u_disturbance_throttle)))
            self._data_dict[THROTTLE_ERR].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.mrac_state.mrac_model_error_throttle)))
            # mrac brake
            self._data_dict[REF_MODEL_BRAKE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.mrac_state.mrac_ref_model_brake)))
            self._data_dict[FEEDBACK_CMD_BRAKE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.a_cmd_feedback_mrac_brake)))
            self._data_dict[KC_BRAKE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_kc_brake)))
            self._data_dict[KS_BRAKE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_ks_brake)))
            self._data_dict[THETA_BRAKE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_theta_brake)))
            self._data_dict[U_REFERENCE_BRAKE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_u_reference_brake)))
            self._data_dict[U_STATE_BRAKE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_u_state_brake)))
            self._data_dict[U_DISTURBANCE_BRAKE].data.append((cur_callback_time, GAIN*float(self._control_cmd.debug_cmd.mrac_state.mrac_u_disturbance_brake)))
            self._data_dict[BRAKE_ERR].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.mrac_state.mrac_model_error_brake)))

            a_cmd_with_mrac = float(self._control_cmd.debug_cmd.mpc_acceleration_cmd) + float(self._control_cmd.debug_cmd.mrac_state.a_cmd_feedback_mrac_throttle)
            # self._data_dict[ACC_TARGET_WITH_MARC].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.a_cmd_with_mrac)))
            self._data_dict[ACC_TARGET_WITH_MARC].data.append((cur_callback_time, a_cmd_with_mrac))
            a_with_delay_compensation = float(self._control_cmd.debug_cmd.mpc_acceleration_cmd) - float(self._control_cmd.debug_cmd.acc_delay_compensator_xi)
            self._data_dict[ACC_TARGET_WITH_COMPENSATION].data.append((cur_callback_time, a_with_delay_compensation))

            self._data_dict[ACTION_TYPE].data.append((cur_callback_time, int(self._control_cmd.debug_cmd.control_action_type)))

            self._data_dict[LAT_ERROR].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.lateral_error)))
            self._data_dict[HEADING_ERROR].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.heading_error)))
            self._data_dict[CURVATURE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.curvature)))
            self._data_dict[YAW_RATE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.yawrate)))
            self._data_dict[HEADING_ANGLE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.heading)))
            self._data_dict[HEADING_REF].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.ref_heading)))
            self._data_dict[STEERING_ANGLE_CMD].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.steering_cmd_from_controller)))
            self._data_dict[V_KALMAN].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.v_kalman)))
            self._data_dict[V_ESTIMATED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.v_estimated)))
            self._data_dict[RESISTANCE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.resistance)))
            
            # self._data_dict[Z_FORM_Z_DIFFERENTIATOR].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.z_from_z_differentiator)))

            self._data_dict[WEIGHT_ESTIMATED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.weight_estimated)))
            self._data_dict[WEIGHT_CALIBRATED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.weight_calibrated)))
            self._data_dict[WEIGHT_ESTIMATION_COVARIANCE].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.weight_estimation_covariance)))
            # self._data_dict[WEIGHT_ESTIMATED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.weight_estimated)))
            self._data_dict[DISTURBANCE_ESTIMATED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.disturbance_estimated)))
            self._data_dict[CONDITION_MET_FLAG].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.condition_met_flag)))
            self._data_dict[WEIGHT_ESTIMATION_VALID].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.weight_estimation_valid)))
            
            self._data_dict[FIRST_RUN_SOLVED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.augmented_mpc_solve_result.first_run_solved)))
            self._data_dict[HOT_RUN_SOLVED].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.augmented_mpc_solve_result.hot_start_solved)))
            self._data_dict[OSQP_SOLVE_STATUS].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.augmented_mpc_solve_result.osqp_solve_status)))
            self._data_dict[SWITCH_CONTROLLER_FLAG].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.switch_fallback_lon_controller)))
            self._data_dict[FALLBACK_CONTROL_CMD].data.append((cur_callback_time, float(self._control_cmd.debug_cmd.fallback_lon_controller_cmd)))

    def control_online_callback(self, msg):
        self.control_callback_util(msg,time.time())
        
    def control_offline_callback(self, msg, bag_time):
        self.control_callback_util(msg,bag_time.to_sec())

    def dbw_callback_util(self, msg, cur_callback_time):
        with self._lock:
            self.update_subscribe_count()
            self._dbw_reports.ParseFromString(msg.data)
            self._data_dict[THROTTLE_INPUT].data.append((cur_callback_time, float(self._dbw_reports.throttle_report.pedal_input)))
            self._data_dict[THROTTLE_OUTPUT].data.append((cur_callback_time, float(self._dbw_reports.throttle_report.pedal_output)))
            self._data_dict[BRAKE_INPUT].data.append((cur_callback_time, float(self._dbw_reports.brake_report.pedal_input)))
            self._data_dict[BRAKE_OUTPUT].data.append((cur_callback_time, float(self._dbw_reports.brake_report.pedal_output)))
            self._data_dict[V_CURRENT_DBW].data.append((cur_callback_time, float(self._dbw_reports.steering_report.speed)))
            self._data_dict[STEERING_ANGLE].data.append((cur_callback_time, 180 * float(self._dbw_reports.steering_report.steering_wheel_angle) / math.pi))
            self._data_dict[RETARDER].data.append((cur_callback_time, float(self._dbw_reports.retarder_msg.actual_retarder_percent_torque)))
            
            self._data_dict[GEAR_REPORT].data.append((cur_callback_time, int(self._dbw_reports.gear_report.state)))
            self._data_dict[GEAR_CMD].data.append((cur_callback_time, int(self._dbw_reports.gear_report.cmd)))
            self._data_dict[ENGINE_BRAKE].data.append((cur_callback_time, float(self._dbw_reports.engine_brake_msg.actual_engine_brake_percent_torque)))
            self._data_dict[ENGINE_FUEL_RATE].data.append((cur_callback_time, float(self._dbw_reports.fuel_economy.engine_fuel_rate)))
            # self._data_dict[TOTAL_FUEL_USED].data.append((cur_callback_time, int(self._dbw_reports.fuel_economy.total_fuel_used)))
            fuel_used = self._last_fuel_used + float(self._dbw_reports.fuel_economy.engine_fuel_rate)/self._dbw_report_freqency
            self._data_dict[TOTAL_FUEL_USED].data.append((cur_callback_time, fuel_used))
            self._last_fuel_used = fuel_used

            self._data_dict[ENGINE_FRICTION_TORQUE].data.append((cur_callback_time, float(self._dbw_reports.throttle_info_report.throttle_pc)))
            self._data_dict[ENGINE_RPM].data.append((cur_callback_time, int(self._dbw_reports.throttle_info_report.engine_rpm)))
            self._data_dict[CLUTCH_SLIP].data.append((cur_callback_time, float(self._dbw_reports.gear_info_report.clutch_slip)))


            self._data_dict[DBW_ENABLED].data.append((cur_callback_time, 5*int(self._dbw_reports.dbw_enabled)))
            self._data_dict[AEB_ENABLED].data.append((cur_callback_time, 5*int(self._dbw_reports.aeb_enabled)))
            self._data_dict[ABS_ACTIVE].data.append((cur_callback_time, 5*int(self._dbw_reports.brake_info_report.abs_active)))
            self._data_dict[ABS_ENABLED].data.append((cur_callback_time, 5*int(self._dbw_reports.brake_info_report.abs_enabled)))
            self._data_dict[STAB_ACTIVE].data.append((cur_callback_time, 5*int(self._dbw_reports.brake_info_report.stab_active)))
            self._data_dict[STAB_ENABLED].data.append((cur_callback_time, 5*int(self._dbw_reports.brake_info_report.stab_enabled)))
            self._data_dict[ENIGIN_OUTPUT_TORQUE].data.append((cur_callback_time, _MAX_ENGINE_TORQUE*float(self._dbw_reports.throttle_report.pedal_output)))

    def dbw_online_callback(self, msg):
        self.dbw_callback_util(msg,time.time())
        
    def dbw_offline_callback(self, msg, bag_time):
        self.dbw_callback_util(msg,bag_time.to_sec())
   
    def vehicle_state_callback_util(self, msg, cur_callback_time):
        with self._lock:
            self.update_subscribe_count()
            self._vehicle_status.ParseFromString(msg.data)
            self._data_dict[COASTING_ACC].data.append((cur_callback_time, float(self._vehicle_status.coasting_acceleration)))
            self._data_dict[COASTING_ACC_EST].data.append((cur_callback_time, float(self._vehicle_status.coasting_acceleration_from_estimation)))
            self._data_dict[COASTING_ACC_FORMULA].data.append((cur_callback_time, float(self._vehicle_status.coasting_acceleration_from_formula)))
            self._data_dict[PITCH_CONTROL].data.append((cur_callback_time, 180 * float(self._vehicle_status.pitch) / math.pi))
            self._data_dict[PITCH_MAP].data.append((cur_callback_time, 180 * float(self._vehicle_status.map_pitch) / math.pi))
            self._data_dict[PITCH_SMOOTHED_POSE].data.append((cur_callback_time, 180 * float(self._vehicle_status.smoothed_pose_pitch) / math.pi))
            self._data_dict[PITCH_POSE].data.append((cur_callback_time, 180 * float(self._vehicle_status.pose_pitch) / math.pi))
            self._data_dict[PITCH_Z].data.append((cur_callback_time, 180 * float(self._vehicle_status.z_pitch) / math.pi))
            self._data_dict[PITCH_DELAY].data.append((cur_callback_time, 180 * float(self._vehicle_status.delayed_pitch) / math.pi))
            self._data_dict[PITCH_CURRENT].data.append((cur_callback_time, 180 * float(self._vehicle_status.current_pitch) / math.pi))
            self._data_dict[PITCH_LOOKAHEAD].data.append((cur_callback_time, 180 * float(self._vehicle_status.lookahead_pitch) / math.pi))
            self._data_dict[PITCH_Z_DIFFERENTIATOR].data.append((cur_callback_time, 180 * float(self._vehicle_status.pitch_from_z_differentiator) / math.pi))
            self._data_dict[GEAR_RATIO].data.append((cur_callback_time, float(self._vehicle_status.transmission_gear_ratio)))
            self._data_dict[A_IMU].data.append((cur_callback_time, float(self._vehicle_status.a_y)))
            
            
            self._data_dict[A_CHASSIS].data.append((cur_callback_time, float(self._vehicle_status.chassis_a_y)))
            self._data_dict[V_CURRENT_WHEEL].data.append((cur_callback_time, float(self._vehicle_status.v)))
            total_vehicle_distance = self._last_total_vehicle_distance + float(self._vehicle_status.v)/self._vehicle_state_freqency
            self._data_dict[TOTAL_VEHICLE_DISTANCE].data.append((cur_callback_time, total_vehicle_distance))
            self._last_total_vehicle_distance = total_vehicle_distance
            
            self._data_dict[ACC_FROM_IMU_FILTERED].data.append((cur_callback_time, float(self._vehicle_status.acc_from_imu_filtered)))
            self._data_dict[ACC_FROM_KALMAN_DIFFERENTIATOR].data.append((cur_callback_time, float(self._vehicle_status.acc_from_kalman_differentiator)))

    def vehicle_state_online_callback(self, msg):
        self.vehicle_state_callback_util(msg,time.time())
        
    def vehicle_state_offline_callback(self, msg, bag_time):
        self.vehicle_state_callback_util(msg,bag_time.to_sec())
       
    def localization_callback_util(self,msg,cur_callback_time):
        with self._lock:
            self.update_subscribe_count()
            self._localization = msg
            self._data_dict[ELEVATION].data.append((cur_callback_time, float(self._localization.pose.pose.position.z))) 

    def localization_online_callback(self, msg):
        self.localization_callback_util(msg,time.time())
        
    def localization_offline_callback(self, msg, bag_time):
        self.localization_callback_util(msg,bag_time.to_sec())

    def compute_acc(self):
        current_speed = 0
        # current_speed = self._data_dict[V_CURRENT].data
        # self._data_dict[ACC_DIFFERENTIAL].data = copy.deepcopy(current_speed)
        # window_size = 2 * self._vehicle_state_freqency
        
        # for i in range(window_size):
        #     self._data_dict[ACC_DIFFERENTIAL].data[i] = (current_speed[i][0],self._data_dict[A_CHASSIS].data[i][1])
            
        # front_size = 10
        # back_size = window_size - front_size
        # i = back_size
        # while((i + front_size)<len(current_speed)):
        #     front_v = current_speed[i + front_size][1]
        #     back_v = current_speed[i-back_size][1]
        #     front_time = current_speed[i + front_size][0]
        #     back_time = current_speed[i - back_size][0]
        #     acc_differential = (front_v -back_v )/(front_time - back_time)
        #     self._data_dict[ACC_DIFFERENTIAL].data[i] = (current_speed[i][0],acc_differential)
        #     i += 1

        # total_length = len(self._data_dict[ACC_DIFFERENTIAL].data)
        # for j in range(i,total_length):
        #     self._data_dict[ACC_DIFFERENTIAL].data[j] = (current_speed[j][0],self._data_dict[A_CHASSIS].data[j][1])

                            
    def compute_fuel_consumption(self):
        logger.info("compute_fuel_consumption")
        return self._last_fuel_used * 100000 / self._last_total_vehicle_distance
