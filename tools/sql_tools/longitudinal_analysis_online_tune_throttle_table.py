#!/usr/bin/env python3

import os
import pandas as pd
from clickhouse_sqlalchemy import make_session
from sqlalchemy import create_engine
from datetime import datetime
from pluspy import db_utils
import matplotlib
import matplotlib.pyplot as plt
import logging
import numpy as np
import math
import time

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

FIG_CONFIG = {
    'figsize': (25, 20),
    'dpi': 80,
    'fontsize': 12,
    'subplots_adjust_left': 0.08,
    'subplots_adjust_bottom': 0.06,
    'subplots_adjust_right': 0.92,
    'subplots_adjust_top': 0.95,
    'subplots_adjust_wspace': 0.15,
    'subplots_adjust_hspace': 0.20,
}

CH_CONFIG = {
    "driver": "clickhouse+native",
    "port": 9000,
    "database": "bagdb",
    "host": "clickhouse-cn",
    "user": "plus_viewer",
    "password": "ex4u1balSAeR68uC",
}
class LonMetricAnalysis:

    def __init__(self, session, config_dict):
        self._session = session
        self._vehicle_name = config_dict.get("vehicle_name")
        start_ts = datetime.strptime(config_dict.get("start_time"),
                                     "%Y-%m-%d %H:%M:%S.%f")
        start_ts = time.mktime(start_ts.timetuple())
        end_ts = datetime.strptime(config_dict.get("end_time"),
                                   "%Y-%m-%d %H:%M:%S.%f")
        end_ts = time.mktime(end_ts.timetuple())
        time_step = config_dict.get("time_step")
        self._search_ts_list = []
        temp_ts = start_ts
        while temp_ts < end_ts:
            self._search_ts_list.append(temp_ts)
            temp_ts += time_step
        self._search_ts_list.append(end_ts)
        self._control_cmd_sql = \
            """
            SELECT ts,
            JSONExtractBool(vehicle_dbw_reports, 'superpilotEnabled') as superpilot_enabled,
            JSONExtractBool(vehicle_dbw_reports, 'dbwEnabled') as dbw_enabled,
            JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'vError') as v_error,
            JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'aReport') as a_report,
            JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'mpcAccelerationCmd') as mpc_acceleration_cmd,
            JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'onlineTuneThrottleTable', 'totalDrivelineInertiaFactor', 'gear11') as driveline_inertia_factor_11,
            JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'onlineTuneThrottleTable', 'totalDrivelineInertiaFactor', 'gear12') as driveline_inertia_factor_12,
            JSONExtractBool(planning_trajectory, 'enabledStitchVelocity') as enabled_stitch_velocity
            FROM bag_messages bm
            WHERE vehicle = '{0}' and  ts > '{1}' and ts < '{2}'
            ORDER BY ts
            """
        self.avg_v_error = [0, 0]
        self.avg_a_error = [0, 0]
        self.avg_driveline_inertia_factor_11 = [0, 0]
        self.avg_driveline_inertia_factor_12 = [0, 0]
        self.v_error_total_list = []
        self.a_error_total_list = []
        self.driveline_inertia_factor_11_total_list = []
        self.driveline_inertia_factor_12_total_list = []
        self.filtered_driveline_inertia_factor_11_total_list = []
        self.filtered_driveline_inertia_factor_12_total_list = []
        self.timestamp_list = []
        self.v_rmse = [0, 0]
        self.a_rmse = [0, 0]
        self.stitch_list = []
        self.enable_stitch_list = []
        self.stitch_precent = 0

    def get_query(self):
        for index, _ in enumerate(range(len(self._search_ts_list) - 1)):
            v_error_list = []
            a_error_list = []
            driveline_inertia_factor_11_list = []
            driveline_inertia_factor_12_list = []

            # print(self._vehicle_name, self._search_ts_list[index],
            #       self._search_ts_list[index + 1])

            try:
                with db_utils.db_session_open_close(CH_CONFIG) as db_session:
                    query_msg = db_session.execute(
                        self._control_cmd_sql.format(
                            self._vehicle_name, self._search_ts_list[index],
                            self._search_ts_list[index + 1])).fetchall()
                    # print('Query Successfully!')
            except Exception:
                print('Query Error!')
                continue

            
            superpilot_enabled_flag = False
            dbw_enabled_flag = False
            enabled_stitch_velocity_flag = False
            for msg in query_msg:
                if msg['enabled_stitch_velocity'] is not None:
                    enabled_stitch_velocity = msg['enabled_stitch_velocity']
                    self.stitch_list.append(enabled_stitch_velocity)
                    if enabled_stitch_velocity == 1:
                        self.enable_stitch_list.append(enabled_stitch_velocity)
                        enabled_stitch_velocity_flag = True
                    else:
                        enabled_stitch_velocity_flag = False

                if msg['superpilot_enabled'] is not None:
                    superpilot_enabled = msg['superpilot_enabled']
                    if superpilot_enabled == 1:
                        superpilot_enabled_flag = True
                    else:
                        superpilot_enabled_flag = False

                if msg['dbw_enabled'] is not None:
                    dbw_enabled = msg['dbw_enabled']
                    if superpilot_enabled == 1:
                        dbw_enabled_flag = True
                    else:
                        dbw_enabled_flag = False
                is_valid_data = enabled_stitch_velocity_flag == True and dbw_enabled_flag == True
                if msg['v_error'] is not None:
                    v_error = msg['v_error']
                    self.v_error_total_list.append(v_error)
                    if is_valid_data is True:
                        v_error_list.append(v_error)

                if (msg['a_report']
                        is not None) and (msg['mpc_acceleration_cmd']
                                          is not None):
                    a_error = msg['mpc_acceleration_cmd'] - msg['a_report']
                    self.a_error_total_list.append(a_error)
                    if is_valid_data is True:
                        a_error_list.append(a_error)

                if msg['driveline_inertia_factor_11'] is not None:
                    driveline_inertia_factor_11 = msg[
                        'driveline_inertia_factor_11']
                    self.driveline_inertia_factor_11_total_list.append(
                        driveline_inertia_factor_11)
                    if is_valid_data is True:
                        driveline_inertia_factor_11_list.append(
                            driveline_inertia_factor_11)

                if msg['driveline_inertia_factor_12'] is not None:
                    driveline_inertia_factor_12 = msg[
                        'driveline_inertia_factor_12']
                    self.driveline_inertia_factor_12_total_list.append(
                        driveline_inertia_factor_12)
                    if is_valid_data is True:
                        driveline_inertia_factor_12_list.append(
                            driveline_inertia_factor_12)

            if len(v_error_list) != 0:
                self.avg_v_error = \
                    [(self.avg_v_error[0] * self.avg_v_error[1] + sum(v_error_list)) / (self.avg_v_error[1] + len(v_error_list)),
                    self.avg_v_error[1] + len(v_error_list)]
                self.v_rmse = \
                    [math.sqrt((np.array(self.v_rmse[0])**2 * self.v_rmse[1] + np.sum(np.array(v_error_list)**2)) / (self.v_rmse[1] + len(v_error_list))), self.v_rmse[1] + len(v_error_list)]

            if len(a_error_list) != 0:
                self.avg_a_error = \
                    [(self.avg_a_error[0] * self.avg_a_error[1] + sum(a_error_list)) / (self.avg_a_error[1] + len(a_error_list)),
                    self.avg_a_error[1] + len(a_error_list)]
                self.a_rmse = \
                    [math.sqrt((np.array(self.a_rmse[0])**2 * self.a_rmse[1] + np.sum(np.array(a_error_list)**2)) / (self.a_rmse[1] + len(a_error_list))), self.a_rmse[1] + len(a_error_list)]

            if len(driveline_inertia_factor_11_list) != 0:
                self.avg_driveline_inertia_factor_11 = \
                [(self.avg_driveline_inertia_factor_11[0] *
                self.avg_driveline_inertia_factor_11[1] +
                sum(driveline_inertia_factor_11_list)) /
                (self.avg_driveline_inertia_factor_11[1] +
                 len(driveline_inertia_factor_11_list)),
                self.avg_driveline_inertia_factor_11[1] +
                len(driveline_inertia_factor_11_list)]

            if len(driveline_inertia_factor_12_list) != 0:
                self.avg_driveline_inertia_factor_12 = \
                [(self.avg_driveline_inertia_factor_12[0] *
                self.avg_driveline_inertia_factor_12[1] +
                sum(driveline_inertia_factor_12_list)) /
                (self.avg_driveline_inertia_factor_12[1] +
                 len(driveline_inertia_factor_12_list)),
                self.avg_driveline_inertia_factor_12[1] +
                len(driveline_inertia_factor_12_list)]
        self.stitch_precent = len(self.enable_stitch_list) / len(
            self.stitch_list)
        self.filtered_driveline_inertia_factor_11_total_list = [
            n
            for n in lon_metric_analyzer.driveline_inertia_factor_11_total_list
            if n > 0
        ]
        self.filtered_driveline_inertia_factor_12_total_list = [
            n
            for n in lon_metric_analyzer.driveline_inertia_factor_12_total_list
            if n > 0
        ]

    def plot_function(self):
        logger.info("plotting")
        if 'fontsize' in FIG_CONFIG:
            matplotlib.rcParams.update({'font.size': FIG_CONFIG['fontsize']})
        if 'figsize' in FIG_CONFIG and 'dpi' in FIG_CONFIG:
            fig = plt.figure(num=0,
                             figsize=FIG_CONFIG['figsize'],
                             dpi=FIG_CONFIG['dpi'])
        fig.canvas.set_window_title('test')
        plt.subplot(3, 1, 1)
        plt.plot(self.filtered_driveline_inertia_factor_11_total_list,
                 label='11 gear factor')
        plt.plot(self.filtered_driveline_inertia_factor_12_total_list,
                 label='12 gear factor')
        plt.legend()
        plt.grid()

        plt.subplot(3, 1, 2)
        plt.plot(self.v_error_total_list, label='v_error')
        plt.legend()
        plt.grid()

        plt.subplot(3, 1, 3)
        plt.plot(self.a_error_total_list, label='a_error')
        plt.legend()
        plt.grid()
        plt.show()


if __name__ == '__main__':
    CH_conf = {
        "user": "plus_viewer",
        "password": "ex4u1balSAeR68uC",
        "server_host": "clickhouse-proxy-cn.plusai.co",
        "port": "9090",
        "db": "bagdb"
    }
    connection = 'clickhouse://{user}:{password}@{server_host}:{port}/{db}'.format(
        **CH_conf)
    engine = create_engine(connection,
                           pool_size=100,
                           pool_recycle=3600,
                           pool_timeout=20)
    session = make_session(engine)
    
    config_dicts = []
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-11 11:00:00.00',
    #     "end_time": '2023-04-11 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-11 13:15:00.00',
    #     "end_time": '2023-04-11 14:45:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-11 15:45:00.00',
    #     "end_time": '2023-04-11 16:15:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-12 11:00:00.00',
    #     "end_time": '2023-04-12 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-12 13:00:00.00',
    #     "end_time": '2023-04-12 13:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-12 14:30:00.00',
    #     "end_time": '2023-04-12 15:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-13 10:30:00.00',
    #     "end_time": '2023-04-13 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-13 12:45:00.00',
    #     "end_time": '2023-04-13 13:15:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-13 14:00:00.00',
    #     "end_time": '2023-04-13 14:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-14 10:00:00.00',
    #     "end_time": '2023-04-14 10:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-14 12:15:00.00',
    #     "end_time": '2023-04-14 12:45:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-14 14:30:00.00',
    #     "end_time": '2023-04-14 15:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-15 10:30:00.00',
    #     "end_time": '2023-04-15 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-15 12:45:00.00',
    #     "end_time": '2023-04-15 13:15:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-15 13:30:00.00',
    #     "end_time": '2023-04-15 14:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-17 14:00:00.00',
    #     "end_time": '2023-04-17 14:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-17 14:30:00.00',
    #     "end_time": '2023-04-17 15:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-17 15:00:00.00',
    #     "end_time": '2023-04-17 15:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-18 11:30:00.00',
    #     "end_time": '2023-04-18 12:00:00.00',
    #     "time_step": 80
    # }
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-18 12:00:00.00',
    #     "end_time": '2023-04-18 12:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-18 13:00:00.00',
    #     "end_time": '2023-04-18 13:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-19 10:00:00.00',
    #     "end_time": '2023-04-19 10:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-19 10:30:00.00',
    #     "end_time": '2023-04-19 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-19 11:00:00.00',
    #     "end_time": '2023-04-19 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-20 10:30:00.00',
    #     "end_time": '2023-04-20 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-20 11:00:00.00',
    #     "end_time": '2023-04-20 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-20 11:30:00.00',
    #     "end_time": '2023-04-20 12:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-21 10:30:00.00',
    #     "end_time": '2023-04-21 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-21 11:00:00.00',
    #     "end_time": '2023-04-21 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-21 11:30:00.00',
    #     "end_time": '2023-04-21 12:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-23 10:30:00.00',
    #     "end_time": '2023-04-23 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-23 11:00:00.00',
    #     "end_time": '2023-04-23 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-23 11:30:00.00',
    #     "end_time": '2023-04-23 12:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-24 10:30:00.00',
    #     "end_time": '2023-04-24 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-24 11:00:00.00',
    #     "end_time": '2023-04-24 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-24 11:30:00.00',
    #     "end_time": '2023-04-24 12:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-25 10:30:00.00',
    #     "end_time": '2023-04-25 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-25 11:00:00.00',
    #     "end_time": '2023-04-25 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-25 12:00:00.00',
    #     "end_time": '2023-04-25 12:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-26 10:30:00.00',
    #     "end_time": '2023-04-26 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-26 11:00:00.00',
    #     "end_time": '2023-04-26 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-26 12:00:00.00',
    #     "end_time": '2023-04-26 12:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-27 10:30:00.00',
    #     "end_time": '2023-04-27 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-27 11:00:00.00',
    #     "end_time": '2023-04-27 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-27 12:00:00.00',
    #     "end_time": '2023-04-27 12:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-28 10:30:00.00',
    #     "end_time": '2023-04-28 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-28 11:00:00.00',
    #     "end_time": '2023-04-28 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-28 12:00:00.00',
    #     "end_time": '2023-04-28 12:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-29 10:00:00.00',
    #     "end_time": '2023-04-29 10:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-29 13:30:00.00',
    #     "end_time": '2023-04-29 14:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-29 19:30:00.00',
    #     "end_time": '2023-04-29 20:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-30 10:30:00.00',
    #     "end_time": '2023-04-30 11:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-30 11:00:00.00',
    #     "end_time": '2023-04-30 11:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-30 12:00:00.00',
    #     "end_time": '2023-04-30 12:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    config_dict = {
        "vehicle_name": 'pdb-l4e-b0007',
        "start_time": '2023-05-01 10:00:00.00',
        "end_time": '2023-05-01 10:30:00.00',
        "time_step": 80
    }
    config_dicts.append(config_dict)
    config_dict = {
        "vehicle_name": 'pdb-l4e-b0007',
        "start_time": '2023-05-01 11:30:00.00',
        "end_time": '2023-05-01 12:00:00.00',
        "time_step": 80
    }
    config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-05-01 12:00:00.00',
    #     "end_time": '2023-05-01 12:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-05-02 10:40:00.00',
    #     "end_time": '2023-05-02 11:10:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-05-02 13:00:00.00',
    #     "end_time": '2023-05-02 13:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-05-02 13:30:00.00',
    #     "end_time": '2023-05-02 14:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
   
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-05-01 05:30:00.00',
    #     "end_time": '2023-05-01 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-05-02 06:00:00.00',
    #     "end_time": '2023-05-02 17:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-20 06:00:00.00',
    #     "end_time": '2023-04-20 17:30:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-23 05:30:00.00',
    #     "end_time": '2023-04-23 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-24 05:30:00.00',
    #     "end_time": '2023-04-24 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-25 05:30:00.00',
    #     "end_time": '2023-04-25 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-26 05:30:00.00',
    #     "end_time": '2023-04-26 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-27 05:30:00.00',
    #     "end_time": '2023-04-27 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-28 05:30:00.00',
    #     "end_time": '2023-04-28 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-04-30 05:30:00.00',
    #     "end_time": '2023-04-30 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)
    # config_dict = {
    #     "vehicle_name": 'pdb-l4e-b0007',
    #     "start_time": '2023-05-01 05:30:00.00',
    #     "end_time": '2023-05-01 18:00:00.00',
    #     "time_step": 80
    # }
    # config_dicts.append(config_dict)

    
    COLUMNS = ['vehicle_name', 'start_time','end_time', 'avg_v_err','v_rmse','avg_a_err','a_rmse']
    df = pd.DataFrame(columns=COLUMNS)
    
    for config_dict in config_dicts:
        
        lon_metric_analyzer = LonMetricAnalysis(session=session,
                                                config_dict=config_dict)
        lon_metric_analyzer.get_query()
        print("========={} : {} to {} with {} frames=========".format(config_dict.get("vehicle_name"), 
                                                                      config_dict.get("start_time"),
                                                                      config_dict.get("end_time"), 
                                                                      lon_metric_analyzer.avg_v_error[1]))

        print("avg v_error: {}, v_rmse: {}".format(lon_metric_analyzer.avg_v_error[0], lon_metric_analyzer.v_rmse[0]))
        print("avg a_error: {}, a_rmse: {}".format(lon_metric_analyzer.avg_a_error[0], lon_metric_analyzer.a_rmse[0]))
        df_new = {COLUMNS[0]:config_dict.get("vehicle_name"),
                  COLUMNS[1]:config_dict.get("start_time"),
                  COLUMNS[2]:config_dict.get("end_time"),
                  COLUMNS[3]:lon_metric_analyzer.avg_v_error[0],
                  COLUMNS[4]:lon_metric_analyzer.v_rmse[0],
                  COLUMNS[5]:lon_metric_analyzer.avg_a_error[0],
                  COLUMNS[6]:lon_metric_analyzer.a_rmse[0],}
        
        dir_path = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(dir_path, 'output')
        if not os.path.exists(output_dir):  
            os.makedirs(output_dir)
        df = df.append(df_new,ignore_index=True)
    
    df.to_csv(os.path.join(output_dir, 'result.csv'))
    # lon_metric_analyzer.plot_function()
