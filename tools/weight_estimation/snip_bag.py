#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from datetime import datetime, timedelta
import shutil
from pluspy import db_utils
import pandas as pd
import argparse
import os
import re
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from collections import deque, OrderedDict 
import yaml
import psycopg2
import subprocess

def download_file(bag_file_path, start, end, output_name):
    url = 'https://bagdb.pluscn.cn:28443/bagdb/snips/fastsnips?compress=true&end={}&fastbag_path=https://bagdb.pluscn.cn:28443/raw{}?_x_user=jianhao.dong@plus.ai&start={}&topics=/vehicle/misc_1_report&topics=/navsat/odom&topics=/plus/odom&topics=/novatel_data/inspva&topics=/novatel_data/inspvax&topics=/imu/data&topics=/vehicle/dbw_enabled&topics=/app_watchdog/latency_report&topics=/app_watchdog/runtime&topics=/auto_calibration/status_report&topics=/bosch_radar_can_node/runtime&topics=/bumper_radar/radar_can_node/latency_report&topics=/bumper_radar/status_report&topics=/control_v2/runtime&topics=/control/status_report&topics=/controller_ros_node/latency_report&topics=/dispatcher_client/latency_report&topics=/dispatcher_client/runtime&topics=/dispatcher_server/latency_report&topics=/dispatcher_server/status_report&topics=/event_recorder/latency_report&topics=/event_recorder/status_report&topics=/fusion_object_tracker/latency_report&topics=/fusion_object_tracker/runtime&topics=/gmsl_cam/runtime&topics=/gmsl_cameras/latency_report&topics=/gmsl_cameras/status_report&topics=/hdi_map_lane_geometry_data&topics=/issue_recorder/latency_report&topics=/lane_detector/latency_report&topics=/lane_detector/runtime&topics=/lane/status_report&topics=/localization/fusion_debug_info&topics=/localization/gnss&topics=/localization/latency_report&topics=/localization/odometry&topics=/localization/runtime&topics=/localization/state&topics=/localization/status_report&topics=/localization/visual_geometry&topics=/notes&topics=/obstacle/status_report&topics=/ota_client_node/latency_report&topics=/ota_client_node/runtime&topics=/ota_client/status_report&topics=/perception/calibrations&topics=/perception/lane_intermediates&topics=/perception/lane_path&topics=/perception/obstacles&topics=/perception/ot_calibrations&topics=/planning/lead_info&topics=/planning/status_report&topics=/planning/trajectory&topics=/plus_scenario_planner/runtime&topics=/plus_voice_recorder/runtime&topics=/plusai_record_1698612782461260581/latency_report&topics=/plusai_record/runtime&topics=/prediction/latency_report&topics=/prediction/obstacles&topics=/prediction/runtime&topics=/prediction/status_report&topics=/radar_can_node/runtime&topics=/rear_radar/radar_can_node/latency_report&topics=/rear_radars/status_report&topics=/recorder/runtime&topics=/robo_sense_lidar_node/runtime&topics=/routing/latency_report&topics=/routing/routes&topics=/routing/runtime&topics=/rs_lidar/latency_report&topics=/rs_lidar/status_report&topics=/scenario_planner/latency_report&topics=/sense_dms_camera/latency_report&topics=/sense_dms_camera/status_report&topics=/side_radar/bosch_radar_can_node/latency_report&topics=/side_radars/status_report&topics=/system_metrics&topics=/system_metrics/latency_report&topics=/system_metrics/runtime&topics=/ublox_gps/runtime&topics=/ublox/latency_report&topics=/ublox/status_report&topics=/uds_server_node/latency_report&topics=/uds_server/runtime&topics=/vehicle_can_node/latency_report&topics=/vehicle_can_node/runtime&topics=/vehicle/button_status&topics=/vehicle/control_cmd&topics=/vehicle/dbw_reports&topics=/vehicle/info&topics=/vehicle/location&topics=/vehicle/navigation&topics=/vehicle/status&topics=/vehicle/status_report&topics=/vehicle/truck_state&topics=/watchdog/current_state&topics=/watchdog/status_report&topics=/ublox/esfalg_debug&topics=/ublox/navatt&topics=/ublox/navpvt&topics=/ublox/navsat'.format(end, bag_file_path, start)
    wget_command = ['wget', url, '-O', output_name]
    try:
        subprocess.check_call(wget_command)
        print('file is save in {}'.format(output_name))
    except subprocess.CalledProcessError as e:
        print('Download {} failed!'.format(e))
        
def main(args):
    df = pd.read_csv(args.bag_list_file)
    bag_snipped_names = []
    for bag_file_path in df['bag_filepath']:
        bag_snipped_name = bag_file_path.split('/')[5].split('.')[0]
        print('name:{}'.format(bag_file_path))
        output_name = os.path.join(args.output_dir, '{}_{}to{}.db'.format(bag_snipped_name,args.start,args.end))
        download_file(bag_file_path, args.start, args.end, output_name)
        bag_snipped_names.append(output_name)
    bag_snipped_names_df = pd.DataFrame(bag_snipped_names)
    bag_snipped_names_df.to_csv(os.path.join(file_dirname, 'bag_snipped_names.csv'))

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    file_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--bag-list-file', required=True, type=str)
    parser.add_argument('--output-dir', default=file_dirname, type=str)
    parser.add_argument('--start', default=0, type=int)
    parser.add_argument('--end', default=100, type=int)
    args = parser.parse_args()
    if not os.path.exists(args.output_dir):  
        os.makedirs(args.output_dir)
    
    main(args)