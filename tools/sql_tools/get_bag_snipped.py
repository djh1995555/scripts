#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from datetime import datetime, timedelta
from pluspy import db_utils
import pandas as pd
import argparse
import os
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from collections import deque, OrderedDict 
import yaml
import requests
import subprocess


FASTBAG_SIZE_THESHOLD = 1 * 1024 ** 3
DURATION_THRESHOLD = 50
AUTO_RATIO_THRESHOLD = 0
DISTANCE_THRESHOLD = 0


DB_CONF = {
    "clickhouse": {
        "driver": "clickhouse+native",
        "port": 9000,
        "database": "bagdb",
        "host": "clickhouse-cn",
        "user": "plus_viewer",
        "password": "ex4u1balSAeR68uC",
    },
}

SQL_BAG_MSG = (
"""      SELECT start_time, end_time, bag_name, fastbag_path, fastbag_size, bag_source, distance, auto_distance
    FROM bags b
    WHERE bag_name=:bag_name"""
)

QUERY_COLUMNS = ['start_time','end_time','bag_name','bag_path','fastbag_size', 'bag_source', 'distance', 'auto_distance']

def filter_data(df): 
    for i in range(len(df)):
        if( df['end_time'][i] - df['start_time'][i] < DURATION_THRESHOLD or 
            df['fastbag_size'][i] < FASTBAG_SIZE_THESHOLD or
            df['bag_source'][i] != 'offline' or 
            df['distance'][i] < DISTANCE_THRESHOLD or
            df['auto_distance'][i] < df['distance'][i] * AUTO_RATIO_THRESHOLD /100):
            df.drop(index = [i],inplace=True)
    
def data_query(bag_name, start_ts, end_ts, df, args, target_file_name, failure_list):  
    with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
        data = db_session.execute(SQL_BAG_MSG,{"bag_name": bag_name}).fetchall()
    
    data = pd.DataFrame(data,columns=QUERY_COLUMNS) 
    if(len(data['bag_path'])==0):
        failure_list.append(bag_name)
    else:
        bag_path = data['bag_path'][0]
        bag_link = get_link(bag_path, start_ts, end_ts)
        new_df = pd.DataFrame([{'bag_name':data['bag_name'][0],'bag_path':bag_path,'bag_link':bag_link}])
            
        df = pd.concat([df, new_df],ignore_index=True)
        response = requests.get(bag_link)
        filename = os.path.join(args.target_bag_folder, '{}.db'.format(target_file_name))
        
        with open(filename, 'wb') as file:
            file.write(response.content)

    return df

def get_link(bag_path, start_ts, end_ts):
    link = "https://bagdb-cn-proxy.plusai.co:8443/bagdb/snips/fastsnips?compress=true&end={}&fastbag_path=https://bagdb-cn-proxy.plusai.co:8443/raw{}?\_x_user=jianhao.dong@plus.ai&start={}&topics=/control_v2/runtime&topics=/localization/odometry&topics=/localization/runtime&topics=/localization/state&topics=/perception/calibrations&topics=/perception/lane_path&topics=/perception/obstacles&topics=/planning/lead_info&topics=/planning/trajectory&topics=/imu/data&topics=/navsat/odom&topics=/novatel_data/inspva&topics=/novatel_data/inspvax&topics=/bumper_radar/status_report&topics=/control/status_report&topics=/dms_cam_node/status_report&topics=/event_recorder/status_report&topics=/gmsl_cameras/status_report&topics=/lane/status_report&topics=/livox/status_report&topics=/localization/status_report&topics=/obstacle/status_report&topics=/planning/status_report&topics=/prediction/status_report&topics=/rear_radars/status_report&topics=/ublox/status_report&topics=/vehicle/status_report&topics=/watchdog/status_report&topics=/vehicle/button_status&topics=/vehicle/control_cmd&topics=/vehicle/dbw_reports&topics=/vehicle/engage_cmd&topics=/vehicle/misc_1_report&topics=/vehicle/status&topics=/vehicle/truck_state&topics=/app_watchdog/runtime&topics=/dms_cam_node/runtime&topics=/faw_can_node/runtime&topics=/fusion_object_tracker/runtime&topics=/gmsl_cam/runtime&topics=/livox/runtime&topics=/plus/odom&topics=/plus_scenario_planner/runtime&topics=/plusai_record/runtime&topics=/recorder/runtime&topics=/system_metrics&topics=/system_metrics/runtime&topics=/uds_server/runtime&topics=/watchdog/current_state".format(end_ts, bag_path, start_ts)
    return link
     
def main(args):
    folder_path = args.bag_list_folder
    failure_list = []
    if not os.path.exists(args.target_bag_folder):
        os.makedirs(args.target_bag_folder) 
    for bag_list in os.listdir(folder_path):
        bag_list_name, file_ext = os.path.splitext(bag_list)
        print('====================query data of {}========================'.format(bag_list_name))
        bag_list = os.path.join(folder_path, bag_list)
        df = pd.DataFrame(columns=['bag_name','bag_path','bag_link'])
        if os.path.isfile(bag_list):
            with open(bag_list, 'r') as file:
                for bag_full_name in file:
                    bag_name_with_ts, file_ext = os.path.splitext(bag_full_name)
                    index = bag_name_with_ts.find('202')
                    target_file_name = bag_name_with_ts[index:]
                    bag_name, ts = target_file_name.rsplit('_',1)
                    bag_name = bag_name + '.bag'
                    start_ts, end_ts = ts.split('to')
                    print(bag_name, start_ts, end_ts)
                    print('query bag path of {}'.format(bag_name))
                    df = data_query(bag_name, start_ts, end_ts, df, args, target_file_name, failure_list)
                    
            output_dir = os.path.join(file_dirname,'output')
            if not os.path.exists(output_dir):
                os.makedirs(output_dir) 
            output_file_name = os.path.join(output_dir,bag_list_name+'.csv')
            df.to_csv(output_file_name) 
            print('faliure list:{}'.format(failure_list))
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    file_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--bag-list-folder', default=os.path.join(file_dirname,'bag_snipped'), type=str)
    parser.add_argument('--target-bag-folder', default=os.path.join(os.environ['HOME'],'bags','scenario'), type=str)
    
    args = parser.parse_args()
    main(args)