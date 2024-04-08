#!/usr/bin/env python

from datetime import datetime, timedelta
import shutil
import time
from pluspy import db_utils
from control import control_command_pb2
from control import vehicle_state_pb2
import pandas as pd
import rosbag
import fastbag
import rospy
import argparse
import os
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from collections import deque, OrderedDict 
import yaml
import psycopg2
import plotly.subplots as sp
import plotly.graph_objects as go   
from pandas.plotting import register_matplotlib_converters

WEIGHT_UNIT = 1000

def read_bag(bag_file_path, bag_name):
    print('read {}'.format(bag_name))
    split_result = bag_name.split('_')
    date = split_result[0]
    vehicle = split_result[1]
    date = date[:8]
    base_bag_name = '{}_{}_{}.db'.format(split_result[0], split_result[1], split_result[2])
    bag = rosbag.Bag(bag_file_path)
    weight_estimated = []
    weight_from_sensor = []
    weight_estimated_old = []
    total_weight = []
    control_pitch = []
    pitch_from_z = []
    speed = []
    
    target_row = original_df[original_df['bag_name'] == base_bag_name]
    # print(target_row)
    weight_gt = target_row.iloc[0]['weight_gt']
    for topic, msg, bag_time in bag.read_messages(topics=['/vehicle/control_cmd','/vehicle/status']):
        if(topic == '/vehicle/control_cmd'):
            control_cmd.ParseFromString(msg.data)
            weight_estimated.append(float(control_cmd.debug_cmd.weight_estimated))
            weight_from_sensor.append(float(control_cmd.debug_cmd.weight_calibrated))
            weight_estimated_old.append(float(control_cmd.debug_cmd.total_weight_estimation))
        elif(topic == '/vehicle/status'):
            vehicle_state.ParseFromString(msg.data)
            total_weight.append(float(vehicle_state.total_weight))    
            control_pitch.append(float(vehicle_state.pitch))    
            pitch_from_z.append(float(vehicle_state.pitch_from_z_differentiator))
            speed.append(float(vehicle_state.v))        
            
        
    if(len(weight_estimated)==0):
        print('len of weight_estimated = 0')
        return None    
    
    mean_of_weight_estimated = sum(weight_estimated)/len(weight_estimated)
    if(mean_of_weight_estimated==0):
        print('mean_of_weight_estimated = 0')
        return None
        
    error_of_end_value = (weight_estimated[-1] - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100
    error_of_mean_value = (mean_of_weight_estimated - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    if(len(weight_from_sensor)==0):
        mean_of_weight_from_sensor = np.nan
    else:
        mean_of_weight_from_sensor = sum(weight_from_sensor)/len(weight_from_sensor)
    if(np.isnan(mean_of_weight_from_sensor)):
        error_of_sensor_value = np.nan
    else:
        error_of_sensor_value = (mean_of_weight_from_sensor - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    if(len(total_weight)==0):
        mean_of_total_weight = np.nan
    else:
        mean_of_total_weight = sum(total_weight)/len(total_weight)
    if(np.isnan(mean_of_total_weight)):
        error_of_total_weight = np.nan
    else:
        error_of_total_weight = (mean_of_total_weight - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    if(len(weight_estimated_old)==0):
        mean_of_weight_estimated_old = np.nan
    else:
        mean_of_weight_estimated_old = sum(weight_estimated_old)/len(weight_estimated_old)
    if(np.isnan(mean_of_weight_estimated_old)):
        error_of_estimation_old_value = np.nan
    else:
        error_of_estimation_old_value = (mean_of_weight_estimated_old - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    differences = np.array(control_pitch) -  np.array(pitch_from_z)
    squared_diff = differences ** 2
    mean_squared_diff = np.mean(squared_diff)
    pitch_rmse = np.sqrt(mean_squared_diff)
    mean_speed = sum(speed)/len(speed)
    re = pd.DataFrame([{'bag_name':'{}.db'.format(bag_name),
                        'bag_filepath':bag_file_path,
                        'vehicle':vehicle,
                        'date':str(date),
                        'start_time':datetime.fromtimestamp(bag.get_start_time()) + timedelta(hours=8),
                        'end_time': datetime.fromtimestamp(bag.get_end_time()) + timedelta(hours=8),
                        'weight_gt': weight_gt,
                        'end_of_weight_estimated' : weight_estimated[-1],
                        'mean_of_weight_estimated' : mean_of_weight_estimated,
                        'mean_of_weight_from_sensor' : mean_of_weight_from_sensor,
                        'mean_of_total_weight' : mean_of_total_weight,
                        'mean_of_weight_estimated_old' : mean_of_weight_estimated_old,
                        'error_of_end_value' : error_of_end_value,
                        'error_of_mean_value' : error_of_mean_value,
                        'error_of_sensor_value' : error_of_sensor_value,
                        'error_of_estimation_old_value' : error_of_estimation_old_value,
                        'error_of_total_weight' : error_of_total_weight,
                        'weather': target_row.iloc[0]['weather'],
                        'wind_heading': target_row.iloc[0]['wind_heading'],
                        'vehicle_heading': target_row.iloc[0]['vehicle_heading'],
                        'wind_speed': target_row.iloc[0]['wind_speed'],
                        'relative_speed': target_row.iloc[0]['relative_speed'],
                        'pitch_rmse': pitch_rmse,
                        'mean_speed':mean_speed
                        }])
    return re

def get_all_filepath(dir, file_list):
    for sub_file in os.listdir(dir):
        sub_full_filepath = os.path.join(dir, sub_file)
        if(os.path.isdir(sub_full_filepath)):
            file_list = get_all_filepath(sub_full_filepath, file_list)
        else:
            # print(sub_full_filepath.split('.')[-1])
            if(sub_full_filepath.split('.')[-1] == 'bag'):
                file_list.append(sub_full_filepath)
    
    return file_list
def main(args):
    result = pd.DataFrame()
    # for date in os.listdir(args.target_folder):
    #     date_dir = os.path.join(args.target_folder, date)
    #     # print('{}'.format(date_dir))
    #     for bag_name in os.listdir(date_dir):
    #         sub_dir = os.path.join(date_dir, bag_name)
    #         # print('{}'.format(sub_dir))
    #         if(os.path.isdir(sub_dir)):
    #             bag_file_path = os.path.join(sub_dir, 'replay_{}.bag'.format(bag_name))
    #             # print('{}'.format(bag_file_path))
    #             if(os.path.isfile(bag_file_path)):
    #                 re = read_bag(bag_file_path, bag_name)
    #                 if(re is not None):
    #                     result = result.append(re,ignore_index=True) 
    #             else:
    #                 print('{} is not existed'.format(bag_file_path))
    
    file_list = []
    file_list = get_all_filepath(args.target_folder, file_list)
    for file in file_list:
        bag_name = file.split('/')[-2]
        re = read_bag(file, bag_name)
        if(re is not None):
            result = result.append(re,ignore_index=True) 

    result.to_csv(os.path.join(args.target_folder,'weight_estimation_analysis_result.csv'), mode = 'w')
if __name__ == '__main__':
    parser = argparse.ArgumentParser('weight comparison')
    control_cmd = control_command_pb2.ControlCommand()
    vehicle_state = vehicle_state_pb2.VehicleState()
    root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
    original_df_path = os.path.join(root_dir, 'result/weight_estimation_analysis_result.csv')
    original_df = pd.read_csv(original_df_path)
    parser.add_argument('--target-folder', default='', type=str)
    args = parser.parse_args()
    current_time = str(datetime.now() + timedelta(hours = 8)).replace(' ','_').replace(':','-').replace('.','-')
    output_dir = args.target_folder
        
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    print('output_dir:{}'.format(output_dir))
    main(args)