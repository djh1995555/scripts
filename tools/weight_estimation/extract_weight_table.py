#!/usr/bin/env python
from datetime import datetime, timedelta
from pluspy import db_utils
from control import control_command_pb2
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

def main(args):
    df = pd.read_csv(args.weight_gt_extracted_filepath)
    result = {}
    result['start_time'] = []
    result['end_time'] = []
    result[args.vehicle_name] = []
    for i in range(len(df)):
        item = OrderedDict()
        start_date = df['start_date'].iloc[i]
        start_time = df['start_time'].iloc[i]
        start_date = start_date.replace('/','-')
        result['start_time'].append('{} {}:00'.format(start_date, start_time))
        end_date = df['end_date'].iloc[i]
        end_time = df['end_time'].iloc[i]
        end_date = end_date.replace('/','-')        
        result['end_time'].append('{} {}:00'.format(end_date, end_time))
        result[args.vehicle_name].append(df['weight_gt'].iloc[i])
    print(result)
    result_df = pd.DataFrame(result)
    result_df.to_csv(os.path.join(output_dir, 'weight_gt_extracted_reorganized.csv'),columns = ['start_time',
                                                                                                        'end_time',
                                                                                                        args.vehicle_name])
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('weight atble reader')
    control_cmd = control_command_pb2.ControlCommand()
    root_dir = os.path.dirname(os.path.abspath(__file__))
    weight_gt_extracted_filepath = os.path.join(root_dir, 'weight_gt_extracted/weight_gt_extracted_original.csv')
    output_dir = os.path.join(root_dir, 'weight_gt_extracted')
    parser.add_argument('--weight-gt-extracted-filepath', default=weight_gt_extracted_filepath, type=str)
    parser.add_argument('--vehicle-name', default='', type=str)
    args = parser.parse_args()

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    print('output_dir:{}'.format(output_dir))
    
    main(args)