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
import shutil

FASTBAG_SIZE_THESHOLD = 10 * 1024 ** 3
DURATION_THRESHOLD = 500
AUTO_RATIO_THRESHOLD = 95
DISTANCE_THRESHOLD = 20000


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
    
def data_query(bag_name, df):  
    with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
        df_new = db_session.execute(SQL_BAG_MSG,{"bag_name": bag_name}).fetchall()
        new_df = pd.DataFrame(df_new,columns=QUERY_COLUMNS) 
        filter_data(new_df)
    
    df = pd.concat([df, new_df],ignore_index=True)
    return df
        
def main(args):
    folder_path = args.bag_list_folder
    for bag_list in os.listdir(folder_path):
        bag_list_name, file_ext = os.path.splitext(bag_list)
        print('====================query data of {}========================'.format(bag_list_name))
        bag_list = os.path.join(folder_path, bag_list)
        df = pd.DataFrame(columns=QUERY_COLUMNS)
        if os.path.isfile(bag_list):
            with open(bag_list, 'r') as file:
                for bag_name in file:
                    bag_name = bag_name.replace('\n','.bag')
                    print('query bag path of {}'.format(bag_name))
                    df = data_query(bag_name, df)
                    
            name_and_path = df[['bag_name','bag_path']]
            output_dir = os.path.join(file_dirname,'output')
            if os.path.exists(output_dir):
                shutil.rmtree(output_dir)
            os.makedirs(output_dir)
            output_file_name = os.path.join(output_dir,bag_list_name+'.csv')
            name_and_path.to_csv(output_file_name) 
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    file_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--bag-list-folder', default=os.path.join(file_dirname,'bag_lists'), type=str)
    parser.add_argument('--output-file-name', default='bag_name_and_path.csv', type=str)
    
    args = parser.parse_args()
    main(args)