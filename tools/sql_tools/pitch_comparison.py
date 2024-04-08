#!/usr/bin/env python

from datetime import datetime, timedelta
from pluspy import db_utils
import pandas as pd
import argparse
import os
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from collections import deque, OrderedDict 
import yaml


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
"""     SELECT ts,
    JSONExtractFloat(vehicle_status, 'mapPitch') as map_pitch,
    JSONExtractFloat(vehicle_status, 'pitchFromZDifferentiator') as pitch_from_z_differentiator
    FROM bagdb.bag_messages
    WHERE vehicle=:vehicle
    and ts between :start_ts and :end_ts
    and dbw_enabled = 1
    ORDER by ts """
)

QUERY_COLUMNS = ['ts','map_pitch','pitch_from_z_differentiator']

def data_query(vehicle_name, start_ts, end_ts):  
    print("data query start")    
    temp_ts = start_ts
    delta_time = timedelta(seconds=50)

    df = pd.DataFrame(columns=QUERY_COLUMNS)
    while temp_ts + delta_time < end_ts:
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": temp_ts + delta_time}).fetchall()
            new_df = pd.DataFrame(df_new,columns=QUERY_COLUMNS)
            # if(len(df)!=0 and len(new_df) == 0):
            #     break
            df = pd.concat([df, new_df],ignore_index=True)
        temp_ts += delta_time
        print("query interval {}/{}: data length = {}".format(temp_ts + timedelta(hours=8), end_ts + timedelta(hours=8), len(df)))
        # print(new_df)

    if (temp_ts + delta_time > end_ts):
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": end_ts}).fetchall()
            df = pd.concat([df, pd.DataFrame(df_new,columns=QUERY_COLUMNS)],ignore_index=True)
    print("query interval {}/{}: data length = {}".format(temp_ts + timedelta(hours=8), end_ts + timedelta(hours=8), len(df)))
    print("data query finish")

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
         
    print(len(df))
    
    # df.to_csv(os.path.join(args.output_dir,args.data_file_name))
    return df


def data_process(args, df, READ_FROM_FILE = False):
    if(READ_FROM_FILE):
        data = pd.read_csv(os.path.join(args.output_dir,args.data_file_name))
    else:
        data = df

    if(len(df)==0):
        return None

    pitch_error = (data['map_pitch'] - data['pitch_from_z_differentiator'])*180/math.pi
    return math.sqrt(sum([x** 2 for x in pitch_error]) / len(pitch_error))
    

def main(args):
    with open(args.config_file, 'r') as f:
        config = yaml.load(f)

    vehicle_names = np.array(config['vehicles'])
    time_intervals = np.array(config['time_interval'])
    df = pd.DataFrame()
    for vehicle_name in vehicle_names:
        print('====================query data of {}========================'.format(vehicle_name))
        for time_interval in time_intervals:
            start_ts = datetime.strptime(time_interval[0], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
            end_ts = datetime.strptime(time_interval[1], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
            data = data_query(vehicle_name, start_ts, end_ts)
            if(len(data) == 0):
                continue
            data = data.interpolate(method='linear')
            data = data.dropna()
            re = data_process(args, data, False)
            time_interval = str(time_interval[0])+' - '+str(time_interval[1])
            print('pitch rmse:{}'.format(re))
            df.to_csv(args.data_file_name_1, mode='a', header=True)
            df = df.append(pd.DataFrame([{'time':time_interval,'rsme':re}]),ignore_index=True)
    print(df)
    df.to_csv(args.data_file_name, mode='a', header=True)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    parser.add_argument('--output-dir', default=os.path.join(os.environ['HOME'],'data/bag_msg'))
    parser.add_argument('--config-file', default='config/pitch_comparison_config.yaml', type=str)
    parser.add_argument('--data-file-name', default='pitch_data.csv', type=str)
    parser.add_argument('--data-file-name_1', default='pitch_data_1.csv', type=str)
    parser.add_argument('--repull-data', default=False, type=bool)
    
    args = parser.parse_args()
    main(args)