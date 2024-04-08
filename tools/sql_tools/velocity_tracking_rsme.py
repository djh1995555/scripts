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
    JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'vError') as v_error,
    JSONExtractFloat(vehicle_dbw_reports, 'steeringReport', 'speed') as speed,
    dbw_enabled as dbw_signal
    FROM bagdb.bag_messages
    WHERE vehicle=:vehicle
    and ts between :start_ts and :end_ts
    and dbw_enabled = 1
    and ABS(JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'vError')) > 0.0
    ORDER by ts """
)

QUERY_COLUMNS = ['ts','v_error','speed','dbw_signal']

def data_query(vehicle_name, start_ts, end_ts):  
    print("data query start")    
    temp_ts = start_ts
    delta_time = timedelta(minutes=5)

    df = pd.DataFrame(columns=QUERY_COLUMNS)
    while temp_ts + delta_time < end_ts:
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": temp_ts + delta_time}).fetchall()
            new_df = pd.DataFrame(df_new,columns=QUERY_COLUMNS)
            if(len(df)!=0 and len(new_df) == 0):
                break
            df = pd.concat([df, new_df],ignore_index=True)
        temp_ts += delta_time
        print("query interval {}/{}: data length = {}".format(temp_ts + timedelta(hours=8), end_ts + timedelta(hours=8), len(df)))

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

    return math.sqrt(sum([x** 2 for x in df['v_error']]) / len(df['v_error']))
    

def main(args):
    # vehicle_name = 'pdb-l4e-b0005'
    # start_ts = '2022-11-08 02:00:00'
    # end_ts = '2022-11-10 09:59:59'
    with open(args.time_list, 'r') as f:
        time_list = yaml.load(f)

    # vehicle_names = ['j7-l4e-LFWSRXSJ7M1F48985']
    vehicle_names = ['pdb-l4e-b0005']
    base_time = np.array(time_list['base_time_list'])
    pr_time = np.array(time_list['pr_time_list'])
    time_list = {'base':base_time, 'pr':pr_time}  
    df = pd.DataFrame()
    for vehicle_name in vehicle_names:
        print('====================query data of {}========================'.format(vehicle_name))
        for test, sub_time_list in time_list.items():
            for time_pairs in sub_time_list:
                start_ts = datetime.strptime(time_pairs[0], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
                end_ts = datetime.strptime(time_pairs[1], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
                data = data_query(vehicle_name, start_ts, end_ts)
                re = data_process(args, data, False)
                time_interval = str(time_pairs[0])+' - '+str(time_pairs[1])
                print("{}: velocity tracking rsme = {}".format(time_interval,re))
                df = df.append(pd.DataFrame([{'test':test,'time':time_interval,'rsme':re}]),ignore_index=True)

    print(df)
    df.to_csv(args.data_file_name, mode='a', header=True)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    parser.add_argument('--output-dir', default=os.path.join(os.environ['HOME'],'data/bag_msg'))
    parser.add_argument('--time-list', default='config/time_list.yaml', type=str)
    parser.add_argument('--data-file-name', default='data.csv', type=str)
    parser.add_argument('--repull-data', default=False, type=bool)
    
    args = parser.parse_args()
    main(args)