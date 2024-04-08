#!/usr/bin/env python

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
"""      SELECT ts,bag_name
    FROM bagdb.bag_messages
    WHERE vehicle=:vehicle
    and ts between :start_ts and :end_ts
    ORDER by ts """
)

QUERY_COLUMNS = ['ts','bag_name']
   
def data_query(vehicle_name, start_ts, end_ts, delta_min = 5):  
    print("data query start")    
    temp_ts = start_ts
    delta_time = timedelta(minutes = delta_min)
    sql_range = timedelta(milliseconds = 1000)
    df = pd.DataFrame(columns=QUERY_COLUMNS)
    while temp_ts < end_ts:
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": temp_ts + sql_range}).fetchall()
            new_df = pd.DataFrame(df_new,columns=QUERY_COLUMNS)
            retry_num = 0
            while(len(df)!=0 and len(new_df) == 0 and retry_num < 20):
                retry_num += 1
                df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                    "start_ts": temp_ts + retry_num * sql_range,
                                                    "end_ts": temp_ts + (retry_num + 1) * sql_range}).fetchall()
            if(len(df)!=0 and len(new_df) == 0):
                return
            
            df = pd.concat([df, new_df],ignore_index=True)
            
        
        print("query interval {}({})/{}({}): data length = {}".format(
            temp_ts + timedelta(hours=8),
            float(time.mktime((temp_ts + timedelta(hours=8)).timetuple())),
            temp_ts + delta_time + timedelta(hours=8),
            float(time.mktime((temp_ts + delta_time + timedelta(hours=8)).timetuple())),
            len(df)))
        temp_ts += delta_time
        
    df = df.drop_duplicates(subset=['bag_name'], keep='first')
    print(df)
    return df  

def save_data(df):
    bag_link = list(["https://bagdb-cn.plus.ai/plusview/{}".format(bag_name) for bag_name in df['bag_name']])
    
    with open(args.output_file_name,'w') as f:
        yaml.dump(data=bag_link, stream=f, allow_unicode=True)
        
def main(args):
    # with open(args.config, 'r') as f:
    #     config = yaml.load(f)
    # vehicle_names = np.array(config['vehciles']) 
    # time_interval = np.array(config['time_interval'])   
    
    time_interval = ['2023-05-02 10:00:00', '2023-05-02 17:59:59']     
    vehicle_names = ['pdb-l4e-b0007'] 
    df = pd.DataFrame()
    delta_min = 20
    for vehicle_name in vehicle_names:
        print('====================query data of {}========================'.format(vehicle_name))
        start_ts = datetime.strptime(time_interval[0], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
        end_ts = datetime.strptime(time_interval[1], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
        data = data_query(vehicle_name, start_ts, end_ts, delta_min)
        
    save_data(data)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    parser.add_argument('--config', default='config/sql_config.yaml', type=str)
    parser.add_argument('--output-file-name', default='bag_link.yaml', type=str)
    
    args = parser.parse_args()
    main(args)