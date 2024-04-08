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

SECOND_TO_MICROSECOND = 1000000
VEHICLE_MASS = 43000
FRONTAL_AREA = 8.0
AIR_DENSITY = 1.225
GRAVITY_ACC = 9.81

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
    JSONExtractFloat(vehicle_dbw_reports, 'steeringReport', 'speed') as speed,
    JSONExtractFloat(vehicle_status, 'chassisAY') as a_report,
    JSONExtractFloat(vehicle_dbw_reports, 'throttleReport', 'pedalOutput') as throttle_output,
    JSONExtractFloat(vehicle_dbw_reports, 'brakeReport', 'pedalOutput') as brake_output,
    JSONExtractFloat(vehicle_status, 'pitch') as road_pitch,
    JSONExtractFloat(vehicle_status, 'coastingAcceleration') as coasting_acc
    FROM bagdb.bag_messages
    WHERE vehicle=:vehicle
    and ts between :start_ts and :end_ts
    and JSONExtractFloat(vehicle_dbw_reports, 'throttleReport', 'pedalOutput')<0.01
    and JSONExtractFloat(vehicle_dbw_reports, 'brakeReport', 'pedalOutput')<0.01
    ORDER by ts """
)

QUERY_COLUMNS = ['ts','speed','a_report','throttle_output','brake_output', 'road_pitch','coasting_acc']

def data_query(vehicle_name, start_ts, end_ts):  
    print("data query start")    
    temp_ts = start_ts
    delta_time = timedelta(minutes=30)

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
        print("query interval {}/{}: data length = {}".format(temp_ts, end_ts, len(df)))

    if (temp_ts + delta_time > end_ts):
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": end_ts}).fetchall()
            df = pd.concat([df, pd.DataFrame(df_new,columns=QUERY_COLUMNS)],ignore_index=True)
    print("query interval {}/{}: data length = {}".format(temp_ts, end_ts, len(df)))
    print("data query finish")

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
         
    print(len(df))
    
    # df.to_csv(os.path.join(args.output_dir,args.data_file_name))
    return df


def filling_data(data):
    x = []
    y = []
    for i in range(len(data)):
        if(math.isnan(data[i])):
            continue
        x.append(i)
        y.append(data[i])
    if(x[0]!=0):
        x.insert(0,0)
        y.insert(0,y[0])
    if(x[-1]!=len(data)-1):
        x.append(len(data)-1)
        y.append(y[-1])
    
    f = interp1d(x, y)
    xnew = np.linspace(0, len(data)-1, num=len(data), endpoint=True)

    return f(xnew)

def filter_data(data, ratio):
    last_value = data[0]
    data_filtered = []
    for i in range(len(data)):
        last_value = last_value * ratio + data[i] * (1 - ratio)
        data_filtered.append(last_value)
    return data_filtered

def get_mse(records_real, records_predict):
    if len(records_real) == len(records_predict):
        return sum([(x - y) ** 2 for x, y in zip(records_real, records_predict)]) / len(records_real)
    else:
        return None

def get_rmse(records_real, records_predict):
    mse = get_mse(records_real, records_predict)
    if mse:
        return math.sqrt(mse)
    else:
        return None
def clean_data(args,df):
    left = 0
    right = 1
    cleaned_df = pd.DataFrame()
    while right < len(df):       
        delta_time = df['ts'][right] - df['ts'][left]
        print('left:{} delta_time:{}'.format(left,delta_time))
        if( delta_time < timedelta(milliseconds=(right-left)*100)):   
            right += 1
            delta_time = df['ts'][right] - df['ts'][left]

    return cleaned_df

def data_process(args, df, READ_FROM_FILE = False):
    if(READ_FROM_FILE):
        data = pd.read_csv(os.path.join(args.output_dir,args.data_file_name))
    else:
        data = df

    if(len(df)==0):
        return None

    print(df)
    # print(len(df))
    # cleaned_df = clean_data(args,df)
    # print('length of cleaned_df:{}'.format(len(cleaned_df)))
    for i in range(1,len(QUERY_COLUMNS)):
        data[QUERY_COLUMNS[i]] = filling_data(data[QUERY_COLUMNS[i]])

    # plt.figure(1)
    # plt.plot(range(len(data['a_report'])),data['a_report'],'r')
    # plt.plot(range(len(data['coasting_acc'])),data['coasting_acc'],'b')
    # plt.show()
    return get_rmse(data['a_report'],data['coasting_acc'])
    

def main(args):
    # vehicle_name = 'pdb-l4e-b0005'
    # start_ts = '2022-11-08 02:00:00'
    # end_ts = '2022-11-10 09:59:59'

    vehicle_names = ['j7-l4e-LFWSRXSJ7M1F48985']
    start_time = '2022-10-25 02:00:00'
    end_time = '2022-10-25 02:59:59'
    result_by_vehicle = OrderedDict()
    for vehicle_name in vehicle_names:
        print('====================query data of {}========================'.format(vehicle_name))
        start_ts = datetime.strptime(start_time, '%Y-%m-%d %H:%M:%S')
        end_ts = datetime.strptime(end_time, '%Y-%m-%d %H:%M:%S') 
        result_by_time = OrderedDict()
        temp_ts = start_ts
        delta_time = timedelta(days=1)
        while temp_ts + delta_time < end_ts:
            data = data_query(vehicle_name, temp_ts, temp_ts + delta_time)
            re = data_process(args, data, False)
            print("{} to {}, coasting acc rsme:{}".format(temp_ts,temp_ts + delta_time,re))
            result_by_time[str(temp_ts)+' to '+str(temp_ts + delta_time)] = re
            temp_ts += delta_time

        data = data_query(vehicle_name, temp_ts, end_ts)
        re = data_process(args, data, False)
        print("{} to {} : coasting acc rsme = {}".format(temp_ts,end_ts,re))
        result_by_time[str(temp_ts)+' to '+str(end_ts)] = re

        result_by_vehicle[vehicle_name] = result_by_time

    for vehicle, result in result_by_vehicle.items():
        print('===================={}========================'.format(vehicle))
        for time_interval, rsme in result.items():
            print("{}: coasting acc rsme = {}".format(time_interval,rsme))
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    parser.add_argument('--output-dir', default=os.path.join(os.environ['HOME'],'data/bag_msg'))
    parser.add_argument('--data-file-name', default='data.csv', type=str)
    parser.add_argument('--repull-data', default=False, type=bool)
    
    args = parser.parse_args()
    main(args)