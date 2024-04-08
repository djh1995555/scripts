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
import jerk_utils
import time

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
    delta_time = timedelta(minutes=20)

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
    if(len(df) > 100000):
        print("data query finish")
        return df

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
    slow = left
    fast = 1
    cleaned_df = pd.DataFrame()
    while fast < len(df):
        # print('left:{}, slow:{}, fast:{}, delta_time:{}'.format(left, slow, fast, df['ts'][fast] - df['ts'][slow]))
        if( df['ts'][fast] - df['ts'][slow] < timedelta(milliseconds=100)):   
            slow += 1
            fast += 1
        else:
            if (df['ts'][slow] - df['ts'][left] > timedelta(seconds=5)):
                cleaned_df = pd.concat([cleaned_df, df.iloc[left:slow]],ignore_index=True)
            left = fast
            slow = left
            fast += 1
    # cleaned_df = cleaned_df.reset_index(drop=True)  
    return cleaned_df

def datetime2timestamp(ts):
   t = ts.timetuple() 
   timeStamp = int(time.mktime(t)) 
   timeStamp = float(str(timeStamp) + str("%06d" % ts.microsecond))/1000000
   return timeStamp

def compute_acc(args,data):
    
    for i in range(len(data)):
        if (math.isnan(data['speed'][i])):
            continue
        current_time = data['ts'][i]
        target_time_length = 1 # unit:s
        front_index = i
        front_time = data['ts'][front_index]
        delta_t = current_time - front_time
        front_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds
        while front_index -1 > 0 and front_delta_time < target_time_length * SECOND_TO_MICROSECOND :
            front_index -= 1
            front_time = data['ts'][front_index]
            delta_t = current_time - front_time
            front_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds

        end_index = i
        end_time = data['ts'][end_index]
        delta_t = end_time - current_time
        end_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds
        # while end_index + 1 < len(data['speed']) and end_delta_time < target_time_length * SECOND_TO_MICROSECOND :
        #     end_index += 1
        #     end_time = data['ts'][end_index]
        #     delta_t = end_time - current_time
        #     end_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds
        
        delta_v = data['speed'][end_index] - data['speed'][front_index]
        acc = delta_v * SECOND_TO_MICROSECOND / (front_delta_time + end_delta_time)
        data['a_report'][i] = acc

    # longitudinal_acc_filter = jerk_utils.TrackingDifferentiator(tracking_factor=5, order=2)
    # for i in range(len(data)):
    #     if (math.isnan(data['speed'][i])):
    #         continue
    #     speed = data['speed'][i]
    #     cur_callback_time = datetime2timestamp(data['ts'][i])
    #     # update imu filter
    #     if not longitudinal_acc_filter.is_activated:
    #         # initialize longitudinal jerk filter
    #         speed_filtered = speed
    #         acc_filtered = 0
    #         longitudinal_acc_filter.reset(cur_callback_time, [speed_filtered, acc_filtered])
    #     else:
    #         (speed_filtered,acc_filtered,) = longitudinal_acc_filter.step(speed, cur_callback_time)

    #     data['a_report'][i] = acc_filtered
    #     print('i = {}, end = "\r"'.format(i))
    
def data_process(args, df, READ_FROM_FILE = False):
    if(READ_FROM_FILE):
        data = pd.read_csv(os.path.join(args.output_dir,args.data_file_name))
    else:
        data = df

    if(len(data)==0):
        return None
    # data = clean_data(args,data)
    # print('len of cleaned data:{}'.format(len(data)))
    # compute_acc(args,data)
    for i in range(1,len(QUERY_COLUMNS)):
        print("filling data of {}".format(QUERY_COLUMNS[i]))
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

    vehicle_names = ['pdb-l4e-b0008']
    start_time = '2022-11-03 00:00:00'
    end_time = '2022-11-11 23:59:59'
    result_by_vehicle = OrderedDict()
    for vehicle_name in vehicle_names:
        print('====================query data of {}========================'.format(vehicle_name))
        start_ts = datetime.strptime(start_time, '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
        end_ts = datetime.strptime(end_time, '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
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