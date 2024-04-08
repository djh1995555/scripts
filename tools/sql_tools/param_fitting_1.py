#!/usr/bin/env python

from datetime import datetime, timedelta
from pluspy import db_utils
import pandas as pd
import argparse
import os
import numpy as np
import math
import matplotlib.pyplot as plt

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
    JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'total_weight_estimation') as weight,
    JSONExtractFloat(vehicle_dbw_reports, 'throttleReport', 'pedalOutput') as throttle_output,
    JSONExtractFloat(vehicle_dbw_reports, 'brakeReport', 'pedalOutput') as brake_output,
    JSONExtractFloat(vehicle_status, 'pitch') as road_pitch,
    JSONExtractFloat(vehicle_status, 'chassis_a_y') as a_chassis
    FROM bagdb.bag_messages
    WHERE vehicle=:vehicle
    and ts between :start_ts and :end_ts
    ORDER by ts """
)

DATA_PROCESS_SQL = (
"""     SELECT ts,
    JSONExtractFloat(vehicle_dbw_reports, 'steeringReport', 'speed') as speed
    FROM bagdb.bag_messages
    WHERE vehicle=:vehicle
    and ts between :start_ts and :end_ts
    ORDER by ts """
)

QUERY_COLUMNS = ['ts','speed','a_report','weight','throttle_output','brake_output','road_pitch','a_chassis']

def data_query(vehicle_name, start_ts, end_ts):  
    print("data query start")    
    temp_ts = start_ts
    delta_time = timedelta(seconds=60)

    df = pd.DataFrame(columns=QUERY_COLUMNS)
    while temp_ts + delta_time < end_ts:
        print("query interval {}/{}:".format(temp_ts,end_ts))
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": temp_ts + delta_time}).fetchall()
            df = pd.concat([df, pd.DataFrame(df_new,columns=QUERY_COLUMNS)],ignore_index=True)
        temp_ts += delta_time

    with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
        df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                            "start_ts": temp_ts,
                                            "end_ts": end_ts}).fetchall()
        df = pd.concat([df, pd.DataFrame(df_new,columns=QUERY_COLUMNS)],ignore_index=True)

    print("data query finish")

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
         
    print(len(df))
    df.to_csv(os.path.join(args.output_dir,args.data_file_name))

def data_process(args,vehicle_name):
    data = pd.read_csv(os.path.join(args.output_dir,args.data_file_name))
    df_processed = pd.DataFrame()
    ts = data['ts']
    with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
        for i in range(len(ts)):
            current_time = datetime.strptime(str(data['ts'][i]),"%Y-%m-%d %H:%M:%S.%f")
            time_window = 2
            front_size = 0.5
            front_time_offset = timedelta(seconds=front_size)
            back_time_offset = timedelta(seconds=time_window - front_size)
            pre_time = (current_time - front_time_offset).strftime("%Y-%m-%d %H:%M:%S.%f")
            back_time = (current_time + back_time_offset).strftime("%Y-%m-%d %H:%M:%S.%f")
        
            df_new = db_session.execute(DATA_PROCESS_SQL,{"vehicle": vehicle_name,
                                                "start_ts": pre_time,
                                                "end_ts": back_time}).fetchall()
            left = 0
            right = len(df_new)-1
            left_found = False
            right_found = False
            while(left < right and (not left_found or not right_found)):
                if(df_new[left][1] == None):
                    left += 1
                else:
                    left_found = True
                if(df_new[right][1] == None):
                    right -= 1
                else:
                    right_found = True
            if(left < right):
                left_time = df_new[left][0]
                right_time = df_new[right][0]
                delta_t = right_time - left_time
                delta_time = delta_t.seconds*1000000 + delta_t.microseconds
                
                if(delta_time<1000000):
                    continue
                left_v = df_new[left][1]
                right_v = df_new[right][1]
                delta_v = right_v - left_v
                acc = delta_v * 1000000 / delta_time
                if (acc < -0.1):
                    continue
                new_df = pd.DataFrame([{'speed':data['speed'][i],'a_report':acc,'pitch':data['road_pitch'][i]}])
                print('{}/{}, left v:{}, right v:{}, delta_time:{}, acc:{}'.format(i, len(ts), left_v, right_v, delta_time, acc))
                df_processed = df_processed.append(new_df,ignore_index=True)                        
    print(df_processed)
    params_fitting(df_processed)


def params_fitting(df):
    vehicle_mass = 43000
    vehicle_frontal_area = 8.0
    air_density = 1.225
    gravity_acc = 9.81

    x = np.array([x*x for x in df['speed']])
    print('speed:\n', df['speed'])
    print('x:\n',x)
    y_list = []
    for i in range(len(df)):
        acc = df['a_report'][i]
        pitch = df['pitch'][i]
        y_list.append(vehicle_mass * (acc))
        print("a:{}, b:{}".format(vehicle_mass * acc, vehicle_mass * gravity_acc * math.sin(-pitch)))
    y = np.array(y_list)

    reg = np.polyfit(x, y, 1)
    print('reg is :\n',reg)
    drag_coefficient = -2*reg[0]/(air_density*vehicle_frontal_area)
    rolling_friction_coefficient = -reg[1]/(vehicle_mass*gravity_acc)
    print("drag_coefficient:{}, rolling_friction_coefficient:{}".format(drag_coefficient,rolling_friction_coefficient))
    yvals = np.polyval(reg, x)

    plt.plot(x, y, 's',label='original values')
    plt.plot(x, yvals, 'r',label='polyfit values')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc=4)
    plt.title('polyfitting')
    plt.show()



def main(args):
    vehicle_name = 'pdb-l4e-b0005'
    start_ts = '2022-11-09 08:00:00'
    end_ts = '2022-11-09 09:00:00'
    start_ts = datetime.strptime(start_ts, '%Y-%m-%d %H:%M:%S')
    end_ts = datetime.strptime(end_ts, '%Y-%m-%d %H:%M:%S') 

    # if not os.path.exists(os.path.join(args.output_dir,args.data_file_name)):   
    #     data_query(vehicle_name, start_ts, end_ts)
    # else:
    #     print("data has existed")
    # data_query(vehicle_name, start_ts, end_ts)

    data_process(args,vehicle_name)

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    parser.add_argument('--output-dir', default=os.path.join(os.environ['HOME'],'data/bag_msg'))
    parser.add_argument('--data-file-name', default='data.csv', type=str)
    args = parser.parse_args()
    main(args)