#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from datetime import datetime, timedelta
from pluspy import db_utils
import pandas as pd
import argparse
import os
import re
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from collections import deque, OrderedDict 
import yaml
import psycopg2

weather_dict = {
    'clear sky':0,
    'haze':1,
    'fog':2,
    'mist':3,
    'few clouds':4,
    'broken clouds':5,
    'scattered clouds':6,
    'overcast clouds':7,
    'light rain':8,
    'light intensity shower rain':9,
    'moderate rain':10,
    'heavy intensity rain':11,
    'very heavy rain':12,
}

WEIGHT_QUERY_SQL = (
"""
    SELECT
        msg,
        start_time
    FROM 
        bag_events
    WHERE 
        category = 'Drive' and
        status = 0 and
        type = 2 and
        tag = 'LoadWeight' and
        vehicle='{}' and
        start_time >= '{}' and
        start_time < '{}'
"""
)

WEATHER_QUERY_SQL = (
"""
    SELECT
        vehicle,
        timestamp,
        weather,
        lat,
        lon
    FROM t_weather_info
    WHERE
        vehicle = '{}' AND
        timestamp >= '{}' AND
        timestamp < '{}'
        
"""
)

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
"""      SELECT vehicle, start_time, end_time, bag_name, fastbag_path, fastbag_size, bag_source, distance, auto_distance
    FROM bags b
    WHERE vehicle=:vehicle
    and start_time > :start_ts 
    and end_time < :end_ts
    and bag_source='offline'
    ORDER by start_time """
)

QUERY_COLUMNS = ['vehicle', 'start_time','end_time','bag_name','fastbag_path','fastbag_size', 'bag_source', 'distance', 'auto_distance']

FASTBAG_SIZE_THESHOLD = 2.0* 1024.0 ** 3.0
DURATION_THRESHOLD = 600.0
AUTO_RATIO_THRESHOLD = 80.0
DISTANCE_THRESHOLD = 1000.0
 
def filter_data(df):
    print("original data num:{}".format(len(df))) 
    drop_list = []
    df = df.dropna(subset=['fastbag_path'])
    df = df.reset_index(drop=True)
    for i in range(len(df)):
        if( (df['end_time'].iloc[i] - df['start_time'].iloc[i]) < DURATION_THRESHOLD or 
            df['fastbag_size'].iloc[i] < FASTBAG_SIZE_THESHOLD or
            df['distance'].iloc[i] < DISTANCE_THRESHOLD or
            df['auto_distance'].iloc[i] < (df['distance'].iloc[i] * AUTO_RATIO_THRESHOLD /100.0)):
            drop_list.append(i)
    df = df.drop(drop_list)
    return df
    
def query_weight_gt(cursor, vehicle_name, start_ts, end_ts):
    cursor.execute(WEIGHT_QUERY_SQL.format(vehicle_name, start_ts, end_ts))
    records = cursor.fetchall()
    weight = ''
    if(len(records)!=0):
        weight = records[0][0]
    match = re.search(r"[-+]?\d*\.?\d+", weight)
    if match:
        weight = float(match.group())
    else:
        weight = 0
    return weight
   
def data_query(vehicle_name, start_ts, end_ts):  
    temp_ts = start_ts
    delta_days = 10
    delta_time = delta_days * 24 * 60 * 60
    df = pd.DataFrame(columns=QUERY_COLUMNS)
    while temp_ts + delta_time < end_ts:
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": temp_ts + delta_time}).fetchall()
            new_df = pd.DataFrame(df_new,columns=QUERY_COLUMNS)
            new_df = new_df.drop_duplicates(subset=['bag_name'], keep='first')
            new_df = new_df.reset_index(drop=True)
            new_df = filter_data(new_df)
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
            new_df = pd.DataFrame(df_new,columns=QUERY_COLUMNS)
            new_df = new_df.drop_duplicates(subset=['bag_name'], keep='first')
            new_df = new_df.reset_index(drop=True)
            print("data query start, from {} to {}".format(start_ts, end_ts)) 
            new_df = filter_data(new_df)
            df = pd.concat([df, new_df],ignore_index=True)
    
    print('total num:{}'.format(len(df)))
    return df

def save_data(df, output_dir):
    df = df.reset_index(drop=True)
    fastbag_path_list = list([fastbag_path.encode('unicode_escape').decode('utf-8').replace('\\u', '') for fastbag_path in df['fastbag_path']])
    data = {'baglist_path':fastbag_path_list}
    yaml.safe_dump(data, open(os.path.join(output_dir, args.output_file_name),'w'), encoding='utf-8', allow_unicode=True)  
    df.to_csv(os.path.join(output_dir, 'fastbag_path.csv'))  

def select_by_day(df, num):
    print(len(df))
    df = df[df['weather']!=np.nan]
    print(len(df))
    grouped = df.groupby(['vehicle', 'date'])
    selected_df = pd.DataFrame()
    for key, group in grouped:
        if(group.shape[0]< num):
            selected_data = group
        else:
            selected_data = group.sample(num)
        selected_df = pd.concat([selected_df, selected_data])

    return selected_df

def compute_heading(lat1, lon1, lat2, lon2):
    
    lat1_rad = np.radians(float(lat1))
    lon1_rad = np.radians(float(lon1))
    lat2_rad = np.radians(float(lat2))
    lon2_rad = np.radians(float(lon2))

    delta_lon = lon2_rad - lon1_rad

    x = np.sin(delta_lon) * np.cos(lat2_rad)
    y = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(delta_lon)
    heading_rad = np.arctan2(x, y)

    heading = (np.degrees(heading_rad) + 360) % 360
    if heading > 180:
        heading -= 360

    return heading

def wind_degree_to_cartesian(wind_degree):
    cartesian_angle = (-wind_degree - 90) % 360
    if cartesian_angle >= 180:
        cartesian_angle -= 360
    return cartesian_angle

def query_weather_info(cursor, vehicle_name, start_ts, end_ts, delta_timestamp):
    i = 0
    record = []
    while(len(record) == 0 and i <= 0):
        cursor.execute(WEATHER_QUERY_SQL.format(vehicle_name, start_ts, end_ts))
        record = cursor.fetchall()
        start_ts -= delta_timestamp
        end_ts += delta_timestamp
        i += 1
    return record

def get_weather_info(row, cursor):
    vehicle_name = row['vehicle']
    start_timestamp = row['start_time']
    end_timestamp = row['end_time']
    delta_timestamp = end_timestamp - start_timestamp
    record_now = []
    start_ts = start_timestamp
    end_ts = end_timestamp
    record_now = query_weather_info(cursor, vehicle_name, start_ts, end_ts, delta_timestamp)
    
    if(len(record_now) < 2):
        return np.nan, np.nan, np.nan, np.nan, np.nan
    else:
        vehicle_heading = compute_heading(record_now[0][3],record_now[0][4],record_now[1][3],record_now[1][4])
        wind_degree = wind_degree_to_cartesian(record_now[0][2]['wind_deg'])
        relative_speed = math.cos(np.radians(wind_degree - vehicle_heading)) * record_now[0][2]['wind_speed']
        weather = weather_dict[record_now[0][2]['weather'][0]['description']]
        return weather, wind_degree, vehicle_heading, record_now[0][2]['wind_speed'],relative_speed

def add_weather_info(data, cursor):
    for index, row in data.iterrows():
        data.loc[index, 'weather'], data.loc[index, 'wind_heading'], data.loc[index, 'vehicle_heading'], data.loc[index, 'wind_speed'],data.loc[index, 'relative_speed'] = get_weather_info(row, cursor)
        
def main(args):
    with open(args.config, 'r') as f:
        config = yaml.load(f)

    vehicle_names = np.array(config['vehciles']) 
    if(len(vehicle_names) == 0):
        vehicle_names = ['pdb-l4e-b0004','pdb-l4e-b0006','pdb-l4e-b0007','pdb-l4e-b0008',
                         'j7-l4e-LFWSRXSJ0M1F48990',    # 杭州-武汉
                         'j7-l4e-LFWSRXSJXM1F50505',    # 江阴-武汉
                         'j7-l4e-LFWSRXSJXM1F50116',    # 江阴-盘锦
                         'j7-l4e-LFWSRXSJ4M1F50502',    # 杭州-济南
                         'j7-l4e-LFWSRXSJ8M1F50115']    # 江阴-济南

    time_interval = np.array(config['time_interval'])
    df = pd.DataFrame(columns=QUERY_COLUMNS)
    for vehicle_name in vehicle_names:
        print('====================query data of {}========================'.format(vehicle_name))
        start_time = datetime.strptime(time_interval[0], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
        start_ts = float(time.mktime(start_time.timetuple()))
        end_time = datetime.strptime(time_interval[1], '%Y-%m-%d %H:%M:%S')- timedelta(hours=8)
        end_ts = float(time.mktime(end_time.timetuple()))
        new_df = data_query(vehicle_name, start_ts, end_ts)
        df = pd.concat([df, new_df],ignore_index=True)
        
    conn_string = "dbname='bagdb' host='sz-typostgres' user='bagdb_guest' password='guest@2022!'"
    conn = psycopg2.connect(conn_string)
    cursor = conn.cursor()   
    weight_gt_values = []
    dates = []
    for index, row in df.iterrows():
        bag_name = row['bag_name']
        date_str = bag_name[:8]
        date = datetime.strptime(date_str, '%Y%m%d')
        start_of_day = datetime(date.year, date.month, date.day, 0, 0, 0) - timedelta(days=1)
        start_timestamp = float(time.mktime(start_of_day.timetuple()))
        end_of_day = datetime(date.year, date.month, date.day, 23, 59, 59) - timedelta(days=1)
        end_timestamp = float(time.mktime(end_of_day.timetuple()))

        # print('start_timestamp:{}'.format(datetime.fromtimestamp(start_timestamp)))
        # print('end_timestamp:{}'.format(datetime.fromtimestamp(end_timestamp)))
        weight_gt = query_weight_gt(cursor, row['vehicle'], start_timestamp, end_timestamp)
        weight_gt_values.append(weight_gt)
        dates.append(date_str)
    weight_gt_df = pd.DataFrame(weight_gt_values,columns=['weight_gt'])
    dates_df = pd.DataFrame(dates,columns=['date'])
    df.insert(1,'weight_gt',weight_gt_df)
    df.insert(1,'date',dates_df)
    
    save_data(df, output_dir)
    weather_conn_string = "host='labeling2' dbname='tractruck_new' user='tractruck_viewer' password='tractruck_viewer'"
    weather_conn = psycopg2.connect(weather_conn_string)
    weather_cursor = weather_conn.cursor()
    add_weather_info(df, weather_cursor)
    df.to_csv(os.path.join(output_dir,'fastbag_path_with_weather.csv'), mode = 'w')
    
    df_selected = select_by_day(df,args.selected_num_per_day)
    df_selected.to_csv(os.path.join(output_dir, 'selected_fastbag_path.csv'))
     
    conn.close()  
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    file_dirname = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(file_dirname, 'bags_queried_for_weight_estimation')
    parser.add_argument('--config', default=os.path.join(file_dirname,'config/sql_config.yaml'), type=str)
    parser.add_argument('--output-file-name', default='fastbag_path.yaml', type=str)
    parser.add_argument('--output-dir', default=output_dir, type=str)
    parser.add_argument('--selected-num-per-day', default=1, type=int)
    if not os.path.exists(output_dir):  
        os.makedirs(output_dir)
    args = parser.parse_args()
    main(args)