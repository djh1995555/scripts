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
    JSONExtractFloat(vehicle_dbw_reports, 'throttleReport', 'pedalOutput') as throttle_output,
    JSONExtractFloat(vehicle_dbw_reports, 'brakeReport', 'pedalOutput') as brake_output,
    JSONExtractFloat(localization_status_report, 'position', 'z') as elevation,
    JSONExtractFloat(vehicle_status, 'pitch') as road_pitch,
    JSONExtractFloat(vehicle_status, 'mapPitch') as map_pitch
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

QUERY_COLUMNS = ['ts','speed','throttle_output','brake_output','elevation', 'road_pitch','map_pitch']

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

def data_process(args):
    data = pd.read_csv(os.path.join(args.output_dir,args.data_file_name))
    df_processed = pd.DataFrame()
    ts = data['ts']
    data['speed'] = filling_data(data['speed'])
    data['road_pitch'] = filling_data(data['road_pitch'])
    data['throttle_output'] = filling_data(data['throttle_output'])
    data['brake_output'] = filling_data(data['brake_output'])
    data['elevation'] = filling_data(data['elevation'])
    # data['speed'] = filter_data(data['speed'], 0.9)


    for i in range(len(data)):
        if (math.isnan(data['speed'][i])):
            continue
        current_time = datetime.strptime(str(data['ts'][i]),"%Y-%m-%d %H:%M:%S.%f")
        target_time_length = 1 # unit:s
        front_index = i
        front_time = datetime.strptime(str(data['ts'][front_index]),"%Y-%m-%d %H:%M:%S.%f")
        delta_t = current_time - front_time
        front_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds
        while front_index -1 > 0 and front_delta_time < target_time_length * SECOND_TO_MICROSECOND :
            front_index -= 1
            front_time = datetime.strptime(str(data['ts'][front_index]),"%Y-%m-%d %H:%M:%S.%f")
            delta_t = current_time - front_time
            front_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds

        end_index = i
        end_time = datetime.strptime(str(data['ts'][end_index]),"%Y-%m-%d %H:%M:%S.%f")    
        delta_t = end_time - current_time
        end_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds
        while end_index + 1 < len(data['speed']) and end_delta_time < target_time_length * SECOND_TO_MICROSECOND :
            end_index += 1
            end_time = datetime.strptime(str(data['ts'][end_index]),"%Y-%m-%d %H:%M:%S.%f")
            delta_t = end_time - current_time
            end_delta_time = delta_t.seconds * SECOND_TO_MICROSECOND + delta_t.microseconds
        # delta_v = data['speed'][end_index] - data['speed'][front_index]
        # acc = delta_v * SECOND_TO_MICROSECOND / (front_delta_time + end_delta_time)

        delta_v = data['speed'][end_index] - data['speed'][i]
        acc = delta_v * SECOND_TO_MICROSECOND /  end_delta_time
        if(math.isnan(acc)):
            continue
        
        current_v = data['speed'][i]
        new_df = pd.DataFrame([{'ts':data['ts'][i],'speed':current_v,'a_report':acc,'pitch':data['road_pitch'][i],'elevation':data['elevation'][i],
                                'throttle_output':data['throttle_output'][i],'brake_output':data['brake_output'][i]}])
        # print('{}/{}, ts:{}, speed:{}, pitch:{}, acc:{}'.format(i, len(ts), data['ts'][i], current_v, pitch, acc))
        df_processed = df_processed.append(new_df,ignore_index=True)
                       
    print(df_processed)
    params_fitting(df_processed)

def polyfitting_method(df):
    x = np.array([x*x for x in df['speed']])

    y_list = []
    for i in range(len(df)):
        acc = df['a_report'][i]
        pitch = df['pitch'][i]
        y_list.append(VEHICLE_MASS * (acc + GRAVITY_ACC * math.sin(-pitch)))
        # print("a:{}, b:{}".format(VEHICLE_MASS * acc, VEHICLE_MASS * GRAVITY_ACC * math.sin(-pitch)))
    y = np.array(y_list)

    reg = np.polyfit(x, y, 1)
    print('reg is :\n',reg)
    average_pitch = df['pitch'].mean()
    drag_coefficient = -2*reg[0]/(AIR_DENSITY*FRONTAL_AREA)
    rolling_friction_coefficient = -reg[1]/(VEHICLE_MASS*GRAVITY_ACC*math.cos(-average_pitch))
    print("drag_coefficient:{}, rolling_friction_coefficient:{} from polyfitting_method".format(drag_coefficient,rolling_friction_coefficient))
    yvals = np.polyval(reg, x)
    plt.figure(2)
    plt.plot(x, y, 'b',label='original values')
    plt.plot(x, yvals, 'r',label='polyfit values')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc=4)
    plt.title('polyfitting')

def first_order_matrix_calculation(df):
    new_df =  pd.concat([df, df],ignore_index=True)
    drag_coefficient_list = []
    rolling_friction_coefficient_list = []
    for i in range(len(df)):
        first_index = i
        second_index = i + 100
        a = np.array([
            [-0.5*AIR_DENSITY*FRONTAL_AREA*new_df['speed'][first_index]*new_df['speed'][first_index], -VEHICLE_MASS*GRAVITY_ACC*math.cos(-new_df['pitch'][first_index])],
            [-0.5*AIR_DENSITY*FRONTAL_AREA*new_df['speed'][second_index]*new_df['speed'][second_index], -VEHICLE_MASS*GRAVITY_ACC*math.cos(-new_df['pitch'][second_index])]])
        b0 = np.array([(new_df['a_report'][first_index]),(new_df['a_report'][second_index])])
        b1 = np.array([GRAVITY_ACC*math.sin(-new_df['pitch'][first_index]),GRAVITY_ACC*math.sin(-new_df['pitch'][second_index])])
        b = VEHICLE_MASS * (b0 + b1)
        expected_param = np.array([0.8, 0.008])
        expected_b = np.dot(a, expected_param)
        expected_acc = (expected_b - b1)/VEHICLE_MASS
        print("a:{}\nexpected_b:{}\nb:{}\nexpected_acc:{}\ncurrent_a:{}\nslope_resistance:{}".format(a,expected_b,b,expected_acc,b0,VEHICLE_MASS*b1))
        res = np.linalg.solve(a,b)
        print("drag_coefficient:{}, rolling_friction_coefficient:{} from first_order_matrix_calculation".format(res[0],res[1]))
        drag_coefficient_list.append(res[0])
        rolling_friction_coefficient_list.append(res[1])
    plt.figure(3)
    plt.plot(drag_coefficient_list, rolling_friction_coefficient_list,'r.')
    plt.xlabel('drag_coefficient')
    plt.ylabel('rolling_friction_coefficient')
    plt.legend(loc=4)

def second_order_matrix_calculation(df):
    new_df =  pd.concat([df, df],ignore_index=True)
    drag_coefficient_list = []
    rolling_friction_coefficient_1_list = []
    rolling_friction_coefficient_2_list = []
    for i in range(len(df)):
        interval = 100
        first_index = i
        second_index = i + interval
        third_index = second_index + interval
        a = np.array([
            [-0.5*AIR_DENSITY*FRONTAL_AREA*new_df['speed'][first_index]*new_df['speed'][first_index], -VEHICLE_MASS*GRAVITY_ACC*new_df['speed'][first_index]*math.cos(-new_df['pitch'][first_index]),-VEHICLE_MASS*GRAVITY_ACC*math.cos(-new_df['pitch'][first_index])],
            [-0.5*AIR_DENSITY*FRONTAL_AREA*new_df['speed'][second_index]*new_df['speed'][second_index], -VEHICLE_MASS*GRAVITY_ACC*new_df['speed'][second_index]*math.cos(-new_df['pitch'][second_index]),-VEHICLE_MASS*GRAVITY_ACC*math.cos(-new_df['pitch'][second_index])],
            [-0.5*AIR_DENSITY*FRONTAL_AREA*new_df['speed'][third_index]*new_df['speed'][third_index], -VEHICLE_MASS*GRAVITY_ACC*new_df['speed'][third_index]*math.cos(-new_df['pitch'][third_index]),-VEHICLE_MASS*GRAVITY_ACC*math.cos(-new_df['pitch'][third_index])]])
        b0 = np.array([(new_df['a_report'][first_index]),(new_df['a_report'][second_index]),(new_df['a_report'][third_index])])
        b1 = np.array([math.sin(-new_df['pitch'][first_index]),math.sin(-new_df['pitch'][second_index]),math.sin(-new_df['pitch'][third_index])])
        b = VEHICLE_MASS * (b0 + GRAVITY_ACC*b1)
        res = np.linalg.solve(a,b)
        print("drag_coefficient:{}, rolling_friction_coefficient:{} from first_order_matrix_calculation".format(res[0],res[1]))
        drag_coefficient_list.append(res[0])
        rolling_friction_coefficient_1_list.append(res[1])
        rolling_friction_coefficient_2_list.append(res[2])
    plt.figure(4)
    ax1 = plt.subplot(2,1,1)
    ax1.plot(drag_coefficient_list, rolling_friction_coefficient_1_list,'r.')
    ax2 = plt.subplot(2,1,2)
    ax2.plot(drag_coefficient_list, rolling_friction_coefficient_2_list,'r.')
    plt.xlabel('drag_coefficient')
    plt.ylabel('rolling_friction_coefficient')
    plt.legend(loc=4)    
def params_fitting(df):

    polyfitting_method(df)
    # first_order_matrix_calculation(df)
    # second_order_matrix_calculation(df)

    plt.figure(1)
    ax1 = plt.subplot(6,1,1)
    ax1.plot(range(len(df['pitch'])),df['pitch'],label = 'pitch')
    plt.legend(loc=4)
    plt.grid()
    ax2 = plt.subplot(6,1,2)
    ax2.plot(range(len(df['speed'])),df['speed'],label = 'speed')
    plt.legend(loc=4)
    plt.grid()
    ax3 = plt.subplot(6,1,3)
    ax3.plot(range(len(df['a_report'])),df['a_report'],label = 'a_report')
    plt.legend(loc=4)
    plt.grid()
    ax4 = plt.subplot(6,1,4)
    ax4.plot(range(len(df['throttle_output'])),df['throttle_output'],label = 'throttle_output')
    plt.legend(loc=4)
    plt.grid()
    ax5 = plt.subplot(6,1,5)
    ax5.plot(range(len(df['brake_output'])),df['brake_output'],label = 'brake_output')
    plt.legend(loc=4)
    plt.grid()
    ax6 = plt.subplot(6,1,6)
    ax6.plot(range(len(df['elevation'])),df['elevation'],label = 'elevation')
    plt.legend(loc=4)
    plt.grid()
    plt.show()



def main(args):
    # vehicle_name = 'pdb-l4e-b0008'
    # start_ts = '2022-11-04 04:54:00'
    # end_ts = '2022-11-04 04:54:05'
    # vehicle_name = 'pdb-l4e-b0005'
    # start_ts = '2022-11-11 02:45:24'
    # end_ts = '2022-11-11 02:45:33'
    # vehicle_name = 'pdb-l4e-b0005'
    # start_ts = '2022-11-10 03:28:10'
    # end_ts = '2022-11-10 03:28:30'
    # vehicle_name = 'pdb-l4e-b0005'
    # start_ts = '2022-11-10 03:39:45'
    # end_ts = '2022-11-10 03:40:10'
    # vehicle_name = 'j7-l4e-LFWSRXSJ7M1F48985'
    # start_ts = '2022-10-27 07:38:00'    # 07:37:50
    # end_ts = '2022-10-27 07:38:05'      # 07:38:30
    vehicle_name = 'j7-l4e-LFWSRXSJ7M1F48985'
    start_ts = '2022-10-25 08:03:10'
    end_ts = '2022-10-25 08:03:15'
    start_ts = datetime.strptime(start_ts, '%Y-%m-%d %H:%M:%S')
    end_ts = datetime.strptime(end_ts, '%Y-%m-%d %H:%M:%S') 

    if args.repull_data or (not os.path.exists(os.path.join(args.output_dir,args.data_file_name))):   
        data_query(vehicle_name, start_ts, end_ts)
    else:
        print("data has existed")
    # data_query(vehicle_name, start_ts, end_ts)

    data_process(args)

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    parser.add_argument('--output-dir', default=os.path.join(os.environ['HOME'],'data/bag_msg'))
    parser.add_argument('--data-file-name', default='data.csv', type=str)
    parser.add_argument('--repull-data', default=False, type=bool)
    
    args = parser.parse_args()
    main(args)