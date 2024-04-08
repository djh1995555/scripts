#!/usr/bin/env python

from collections import OrderedDict
from datetime import datetime, timedelta
from pluspy import db_utils
import pandas as pd
import argparse
import os
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils.report_plotter import ReportPlotter
from utils.rgdf1tls_estimator import RGDF1TLSEstimator

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
    vehicle as vehicle_name,
    dbw_enabled,
    JSONExtractFloat(vehicle_status, 'v') as v,
    JSONExtractFloat(vehicle_status, 'pitch') as pitch,
    JSONExtractFloat(vehicle_status, 'accFromKalmanDifferentiator') as a_report,
    JSONExtractFloat(vehicle_status, 'coastingAcceleration') as coasting_acc,
    JSONExtractFloat(vehicle_status, 'transmissionGearRatio') as gear_ratio,
    JSONExtractFloat(vehicle_status, 'state_gear') as gear,
    JSONExtractFloat(vehicle_status, 'engineFrictionTorque') as throttle_friction,
    JSONExtractFloat(vehicle_status, 'engineTorque') as throttle_output,
    JSONExtractFloat(vehicle_control_cmd, 'throttleCmd', 'normalizedValue') as throttle_cmd,
    JSONExtractFloat(vehicle_control_cmd, 'brakeCmd', 'normalizedValue') as brake_cmd,
    JSONExtractFloat(vehicle_control_cmd, 'brakeCmd', 'retarderTorquePct') as retarder_cmd,
    JSONExtractFloat(vehicle_control_cmd, 'brakeCmd', 'engineBrakeTorquePct') as engien_brake_cmd,
    JSONExtractFloat(vehicle_control_cmd, 'steeringWheelAngleCmd') as steering_angle
    FROM bagdb.bag_messages
    WHERE vehicle=:vehicle
    and ts between :start_ts and :end_ts
    and JSONExtractFloat(vehicle_control_cmd, 'throttleCmd', 'normalizedValue') = 0
    and dbw_enabled = 1
    ORDER by ts """
)

QUERY_COLUMNS = ['ts', 'vehicle_name','dbw_enabled', 'v','pitch','a_report','coasting_acc','gear_ratio','gear','throttle_friction','throttle_output','throttle_cmd','brake_cmd','retarder_cmd','engien_brake_cmd','steering_angle']


def plot_signal(report_plotter, name, data_dict):
    subplot_figure = None
    plot_html_str = ""
    figure_list = []
    for segment_name, signals in data_dict.items():
        # print('segment_name:{}'.format(segment_name))
        legend_list = []
        y_list = []
        x_list = []
        for signal_name, data in signals.items():
            # print(' segment_name:{}, len:{}'.format(segment_name, len(data)))
            legend_list.append(signal_name)
            y_list.append(np.array(data))
            x_list = np.array(range(len(data)))
        subplot = report_plotter.plot_figure_plotly(x_list = x_list, 
                                                    y_list = y_list,
                                                    legend_list = legend_list,
                                                    x_label = 'timestamp',
                                                    y_label = 'value',
                                                    title = '{}'.format(segment_name),
                                                    legend_prefix = '',
                                                    figure_height= 300,)
        figure_list.append(subplot)

    subplot_figure_list = [(i + 1, 1, fig) for i, fig in enumerate(figure_list)]
    subplot_figure = report_plotter.append_figure_to_subplot_plotly(subplot_figure_list, 
                                                                        row_num = len(figure_list), 
                                                                        col_num = 1, 
                                                                        template="plotly_dark", 
                                                                        subplot_fig=subplot_figure,
                                                                        figure_height = 300,
                                                                        vertical_spacing = 0.02)
    plot_html_str += report_plotter.get_fuel_fig_html_str({'output_name': subplot_figure})
    html_str = report_plotter.generate_html_fuel_report(plot_html_str)
    with open('{}.html'.format(name), 'w') as f:
        f.write(html_str)

def data_query(vehicle_name, start_ts, end_ts, output_name):  
    print("data query start")  
    temp_ts = start_ts
    delta_time = timedelta(minutes=1)
    df = pd.DataFrame(columns=QUERY_COLUMNS)
    while temp_ts + delta_time < end_ts:
        print("query interval {}/{}:".format(temp_ts,end_ts))
        with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
            df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                                "start_ts": temp_ts,
                                                "end_ts": temp_ts + delta_time}).fetchall()
            temp_ts += delta_time
            print('len of data:{}'.format(len(df_new)))
            # if(len(df_new) < 100):
            #     continue
            df = pd.concat([df, pd.DataFrame(df_new,columns=QUERY_COLUMNS)],ignore_index=True)
        

    with db_utils.db_session_open_close(DB_CONF['clickhouse']) as db_session:
        df_new = db_session.execute(SQL_BAG_MSG,{"vehicle": vehicle_name,
                                            "start_ts": temp_ts,
                                            "end_ts": end_ts}).fetchall()
        df = pd.concat([df, pd.DataFrame(df_new,columns=QUERY_COLUMNS)],ignore_index=True)
    print("data query finish")   

    if(len(df)==0):
        print('len of data:{}'.format(len(df)))
        return df
    
    df.to_csv('{}.csv'.format(output_name))

    return df

def filter_data(df):
    if(len(df)==0):
        return df
    
    delay_time = 0.0
    sample_time = 0.05
    acc_data = df['a_report']
    acc_data = acc_data.shift(int(-delay_time/sample_time))
    df = df.drop('a_report', axis=1)
    df = pd.concat([df,acc_data],axis=1)


    condition = (df['gear'] == 0)
    df.loc[condition, 'gear_ratio'] = 0
    condition = (df['gear'] == 1)
    df.loc[condition, 'gear_ratio'] = 1.0
    condition = (df['gear_ratio'] == 0)
    df.loc[condition, 'gear'] = 0
    condition = (df['gear_ratio'] == 1.0)
    df.loc[condition, 'gear'] = 12
    # df = df[(df['gear'] == 12) | (df['gear'] == 0)]   
    # df = df.dropna(subset=['gear'])
    # df = df.dropna(subset=['gear_ratio'])
    df = df.interpolate()
    df = df.dropna()
    # df = df[(df['v'] > 10) &
    #         (df['brake_iutput'] == 0) &  
    #         (df['brake_cmd'] == 0) &
    #         (df['retarder_output'] == 0) &
    #         (df['retarder_cmd'] == 0) &
    #         (df['engien_brake_output'] == 0) &
    #         (df['engien_brake_cmd'] == 0) &
    #         (abs(180 * df['steering_angle'] / math.pi) < 10 ) ]
    return df

def params_fitting(rls, data, vehicle_name, date, target_acc, use_resistance_without_v = True):
    vehicle_mass_dict = {}
    if(target_acc=='a_report'):
        vehicle_mass_dict['pdb-l4e-b0003'] = {
            '2023-08-03':43000,
            '2023-08-04':41100,
            '2023-08-05':41100,
            '2023-08-06':41100,
            '2023-08-07':41100,
            '2023-08-08':41100,
            '2023-08-09':41100,
            '2023-08-10':27700,
            '2023-08-11':27700,
            '2023-08-12':42800,
        }
    else:
        vehicle_mass_dict['pdb-l4e-b0003'] = {
            '2023-08-03':21373,
            '2023-08-04':35043,
            '2023-08-05':35822,
            '2023-08-06':35571,
            '2023-08-07':35272,
            '2023-08-08':36238,
            '2023-08-09':35772,
            '2023-08-10':21161,
            '2023-08-11':21409,
            '2023-08-12':36100,
        }        

    vehicle_frontal_area = 8.0
    air_density = 1.225
    gravity_acc = 9.81
    inertia_wheels = 60
    inertia_engine = 4.0
    transmission_efficiency = 0.95
    axle_drive_ratio = 2.714
    max_throttle_engine_torque = 3125
    transmission_efficiency = 0.95
    effective_tire_radius = 0.51

    vehicle_mass = vehicle_mass_dict[vehicle_name][date]
    df= data[data['vehicle_name'] == vehicle_name]

    overall_ratio = df['gear_ratio'] * axle_drive_ratio
    # print('len:{}'.format(df['pitch']))
    rotation_equivalent_mass = (inertia_wheels + inertia_engine * transmission_efficiency * overall_ratio * overall_ratio) / (effective_tire_radius**2)

    if(target_acc=='a_report'):
        F_drive = (df['throttle_cmd'] - df['throttle_friction']) * max_throttle_engine_torque * overall_ratio * transmission_efficiency / effective_tire_radius
    else:
        F_drive = -df['throttle_friction'] * max_throttle_engine_torque * overall_ratio * transmission_efficiency / effective_tire_radius

    

    print('vehicle name:{}, len:{}, date:{}, mass:{}'.format(vehicle_name, len(df), date, vehicle_mass))
    y = F_drive + vehicle_mass * gravity_acc * np.sin(df['pitch']) - (vehicle_mass + rotation_equivalent_mass) * df[target_acc] 
    y = np.array(y)
    y = y.reshape(-1,1)

    
    if(use_resistance_without_v):
        A = np.zeros((y.shape[0],2))
        A[:,0] = df['v']*df['v']
        A[:,1] = vehicle_mass * gravity_acc * np.cos(df['pitch'])
    else:
        A = np.zeros((y.shape[0],3))
        A[:,0] = df['v']*df['v']
        A[:,1] = vehicle_mass * gravity_acc * np.cos(df['pitch'])
        A[:,2] = vehicle_mass * gravity_acc * np.cos(df['pitch']) * df['v']

    params, residuals, rank, singular_values = np.linalg.lstsq(A, y, rcond=-1)
    coefficients = params.flatten()

    if(use_resistance_without_v):
        ca = coefficients[0]
        cr = coefficients[1] 
        df['a_estimated'] = (F_drive + vehicle_mass * gravity_acc * (np.sin(df['pitch']) - cr * np.cos(df['pitch'])) - ca * df['v']*df['v'])/(vehicle_mass + rotation_equivalent_mass)
    else:
        ca = coefficients[0]
        cr = coefficients[1] 
        f0 = coefficients[2]
        df['a_estimated'] = (F_drive + vehicle_mass * gravity_acc * (np.sin(df['pitch']) - (cr + f0 * df['v']) * np.cos(df['pitch'])) - ca * df['v']*df['v'])/(vehicle_mass + rotation_equivalent_mass)

    print('{}: ca = {}, cr = {}'.format(target_acc, ca, cr))
    return df, ca, cr


def deal_with_time_string(time_string):
    time_string = time_string.replace('-','_')
    time_string = time_string.replace(' ','_')
    time_string = time_string.replace(':','_')
    return time_string

def assembly_data(df, ca, cr):
    output_data = OrderedDict()

    acc_plot = OrderedDict()
    acc_plot['a_report'] = df['a_report']
    acc_plot['a_estimated'] = df['a_estimated']
    output_data['acc_plot'] = acc_plot

    v_plot = OrderedDict()
    v_plot['v'] = df['v']
    output_data['v_plot'] = v_plot

    pitch_plot = OrderedDict()
    pitch_plot['pitch'] = df['pitch']
    output_data['pitch_plot'] = pitch_plot

    ca_plot = OrderedDict()
    ca_plot['ca'] = [ca] * len(df['pitch'])
    output_data['ca_plot'] = ca_plot

    cr_plot = OrderedDict()
    cr_plot['cr'] = [cr] * len(df['pitch'])
    output_data['cr_plot'] = cr_plot
    return output_data

def main(args):
    report_plotter = ReportPlotter('ReportGenerator')
    time_intervals = [
        # ['2023-08-05 08:00:00', '2023-08-05 18:00:00'],
        # ['2023-08-06 08:00:00', '2023-08-06 18:00:00'],
        # ['2023-08-07 08:00:00', '2023-08-07 18:00:00'],
        # ['2023-08-08 08:00:00', '2023-08-08 18:00:00'],
        # ['2023-08-09 08:00:00', '2023-08-09 18:00:00'],
        # ['2023-08-10 08:00:00', '2023-08-10 18:00:00'],
        # ['2023-08-11 08:00:00', '2023-08-11 18:00:00'],
        # ['2023-08-12 08:00:00', '2023-08-12 18:00:00'],
        ['2023-08-11 15:00:00', '2023-08-11 16:00:00'],
        
    ]     
    vehicle_names = [
        'pdb-l4e-b0003',
    ] 
    time_now = str(datetime.now())
    for vehicle_name in vehicle_names:
        for time_interval in time_intervals:
            init_state = np.array([[3.92,0.007]])
            init_state = init_state.T
            init_P = 1 * np.identity(init_state.shape[0]+1)
            rls = RGDF1TLSEstimator(init_state, init_P)

            ts_start = deal_with_time_string(time_interval[0])
            ts_end = deal_with_time_string(time_interval[1])
            output_data_dir = os.path.join(args.target_dir, 'data')
            if not os.path.exists(output_data_dir):
                os.makedirs(output_data_dir)
            output_name = os.path.join(output_data_dir,'{}_{}_to_{}'.format(vehicle_name,ts_start,ts_end))
            if args.repull_data or (not os.path.exists('{}.csv'.format(output_name))):   
                print('**** query data of {} from {} to {} ****'.format(vehicle_name, time_interval[0], time_interval[1]))
                start_ts = datetime.strptime(time_interval[0], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
                end_ts = datetime.strptime(time_interval[1], '%Y-%m-%d %H:%M:%S') - timedelta(hours=8)
                data = data_query(vehicle_name, start_ts, end_ts, output_name)
            else:
                print('**** data of {} from {} to {} has existed ****'.format(vehicle_name, time_interval[0], time_interval[1]))
                data = pd.read_csv('{}.csv'.format(output_name))

            df = filter_data(data)
            df.to_csv('{}_filtered.csv'.format(output_name))
            if(len(df) == 0):
                continue
            print('len of data = {}'.format(len(df)))
            date,time = time_interval[0].split(' ')
            params_fitting(rls, df, vehicle_name, date, 'coasting_acc')
            # data, ca, cr = params_fitting(rls, data, vehicle_name, date, 'a_report', use_resistance_without_v = False)
            df, ca, cr = params_fitting(rls, df, vehicle_name, date, 'a_report')
            output_data = assembly_data(df, ca ,cr)
            output_plot_dir = os.path.join(args.target_dir, 'plot_{}'.format(time_now))
            if not os.path.exists(output_plot_dir):
                os.makedirs(output_plot_dir)
            plot_signal(report_plotter, '{}/{}_{}_to_{}_{:.3f}_{:.5f}'.format(output_plot_dir,vehicle_name,ts_start,ts_end,ca,cr), output_data)
            print("=============================================================================================")



    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag report generator')
    file_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--target-dir', default=os.path.join(file_dirname,'coasting_data'))
    parser.add_argument('--data-file-name', default='data.csv', type=str)
    parser.add_argument('--repull-data', default=False, type=bool)
    args = parser.parse_args()
    main(args)