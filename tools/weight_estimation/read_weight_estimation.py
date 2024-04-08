#!/usr/bin/env python

from datetime import datetime, timedelta
import shutil
import time
from pluspy import db_utils
from control import control_command_pb2
from control import vehicle_state_pb2
import pandas as pd
import rosbag
import fastbag
import rospy
import argparse
import os
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from collections import deque, OrderedDict 
import yaml
import psycopg2
import plotly.subplots as sp
import plotly.graph_objects as go   
from pandas.plotting import register_matplotlib_converters
register_matplotlib_converters()

COLORS = ['red', 'blue', 'green', 'black','purple']
WEIGHT_UNIT = 1000
NUM_OF_FIGS_PER_ROW = 1
def get_histogram(data, bag_name):
    hist, bins = np.histogram(data, bins=200, range = (15000, 50000))
    fig = plt.figure(figsize=(8, 6))
    ax = fig.subplots()
    ax.bar(bins[:-1], hist, width=(bins[1]-bins[0]), align='edge')

    ax.set_title('Distribution of Random Data')
    ax.set_xlabel('Value')
    ax.set_ylabel('Frequency')
    output_filename = os.path.join(output_dir,'{}_histogram.png'.format(bag_name))
    plt.savefig(output_filename, dpi=100)

def get_valid_data(data, range):

    mean = np.mean(data)
    std_dev = np.std(data)

    lower_limit = mean - range * std_dev
    upper_limit = mean + range * std_dev

    filtered_data = [x for x in data if lower_limit <= x <= upper_limit]
    return filtered_data

def process(row):
    bag_filepath = row['fastbag_path']
    bag_name = bag_filepath.split('/')[-1].split('.bag')[0]
    print('bag_name:{}'.format(bag_name))
    split_result = bag_name.split('_')
    print('split_result:{}'.format(split_result))
    date = split_result[0]
    vehicle = split_result[1]
    date = date[:8]
    
    if ".bag" in bag_filepath:
        bag = rosbag.Bag(bag_filepath)
    elif ".db" in bag_filepath:
        bag = fastbag.Reader(bag_filepath)
        bag.open()
    else:
        print("incorrect bag {} to read, because it should be .db or .bag ended".format(bag_filepath))
        return
    weight_estimated = []
    weight_from_sensor = []
    weight_estimated_old = []
    total_weight = []
    speed = []
    control_pitch = []
    pitch_from_z = []
    time_drift = 0
    start_time = rospy.Time(bag.get_start_time() + time_drift)
    end_time = rospy.Time(bag.get_start_time() + time_drift + args.read_bag_length)

    weight_gt = get_weight_gt(vehicle, pd.to_datetime(datetime.fromtimestamp(bag.get_start_time()) + timedelta(hours=8)))
    print('weight_gt:{}'.format(weight_gt))
    if(pd.isna(weight_gt)):
        return None
    
    for topic, msg, bag_time in bag.read_messages(topics=['/vehicle/control_cmd','/vehicle/status'], start_time = start_time, end_time = end_time):
        if(topic == '/vehicle/control_cmd'):
            control_cmd.ParseFromString(msg.data)
            weight_estimated.append(float(control_cmd.debug_cmd.weight_estimated))
            weight_from_sensor.append(float(control_cmd.debug_cmd.weight_calibrated))
            weight_estimated_old.append(float(control_cmd.debug_cmd.total_weight_estimation))
        elif(topic == '/vehicle/status'):
            vehicle_state.ParseFromString(msg.data)
            total_weight.append(float(vehicle_state.total_weight))    
            control_pitch.append(float(vehicle_state.pitch))    
            pitch_from_z.append(float(vehicle_state.pitch_from_z_differentiator))    
            speed.append(float(vehicle_state.v))    
            
        
    if(len(weight_estimated)==0):
        return None    
    
    mean_of_weight_estimated = sum(weight_estimated)/len(weight_estimated)
    if(mean_of_weight_estimated==0 or mean_of_weight_estimated==15000 or mean_of_weight_estimated==25000 or mean_of_weight_estimated==50000):
        return None
        
    error_of_end_value = (weight_estimated[-1] - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100
    error_of_mean_value = (mean_of_weight_estimated - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    if(len(weight_from_sensor)==0):
        mean_of_weight_from_sensor = np.nan
    else:
        mean_of_weight_from_sensor = sum(weight_from_sensor)/len(weight_from_sensor)
    if(np.isnan(mean_of_weight_from_sensor)):
        error_of_sensor_value = np.nan
    else:
        error_of_sensor_value = (mean_of_weight_from_sensor - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    if(len(total_weight)==0):
        mean_of_total_weight = np.nan
    else:
        mean_of_total_weight = sum(total_weight)/len(total_weight)
    if(np.isnan(mean_of_total_weight)):
        error_of_total_weight = np.nan
    else:
        error_of_total_weight = (mean_of_total_weight - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    if(len(weight_estimated_old)==0):
        mean_of_weight_estimated_old = np.nan
    else:
        mean_of_weight_estimated_old = sum(weight_estimated_old)/len(weight_estimated_old)
    if(mean_of_weight_estimated_old==7500):
        return None
    if(np.isnan(mean_of_weight_estimated_old)):
        error_of_estimation_old_value = np.nan
    else:
        error_of_estimation_old_value = (mean_of_weight_estimated_old - weight_gt * WEIGHT_UNIT) / (weight_gt * WEIGHT_UNIT) * 100

    if(len(control_pitch)==0 or len(pitch_from_z)==0):
        pitch_rmse = np.nan
    else:
        differences = np.array(control_pitch) -  np.array(pitch_from_z)
        squared_diff = differences ** 2
        mean_squared_diff = np.mean(squared_diff)
        pitch_rmse = np.sqrt(mean_squared_diff)
    
    if(len(speed)==0):
        mean_speed = np.nan
    else:
        mean_speed = sum(speed)/len(speed)
    re = pd.DataFrame([{'bag_name':bag_name,
                        'bag_filepath':bag_filepath,
                        'vehicle':vehicle,
                        'date':str(date),
                        'start_time':datetime.fromtimestamp(bag.get_start_time()) + timedelta(hours=8),
                        'end_time': datetime.fromtimestamp(bag.get_end_time()) + timedelta(hours=8),
                        'weight_gt': weight_gt,
                        'end_of_weight_estimated' : weight_estimated[-1],
                        'mean_of_weight_estimated' : mean_of_weight_estimated,
                        'mean_of_weight_from_sensor' : mean_of_weight_from_sensor,
                        'mean_of_total_weight' : mean_of_total_weight,
                        'mean_of_weight_estimated_old' : mean_of_weight_estimated_old,
                        'error_of_end_value' : error_of_end_value,
                        'error_of_mean_value' : error_of_mean_value,
                        'error_of_sensor_value' : error_of_sensor_value,
                        'error_of_estimation_old_value' : error_of_estimation_old_value,
                        'error_of_total_weight' : error_of_total_weight,
                        'weather': row['weather'],
                        'wind_heading': row['wind_heading'],
                        'vehicle_heading': row['vehicle_heading'],
                        'wind_speed': row['wind_speed'],
                        'relative_speed': row['relative_speed'],
                        'pitch_rmse': pitch_rmse,
                        'mean_speed':mean_speed
                        }])
    return re

def get_weight_gt(vehcile_name, start_time):
    weight_gt_record_filepath = os.path.join(weight_gt_record_dir, '{}.csv'.format(vehcile_name))
    weight_gt_record = pd.read_csv(weight_gt_record_filepath)
    weight_gt_record['start_time'] = pd.to_datetime(weight_gt_record['start_time'])
    weight_gt_record['end_time'] = pd.to_datetime(weight_gt_record['end_time'])
    print('bag start time:{}'.format(start_time))
    result_row = weight_gt_record[(weight_gt_record['start_time'] <= start_time) & (weight_gt_record['end_time'] >= start_time)]
    if(result_row.shape[0]==0):
        return np.nan
    return result_row['weight_gt'].iloc[0]

def compute_error_quantile(data, column_names, quantiles):
    result = pd.DataFrame()
    for column_name in column_names:
        re = data.groupby('vehicle')[column_name].agg([
            ('P({}<{}%)'.format(column_name,quantiles[0]),lambda x: (abs(x) < quantiles[0]).mean().round(4)*100),
            ('P({}<{}%)'.format(column_name,quantiles[1]),lambda x: (abs(x) < quantiles[1]).mean().round(4)*100),
            ('P({}<{}%)'.format(column_name,quantiles[2]),lambda x: (abs(x) < quantiles[2]).mean().round(4)*100)]
        ).reset_index()
        result = pd.concat([result,re],axis=1)
    result = result.loc[:, ~result.columns.duplicated()]
    result.to_csv(os.path.join(output_dir,'error_distribution.csv'), mode = 'w')
    result.set_index(["vehicle"], inplace=True)
    return result

def result_plot(data):
    data['weight_gt'] *= WEIGHT_UNIT
    data['error_of_mean_value_kg'] = np.where((data['mean_of_weight_estimated'] == 0) | (data['weight_gt'] == 0), 0, data['mean_of_weight_estimated'] - data['weight_gt'])
    data['error_of_sensor_value_kg'] = np.where((data['mean_of_weight_from_sensor'] == 0) | (data['weight_gt'] == 0), 0, data['mean_of_weight_from_sensor'] - data['weight_gt'])
    data['error_of_total_weight_kg'] = np.where((data['mean_of_total_weight'] == 0) | (data['weight_gt'] == 0), 0, data['mean_of_total_weight'] - data['weight_gt'])
    data['error_of_estimation_old_value_kg'] = np.where((data['mean_of_weight_estimated_old'] == 0) | (data['weight_gt'] == 0), 0, data['mean_of_weight_estimated_old'] - data['weight_gt'])
    data['pitch_rmse'] *= 10000
    
    error_quantile = compute_error_quantile(data,
                                            ['error_of_end_value',
                                             'error_of_mean_value',
                                             'error_of_sensor_value',
                                             'error_of_total_weight',
                                             'error_of_estimation_old_value'],[6,10,20])
    
    save_days_with_max_err(data, ['error_of_mean_value','error_of_sensor_value','error_of_total_weight','error_of_estimation_old_value','pitch_rmse'], args.num_per_vehicle, args.num_per_day)
    line_plot_for_day_result(data, ['error_of_mean_value','pitch_rmse'], 'result_comparison_with_pitch')
    days_result_plot(data, ['weight_gt','mean_of_weight_estimated','mean_of_weight_estimated_old','mean_of_total_weight','mean_of_weight_from_sensor'], 'result_comparison', error_quantile)
    days_result_plot(data,['error_of_mean_value_kg','error_of_estimation_old_value_kg','error_of_sensor_value_kg','error_of_total_weight_kg'], 'error_kg_comparsion', error_quantile)
    days_result_plot(data,['error_of_mean_value','error_of_estimation_old_value','error_of_sensor_value','error_of_total_weight'], 'error_percentage_comparsion', error_quantile)
    
    scatter_plot(data, ['error_of_mean_value'], error_quantile)
    histogram_plot(data, ['error_of_mean_value'], error_quantile)
    scatter_plot(data, ['error_of_sensor_value'], error_quantile)
    histogram_plot(data, ['error_of_sensor_value'], error_quantile)
    scatter_plot(data, ['error_of_estimation_old_value'], error_quantile)
    histogram_plot(data, ['error_of_estimation_old_value'], error_quantile)

def save_days_with_max_err(data, column_names, num_per_vehcile, num_per_day, abs_value = False):
    result = data.groupby(['vehicle', 'date'])
    if(abs_value):
        result = result[column_names].apply(lambda x: abs(x).mean()).reset_index()
    else:
        result = result[column_names].apply(lambda x: x.mean()).reset_index()
    print(result)
    result = result[np.isnan(result['pitch_rmse'])]
    print(result)
    def select_max_err_days(group, target_column, threshold, num):
        abs_values = group[target_column].abs()
        selected_rows = group[abs_values > threshold]
        if len(selected_rows) < num:
            selected_rows = group.loc[abs_values.nlargest(num).index]
        print(selected_rows)
        return selected_rows
    
    top_N = result.groupby('vehicle', group_keys=False).apply(select_max_err_days, 
                                                              target_column = column_names[0], 
                                                              num = num_per_vehcile,
                                                              threshold = 10)    
    # print('top N:{}'.format(top_N))
    max_error_days = pd.DataFrame()
    for i, row in top_N.iterrows():
        vehicle_name = row['vehicle']
        date = str(row['date'])
        date = date.replace('-','')
        re = data[(data['vehicle'] == vehicle_name) & (data['date'] == float(date)) & (data['mean_speed'] > 15)]
        # print('re:{}'.format(re))
        # print('date1:{}'.format(type(date)))
        # print('date2:{}'.format(type(data['date'][0])))
        if(re.shape[0] > 0):
            if(re.shape[0] > num_per_day):
                re = re.sample(num_per_day)
            for column_name in column_names:
                re['{}_per_day'.format(column_name)] = row[column_name]
            max_error_days = pd.concat([max_error_days, re], axis=0, sort=True)
    max_error_days.to_csv(os.path.join(output_dir,'days_with_max_{}.csv'.format(column_names[0])))

def line_plot_for_day_result(data, column_names, output_name, abs_value = False):
    result = data.groupby(['vehicle', 'date'])
    if(abs_value):
        result = result[column_names].apply(lambda x: abs(x).mean()).reset_index()
    else:
        result = result[column_names].apply(lambda x: x.mean()).reset_index()
        
    vehicles = result['vehicle'].unique()
    num_rows = len(vehicles) // NUM_OF_FIGS_PER_ROW + (len(vehicles) % NUM_OF_FIGS_PER_ROW > 0)
    fig = sp.make_subplots(rows=num_rows, cols=NUM_OF_FIGS_PER_ROW, subplot_titles=['{}'.format(vehicle,) for vehicle in vehicles])
    for i, vehicle in enumerate(vehicles, start=1):
        vehicle_data = result[result['vehicle'] == vehicle]
        row_idx = (i - 1) // NUM_OF_FIGS_PER_ROW + 1
        col_idx = (i - 1) % NUM_OF_FIGS_PER_ROW + 1
        for i in range(len(column_names)):
            trace = go.Scatter(x=pd.to_datetime(vehicle_data['date'],format='%Y%m%d'), y=vehicle_data[column_names[i]], mode='lines+markers',
                        name='{}_{}'.format(vehicle,column_names[i]),marker=dict(color=COLORS[i]), legendgroup='{}'.format(column_names[i]))
            fig.add_trace(trace, row=row_idx, col=col_idx)
            
        fig.update_xaxes(title_text=' ', tickangle=0, row=row_idx, col=col_idx)

    fig.update_layout(title_text=column_names[0].replace('_',' ').upper(), title=dict(x=0.5, y=0.98), title_font=dict(size=40),
                  height=1500, width=2500, xaxis=dict(exponentformat='none'))
    fig.write_html(os.path.join(output_dir,'date_plot_of_{}.html'.format(output_name)))
                                           
def days_result_plot(data, column_names, output_name, error_quantile, abs_value = False):
    result = data.groupby(['vehicle', 'date'])
    if(abs_value):
        result = result[column_names].apply(lambda x: abs(x).mean()).reset_index()
    else:
        result = result[column_names].apply(lambda x: x.mean()).reset_index()
    result.to_csv(os.path.join(output_dir,'days_{}.csv'.format(column_names[0])))

    vehicles = result['vehicle'].unique()
    num_rows = len(vehicles) // NUM_OF_FIGS_PER_ROW + (len(vehicles) % NUM_OF_FIGS_PER_ROW > 0)
    if(output_name == 'error_percentage_comparsion'):
        fig = sp.make_subplots(rows=num_rows, 
                            cols=NUM_OF_FIGS_PER_ROW, 
                            subplot_titles=['{}\nP(e<20%)={:.2f}% P(e<10%)={:.2f}% P(e<6%)={:.2f}%'.format(
                            vehicle,
                            error_quantile.loc[vehicle,'P({}<20%)'.format(column_names[0])], 
                            error_quantile.loc[vehicle,'P({}<10%)'.format(column_names[0])],
                            error_quantile.loc[vehicle,'P({}<6%)'.format(column_names[0])]
                            ) for vehicle in vehicles])
    else:
        fig = sp.make_subplots(rows=num_rows, 
                            cols=NUM_OF_FIGS_PER_ROW, 
                            subplot_titles=['{}\nP(e<20%)={:.2f}% P(e<10%)={:.2f}% P(e<6%)={:.2f}%'.format(
                            vehicle,
                            error_quantile.loc[vehicle,'P(error_of_mean_value<20%)'], 
                            error_quantile.loc[vehicle,'P(error_of_mean_value<10%)'],
                            error_quantile.loc[vehicle,'P(error_of_mean_value<6%)']
                            ) for vehicle in vehicles])

    for i, vehicle in enumerate(vehicles, start=1):
        vehicle_data = result[result['vehicle'] == vehicle]
        row_idx = (i - 1) // NUM_OF_FIGS_PER_ROW + 1
        col_idx = (i - 1) % NUM_OF_FIGS_PER_ROW + 1
        for i in range(len(column_names)):
            trace = go.Bar(x=pd.to_datetime(vehicle_data['date'],format='%Y%m%d'), y=vehicle_data[column_names[i]],
                        name='{}_{}'.format(vehicle,column_names[i]),marker=dict(color=COLORS[i]), legendgroup='{}'.format(column_names[i]))
            fig.add_trace(trace, row=row_idx, col=col_idx)
            
        fig.update_xaxes(title_text=' ', tickangle=0, row=row_idx, col=col_idx)

    fig.update_layout(title_text=column_names[0].replace('_',' ').upper(), title=dict(x=0.5, y=0.98), title_font=dict(size=40),
                  height=1500, width=2500, xaxis=dict(exponentformat='none'))
    fig.write_html(os.path.join(output_dir,'date_plot_of_{}.html'.format(output_name)))

def scatter_plot(df, column_names, error_quantile):
    vehicles = df['vehicle'].unique()
    num_rows = len(vehicles) // 3 + (len(vehicles) % 3 > 0)
    subplot_titles = []
    for vehicle in vehicles:
        if(pd.isna(vehicle)):
            continue
        subplot_titles.append(('{}: P(e<20%)={:.2f}% P(e<10%)={:.2f}% P(e<6%)={:.2f}%'.format(vehicle, 
                                                                                                 error_quantile.loc[vehicle,'P({}<20%)'.format(column_names[0])], 
                                                                                                 error_quantile.loc[vehicle,'P({}<10%)'.format(column_names[0])],
                                                                                                 error_quantile.loc[vehicle,'P({}<6%)'.format(column_names[0])])))

    fig = sp.make_subplots(rows=num_rows, cols=3, subplot_titles=subplot_titles)
    for i, vehicle in enumerate(vehicles, start=1):
        vehicle_data = df[df['vehicle'] == vehicle]
        row_idx = (i - 1) // 3 + 1
        col_idx = (i - 1) % 3 + 1
        for i in range(len(column_names)):
            trace = go.Scatter(x=vehicle_data['weight_gt']*WEIGHT_UNIT, y=vehicle_data[column_names[i]], mode='markers', name='{}_{}'.format(vehicle, column_names[i]), marker=dict(color=COLORS[i]))
            fig.add_trace(trace, row=row_idx, col=col_idx)

        fig.update_xaxes(title_text='Error', row=row_idx, col=col_idx)

    fig.update_layout(title_text=column_names[0].replace('_',' ').upper(), title=dict(x=0.5, y=0.98), title_font=dict(size=40),
                  height=1500, width=2500)
    fig.write_html(os.path.join(output_dir,'scatter_of_{}.html'.format(column_names[0])))

def histogram_plot(df, column_names, error_quantile):
    vehicles = df['vehicle'].unique()
    num_rows = len(vehicles) // 3 + (len(vehicles) % 3 > 0)
    subplot_titles = []
    for vehicle in vehicles:
        if(pd.isna(vehicle)):
            continue
        subplot_titles.append(('{}: P(e<20%)={:.2f}% P(e<10%)={:.2f}% P(e<6%)={:.2f}%'.format(vehicle, 
                                                                                                 error_quantile.loc[vehicle,'P({}<20%)'.format(column_names[0])], 
                                                                                                 error_quantile.loc[vehicle,'P({}<10%)'.format(column_names[0])],
                                                                                                 error_quantile.loc[vehicle,'P({}<6%)'.format(column_names[0])])))

    fig = sp.make_subplots(rows=num_rows, cols=3, subplot_titles=subplot_titles)
    for i, vehicle in enumerate(vehicles, start=1):
        vehicle_data = df[df['vehicle'] == vehicle]
        row_idx = (i - 1) // 3 + 1
        col_idx = (i - 1) % 3 + 1 
        for i in range(len(column_names)):
            trace = go.Histogram(x=vehicle_data[column_names[i]], autobinx=False, xbins=dict(size=2), name='{}_{}'.format(vehicle, column_names[i]), marker=dict(color=COLORS[i],line=dict(color='black', width=1)))
            fig.add_trace(trace, row=row_idx, col=col_idx)
        fig.update_yaxes(title_text='Frequency', row=row_idx, col=col_idx)

    fig.update_layout(title_text=column_names[0].replace('_',' ').upper(), title=dict(x=0.5, y=0.98), title_font=dict(size=40),
                  height=1500, width=2500)
    fig.write_html(os.path.join(output_dir,'histogram_of_{}.html'.format(column_names[0])))

def main(args):
    if(not args.result_existed):
        result = pd.DataFrame()
        if(args.mode == 'batch_list'):
            i = 0
            df = pd.read_csv(bag_record_filepath)
            for i, row in df.iterrows():
                i += 1
                print('===================={}/{}===================='.format(i, df.shape[0]))
                if(pd.isna(row['fastbag_path'])):
                    continue
                re = process(row)
                if(re is not None):
                    result = result.append(re,ignore_index=True) 
                
        # elif(args.mode == 'batch_folder'):
        #     for bag_name in os.listdir(args.target_folder):
        #         bag_dir = os.path.join(args.target_folder, bag_name)
        #         if(not os.path.isdir(bag_dir)):
        #             continue
        #         bag_filepath = os.path.join(bag_dir, 'replay_{}.bag'.format(bag_name))
        #         if(not os.path.exists(bag_filepath)):
        #             continue
        #         re = process(bag_filepath, bag_name)
        #         if(re is not None):
        #             result = result.append(re,ignore_index=True)   
        # elif(args.mode == 'single'):
        #     bag_name = args.bag_name
        #     bag_filepath = os.path.join(args.target_folder, bag_name, 'replay_{}.bag'.format(bag_name))
        #     re = process(bag_filepath, bag_name)
        #     if(re is not None):
        #         result = result.append(re,ignore_index=True)           
    else:
        result = pd.read_csv(args.result_filepath)
        
    target_columns = ['bag_name',
                    'bag_filepath',
                    'vehicle',
                    'date',
                    'start_time', 
                    'end_time',
                    'end_of_weight_estimated', 
                    'mean_of_weight_estimated', 
                    'mean_of_weight_from_sensor',
                    'mean_of_weight_estimated_old',
                    'mean_of_total_weight',
                    'error_of_end_value',
                    'error_of_mean_value',
                    'error_of_sensor_value',
                    'error_of_estimation_old_value',
                    'error_of_total_weight',
                    'weight_gt',
                    'weather',
                    'wind_heading',
                    'vehicle_heading',
                    'wind_speed',
                    'relative_speed',
                    'pitch_rmse',
                    'mean_speed']
    result.to_csv(os.path.join(output_dir,'weight_estimation_analysis_result.csv'), mode = 'w', columns = target_columns)

    shutil.copyfile(bag_record_filepath, os.path.join(output_dir,'selected_fastbag_path.csv'))
    shutil.copyfile(bag_record_filepath, os.path.join(output_dir,'fastbag_path_with_weather.csv'))
    result_plot(result)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('weight comparison')
    control_cmd = control_command_pb2.ControlCommand()
    vehicle_state = vehicle_state_pb2.VehicleState()
    root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
    
    bag_record_filepath = os.path.join(root_dir, 'bags_queried_for_weight_estimation/selected_fastbag_path.csv')
    weight_gt_record_dir = os.path.join(root_dir, 'weight_gt_record')
    
    parser.add_argument('--mode', choices=['batch_folder', 'batch_list', 'single_bag'], default='batch_list', type=str)
    parser.add_argument('--target-folder', default='', type=str)
    parser.add_argument('--bag-name', default='', type=str)
    parser.add_argument('--bag-record-filepath', default=bag_record_filepath, type=str)
    parser.add_argument('--output-dir', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analysis_output'), type=str)
    parser.add_argument('--result-existed', default = False, type = bool)
    parser.add_argument('--result-filepath', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analysis_output','weight_estimation_analysis_result.csv'), type=str)
    parser.add_argument('--num-per-vehicle', default=3, type=int)
    parser.add_argument('--num-per-day', default=1, type=int)
    parser.add_argument('--read-bag-length', default=100, type=int)
    args = parser.parse_args()
    current_time = str(datetime.now() + timedelta(hours = 8)).replace(' ','_').replace(':','-').replace('.','-')
    if(args.mode == 'batch_folder'):
        output_dir = os.path.join(args.target_folder, '{}'.format(current_time))
    else:
        output_dir = os.path.join(args.output_dir, '{}'.format(current_time))
        
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    print('output_dir:{}'.format(output_dir))
    main(args)