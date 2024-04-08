#!/usr/bin/env python3

import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import os
# matplotlib.use('TkAgg')

TIME = 0
SPEED = 1
GEAR = 2
PEDAL_OUTPUT = 3
ACC = 4
ENGINR_RPM = 5
PITCH = 6
WEIGHT = 7
BRAKE = 8
FUEL_RATE = 9
RETARDER = 10
BRAKE_SPEED = 11
PEDAL_SPEED = 12
BSFC = 13

WIDTH = 40
HEIGHT = 20
SCALE = 1.5

TARGET_SIGNAL_LIST = ['time', 'speed', 'gear', 'pedal_output', 'acc', 'engine_rpm', 'pitch',
                     'weight', 'brake','fuel_rate','retarder', 'brake_speed', 'pedal_speed', 'bsfc']


COLOR_LIST = ['r','g','b','c','k','y','m','orange','indigo','gray','purple','pink','brown','teal', 'turquoise', 'tan','aqua', 'lavender','violet']

PRECISION = 4

def plot3d_gear_shift(data_dict, x_signal, y_signal, z_signal,output_dir):
    fig = plt.figure()
    if(len(data_dict) ==1 ):
        subplot_layout = [1,1]
    else:
        subplot_layout = [2,2]
    
    i = 0
    for data_name, data in data_dict.items():
        sub_ax = fig.add_subplot(subplot_layout[0],subplot_layout[1],i+1,projection='3d')
        sub_fig = sub_ax.scatter(data.iloc[:,x_signal], data.iloc[:,y_signal], data.iloc[:,z_signal], c=data.iloc[:,GEAR], cmap="tab20")
        sub_ax.set_title(data_name)
        sub_ax.grid()   
        sub_ax.set_xlabel(TARGET_SIGNAL_LIST[x_signal])
        sub_ax.set_ylabel(TARGET_SIGNAL_LIST[y_signal])
        sub_ax.set_zlabel(TARGET_SIGNAL_LIST[z_signal])
        label = list(pd.unique(data.iloc[:,GEAR]))
        label.sort()
        sub_ax.legend(handles=sub_fig.legend_elements()[0], labels=label, loc=4)
        i += 1

    plt.savefig(os.path.join(output_dir,'3d_{}_{}_{}.png'.format(TARGET_SIGNAL_LIST[x_signal],TARGET_SIGNAL_LIST[y_signal],TARGET_SIGNAL_LIST[z_signal])))

def plot_bsfc(data_dict, x_signal, y_signal, slide_info,output_dir):
    print('plot {} with {}, compared to bsfc'.format(TARGET_SIGNAL_LIST[x_signal],TARGET_SIGNAL_LIST[y_signal]))
    if(len(data_dict) ==1 ):
        subplot_layout = [1,1]
    else:
        subplot_layout = [2,2]      
    fig, ax = plt.subplots(subplot_layout[0], subplot_layout[1],figsize=(WIDTH * SCALE, HEIGHT * SCALE))
    
    i = 0
    interval_start = slide_info[0]
    interval_end = slide_info[1]
    interval_length = (interval_end - interval_start)/slide_info[2]
    for data_name, data in data_dict.items():
        if(len(data_dict) == 1):
            sub_ax = ax
        else:
            sub_ax = ax[int(i/2),int(i%2)]

        sub_ax.scatter(x=data.iloc[:,x_signal], y=data.iloc[:,y_signal], c=data.iloc[:,GEAR], cmap="tab20")
        bsfc = data.iloc[:,FUEL_RATE] / (data.iloc[:,PEDAL_OUTPUT] * data.iloc[:,ENGINR_RPM])
        for j in range(slide_info[2]):
            interval = [interval_start + j * interval_length, interval_start + (j+1) * interval_length]
            df = data[(bsfc > interval[0]) & (bsfc < interval[1])]
            sub_ax.scatter(x=df.iloc[:,x_signal], y=df.iloc[:,y_signal], c=COLOR_LIST[j], marker='+', label='{}~{}'.format(round(interval[0],PRECISION),round(interval[1],PRECISION)))

        sub_ax.set_title(data_name)   
        sub_ax.set_xlabel(TARGET_SIGNAL_LIST[x_signal])
        sub_ax.set_ylabel(TARGET_SIGNAL_LIST[y_signal])
        sub_ax.legend(loc=1)
        sub_ax.grid()
        i += 1

    plt.savefig(os.path.join(output_dir,'bsfc_{}_{}_with_bsfc_from_{}_to_{}.png'.format(TARGET_SIGNAL_LIST[x_signal],
                                                                                             TARGET_SIGNAL_LIST[y_signal],
                                                                                             slide_info[0],
                                                                                             slide_info[1])))
    plt.close()

def plot2d_gear_shift_compare(data_dict, x_signal, y_signal, compare_signal, slide_info,output_dir):
    print('plot {} with {}, compared to {}'.format(TARGET_SIGNAL_LIST[x_signal],TARGET_SIGNAL_LIST[y_signal],TARGET_SIGNAL_LIST[compare_signal]))
    if(len(data_dict) ==1 ):
        subplot_layout = [1,1]
    else:
        subplot_layout = [2,2]      
    fig, ax = plt.subplots(subplot_layout[0], subplot_layout[1],figsize=(WIDTH * SCALE, HEIGHT * SCALE))
    
    i = 0
    interval_start = slide_info[0]
    interval_end = slide_info[1]
    interval_length = (interval_end - interval_start)/slide_info[2]
    for data_name, data in data_dict.items():
        if(len(data_dict) == 1):
            sub_ax = ax
        else:
            sub_ax = ax[int(i/2),int(i%2)]

        sub_ax.scatter(x=data.iloc[:,x_signal], y=data.iloc[:,y_signal], c=data.iloc[:,GEAR], cmap="tab20")

        for j in range(slide_info[2]):
            interval = [interval_start + j * interval_length, interval_start + (j+1) * interval_length]
            df = data[(data.iloc[:,compare_signal] > interval[0]) & (data.iloc[:,compare_signal] < interval[1])]
            sub_ax.scatter(x=df.iloc[:,x_signal], y=df.iloc[:,y_signal], c=COLOR_LIST[j], marker='+', label='{}~{}'.format(round(interval[0],PRECISION),round(interval[1],PRECISION)))
        sub_ax.set_title(data_name)   
        sub_ax.set_xlabel(TARGET_SIGNAL_LIST[x_signal])
        sub_ax.set_ylabel(TARGET_SIGNAL_LIST[y_signal])
        sub_ax.legend(loc=1)
        sub_ax.grid()
        i += 1

    plt.savefig(os.path.join(output_dir,'compare_{}_{}_with_differnent_{}_from_{}_to_{}.png'.format(TARGET_SIGNAL_LIST[x_signal],
                                                                                                         TARGET_SIGNAL_LIST[y_signal],
                                                                                                         TARGET_SIGNAL_LIST[compare_signal],
                                                                                                         slide_info[0],
                                                                                                         slide_info[1])))
    plt.close()

def plot2d_gear_shift(data_dict, x_signal, y_signal,output_dir):
    print('plot {} with {}'.format(TARGET_SIGNAL_LIST[x_signal],TARGET_SIGNAL_LIST[y_signal]))
    if(len(data_dict) ==1 ):
        subplot_layout = [1,1]
    else:
        subplot_layout = [2,2]      
    fig, ax = plt.subplots(subplot_layout[0], subplot_layout[1],figsize=(WIDTH * SCALE, HEIGHT * SCALE))
    
    i = 0
    for data_name, data in data_dict.items():
        if(len(data_dict) == 1):
            sub_ax = ax
        else:
            sub_ax = ax[int(i/2),int(i%2)]
        sub_fig = sub_ax.scatter(x=data.iloc[:,x_signal], y=data.iloc[:,y_signal], c=data.iloc[:,GEAR], cmap="tab20")
        sub_ax.set_title(data_name)
        sub_ax.set_xlabel(TARGET_SIGNAL_LIST[x_signal])
        sub_ax.set_ylabel(TARGET_SIGNAL_LIST[y_signal])
        label = list(pd.unique(data.iloc[:,GEAR]))
        label.sort()
        sub_ax.legend(handles=sub_fig.legend_elements()[0], labels=label, loc=4)
        sub_ax.grid()
        i += 1

    plt.savefig(os.path.join(output_dir,'2d_{}_{}.png'.format(TARGET_SIGNAL_LIST[x_signal],TARGET_SIGNAL_LIST[y_signal])))
    plt.close()

if __name__=='__main__':
    fe_mode = 'DE'
    target_vehicles = ['j7-l4e-c0004_LFWSRXSJ7M1F45293','j7-l4e-LFWSRXSJ6M1F50503','j7-l4e-LFWSRXSJ8M1F50115']
    for vehicle_name in target_vehicles:
        print('=======================plot figures for {}======================='.format(vehicle_name))
        source_filepath = os.path.join(os.getenv('HOME'),'data/database',fe_mode,vehicle_name)
        output_dir = os.path.join(source_filepath,'figure')
        if os.path.exists(output_dir):
            print("the dir existed")
        else:
            os.makedirs(output_dir) 
        auto_upshift_path = os.path.join(source_filepath,'auto_upshift.xlsx')
        auto_downshift_path = os.path.join(source_filepath,'auto_downshift.xlsx')
        manual_upshift_path = os.path.join(source_filepath,'manual_upshift.xlsx')
        manual_downshift_path = os.path.join(source_filepath,'manual_downshift.xlsx')

        data_dict = {}
        data_dict['auto_upshift'] = pd.read_excel(auto_upshift_path, engine='openpyxl')
        data_dict['auto_downshift'] = pd.read_excel(auto_downshift_path, engine='openpyxl')
        data_dict['manual_upshift'] = pd.read_excel(manual_upshift_path, engine='openpyxl')
        data_dict['manual_downshift'] = pd.read_excel(manual_downshift_path, engine='openpyxl')


        for i in range(PEDAL_OUTPUT, PEDAL_SPEED + 1):
            plot2d_gear_shift(data_dict,SPEED,i,output_dir)

        for i in range(PEDAL_OUTPUT, PEDAL_SPEED + 1):
            plot2d_gear_shift(data_dict,ENGINR_RPM,i,output_dir)

        
        plot_bsfc(data_dict,SPEED,PEDAL_OUTPUT,[0,0.1,len(COLOR_LIST)],output_dir)
        plot2d_gear_shift_compare(data_dict,SPEED,PEDAL_OUTPUT,PEDAL_SPEED,[0,0.2,len(COLOR_LIST)],output_dir)
        plot2d_gear_shift_compare(data_dict,SPEED,PEDAL_OUTPUT,PEDAL_SPEED,[-0.2,0,len(COLOR_LIST)],output_dir)

        plot_bsfc(data_dict,ENGINR_RPM,PEDAL_OUTPUT,[0,0.1,len(COLOR_LIST)],output_dir)
        plot2d_gear_shift_compare(data_dict,ENGINR_RPM,PEDAL_OUTPUT,PEDAL_SPEED,[0,0.2,len(COLOR_LIST)],output_dir)
        plot2d_gear_shift_compare(data_dict,ENGINR_RPM,PEDAL_OUTPUT,PEDAL_SPEED,[-0.2,0,len(COLOR_LIST)],output_dir)
