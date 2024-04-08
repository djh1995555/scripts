#!/usr/bin/env python
import math
import os
import sys
import time
import argparse
import logging
import subprocess
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from scipy import signal
from subprocess import PIPE, STDOUT
from utils.data_container import DataContainer
from utils.logger_setting import *
from utils.report_index_dict import *
from utils.bag_finder import BagFinder
from utils.report_plotter import ReportPlotter
import rosbag
import fastbag
import rospy

# logger = logging.getLogger(__name__)
TARGET_ACC = ACC_REPORT
TARGET_DATA_NAME_LIST = [THROTTLE_CMD,
                            THROTTLE_INPUT,
                            THROTTLE_OUTPUT,
                            BRAKE_CMD,
                            BRAKE_INPUT,
                            BRAKE_OUTPUT,
                            STEERING_ANGLE,
                            RETARDER,
                            ENGINE_BRAKE,
                            GEAR_REPORT,
                            GEAR_RATIO,
                            DBW_ENABLED,
                            PITCH_CONTROL,
                            V_CURRENT_WHEEL,
                            ACC_REPORT,
                            A_CHASSIS,
                            A_IMU,
                            ACC_FROM_IMU_FILTERED,
                            COASTING_ACC,
                            ENGINE_FRICTION_TORQUE]
class ParamEstimator:
    def __init__(self,args):
        self._args = args
        self._data_container = DataContainer(False)
        self._report_plotter = ReportPlotter('ReportGenerator')
        self._figure_height = 600
    
    def generate_report(self, bag_filename):
        plots_dict = REPORT_INDEX_DICT
        subplot_figure = None
        plot_html_str = ""

        self._data_container.compute_acc()
        
        # fuel_per_100km = self._data_container.compute_fuel_consumption()
        # print("fuel_per_100km = {}".format(fuel_per_100km))            
        # output_filename = os.path.join(output_dir,bag_filename+'_'+str(fuel_per_100km)+'.html')
        
        output_filename = os.path.join(self._output_dir,bag_filename+'.html')
        
        figure_list = []
        for subdict_title, subdict in plots_dict.items():
            legend_list = []
            value_list = []
            time_list = []
            for signal_name in subdict:
                legend_list.append(signal_name)
                data_dict = self._data_container.get_data_dict()
                plot_data = data_dict[signal_name]
                data = np.array([x[1] for x in plot_data.data])
                # if(signal_name == V_ESTIMATED):
                #     self.generate_histogram(data)
                data_time = np.array([x[0] - plot_data.data[0][0] for x in plot_data.data])
                time_list.append(data_time)
                value_list.append(data)

            subplot = self._report_plotter.plot_figure_plotly(x_list = time_list, 
                                                y_list = value_list,
                                                legend_list = legend_list,
                                                x_label = 'time / s',
                                                y_label = '',
                                                title = subdict_title,
                                                legend_prefix = '',
                                                figure_height=self._figure_height,)
            figure_list.append(subplot)

        subplot_figure_list = [(i + 1, 1, fig) for i, fig in enumerate(figure_list)]
        subplot_figure = self._report_plotter.append_figure_to_subplot_plotly(subplot_figure_list, len(figure_list), 1, template="plotly_dark", subplot_fig=subplot_figure)
        plot_html_str += self._report_plotter.get_fuel_fig_html_str({"Comparison": subplot_figure})
        html_str = self._report_plotter.generate_html_fuel_report(plot_html_str)
        with open(output_filename, 'w') as f:
            f.write(html_str)
        print("\n========report of {} is generated!========".format(bag_filename))
     
    def filter_data(self, df, filtered_file_name):
        condition = (df[GEAR_REPORT] == 0)
        df.loc[condition, GEAR_RATIO] = 0
        condition = (df[GEAR_REPORT] == 12)
        df.loc[condition, GEAR_RATIO] = 1.0
        condition = (df[GEAR_REPORT] == 11)
        df.loc[condition, GEAR_RATIO] = 1.2920
        condition = (df[GEAR_RATIO] == 0)
        df.loc[condition, GEAR_REPORT] = 0
        condition = (df[GEAR_RATIO] == 1.0)
        df.loc[condition, GEAR_REPORT] = 12
        condition = (abs(df[GEAR_RATIO] - 1.2920) < 0.01)
        df.loc[condition, GEAR_REPORT] = 11

        delay_time = 0.0
        sample_time = 0.05
        acc_data = df[TARGET_ACC]
        acc_data = acc_data.shift(int(-delay_time/sample_time))
        df = df.drop(TARGET_ACC, axis=1)
        df = pd.concat([df,acc_data],axis=1)
        df = df.dropna()
        
        if(len(df)==0):
            return df
        

        df = df[(df[GEAR_REPORT] == 12) | (df[GEAR_REPORT] == 11) |(df[GEAR_REPORT] == 0)]                

        df = df[(df[V_CURRENT_WHEEL] > 10) 
                & (df[THROTTLE_CMD] == 0) 
                & (df[BRAKE_INPUT] == 0)  
                & (df[BRAKE_CMD] == 0)
                & (df[RETARDER] == 0)
                & (df[ENGINE_BRAKE] == 0)
                & (abs(df[STEERING_ANGLE]) < 20 ) 
                ]
        
        df.to_csv(filtered_file_name)
        return df
    
    def assembly_data(self,bag_file_name):
        df = pd.DataFrame()
        data_dict = self._data_container.get_data_dict()

        min_length = 999999
        for data_name in TARGET_DATA_NAME_LIST:
            min_length = min(min_length, len(data_dict[data_name].data))
            
        for data_name in TARGET_DATA_NAME_LIST:   
            data = [x[1] for x in data_dict[data_name].data] 
            # print('type:{}, len:{}, data_name:{}'.format(type(data), len(data), data_name))
            df[data_name] = data[:min_length]
        
        df.to_csv(bag_file_name)

        return df
    
    def param_fitting(self, df, vehicle_name, date, target_acc, use_resistance_without_v = True):
        vehicle_mass_dict = {}
        vehicle_mass_dict['pdb-l4e-b0003'] = {
            '20230803':43000,
            '20230804':41100,
            '20230805':41100,
            '20230806':41100,
            '20230807':41100,
            '20230808':41100,
            '20230809':41100,
            '20230810':27700,
            '20230811':27700,
            '20230812':42800,
        }
        if(target_acc == COASTING_ACC):
            vehicle_mass_dict['pdb-l4e-b0003'] = {
                '20230803':21373,
                '20230804':35043,
                '20230805':35822,
                '20230806':35571,
                '20230807':35272,
                '20230808':36238,
                '20230809':35772,
                '20230810':21161,
                '20230811':21409,
                '20230812':36100,
            }     
        vehicle_mass = vehicle_mass_dict[vehicle_name][date]
        
        vehicle_frontal_area = 10.0
        air_density = 1.225
        gravity_acc = 9.81
        inertia_wheels = 60
        inertia_engine = 4.0
        transmission_efficiency = 0.95
        axle_drive_ratio = 2.714
        max_throttle_engine_torque = 3125
        transmission_efficiency = 0.95
        effective_tire_radius = 0.51
        ca = 0.68 * 0.5 * air_density * vehicle_frontal_area
        
        
        overall_ratio = df[GEAR_RATIO] * axle_drive_ratio
        rotation_equivalent_mass = (inertia_wheels + inertia_engine * transmission_efficiency * overall_ratio * overall_ratio) / (effective_tire_radius**2)

        F_drive = (df[THROTTLE_CMD] - df[ENGINE_FRICTION_TORQUE]) * max_throttle_engine_torque * overall_ratio * transmission_efficiency / effective_tire_radius

        print('vehicle name:{}, len:{}, date:{}, mass:{}'.format(vehicle_name, len(df), date, vehicle_mass))
        pitch_data = df[PITCH_CONTROL] * math.pi / 180
        y = F_drive + vehicle_mass * gravity_acc * np.sin(pitch_data) - (vehicle_mass + rotation_equivalent_mass) * df[target_acc] -  ca * df[V_CURRENT_WHEEL]*df[V_CURRENT_WHEEL]
        y = np.array(y)
        y = y.reshape(-1,1)

        
        if(use_resistance_without_v):
            A = np.zeros((y.shape[0],1))
            A[:,0] = vehicle_mass * gravity_acc * np.cos(pitch_data)
        else:
            A = np.zeros((y.shape[0],3))
            A[:,0] = df[V_CURRENT_WHEEL]*df[V_CURRENT_WHEEL]
            A[:,1] = vehicle_mass * gravity_acc * np.cos(pitch_data)
            A[:,2] = vehicle_mass * gravity_acc * np.cos(pitch_data) * df[V_CURRENT_WHEEL]

        params, residuals, rank, singular_values = np.linalg.lstsq(A, y, rcond=-1)
        coefficients = params.flatten()

        if(use_resistance_without_v):
            cr = coefficients[0] 
            df['a_estimated'] = (F_drive + vehicle_mass * gravity_acc * (np.sin(pitch_data) - cr * np.cos(pitch_data)) - ca * df[V_CURRENT_WHEEL]*df[V_CURRENT_WHEEL])/(vehicle_mass + rotation_equivalent_mass)
            
        else:
            ca = coefficients[0]
            cr = coefficients[1] 
            f0 = coefficients[2]
            df['a_estimated'] = (F_drive + vehicle_mass * gravity_acc * (np.sin(pitch_data) - (cr + f0 * df[V_CURRENT_WHEEL]) * np.cos(pitch_data)) - ca * df[V_CURRENT_WHEEL]*df[V_CURRENT_WHEEL])/(vehicle_mass + rotation_equivalent_mass)
        
        err = ((df[ACC_REPORT] - df[COASTING_ACC]) * (vehicle_mass + rotation_equivalent_mass) / (np.cos(pitch_data) * gravity_acc * vehicle_mass)).mean()
        # err = (df[ACC_REPORT] - df[COASTING_ACC]).mean()
        print('{}: ca = {}, cr = {}'.format(target_acc, ca, cr))
        return df, ca, cr, err
    
    def run(self):
        dir_name_list = [item for argument in self._args.bagdir for item in argument.split(',') if item != '']
        baglist = BagFinder().find_bags_in_(dir_name_list)
        print("baglist:{}".format(baglist))

        rosnode_process = []
        topic_process = subprocess.Popen(['rostopic', 'list'],
                                    stdout=PIPE,
                                    stderr=STDOUT)
        result, err = topic_process.communicate()
        if result == 'ERROR: Unable to communicate with master!\n':
            print("launch roscore")
            ros_process = subprocess.Popen('roscore')
            rosnode_process.append(ros_process)
            time.sleep(3)
        # set use_sim_time true
        subprocess.Popen(['rosparam', 'set', 'use_sim_time', 'true'])
        rospy.init_node('bag_report_generator', anonymous=True)

        self._output_dir = self._args.output_dir
        if not os.path.exists(self._output_dir):
            os.makedirs(self._output_dir) 
        result = pd.DataFrame()
        df = pd.DataFrame()

        for count, bag_file in enumerate(baglist):
            
            if ".bag" in bag_file:
                bag_file_name = os.path.basename(bag_file).split('.bag')[0]
            elif ".db" in bag_file:
                bag_file_name = os.path.basename(bag_file).split('.db')[0]
            else:
                print("incorrect bag {} to read, because it should be .db or .bag ended".format(bag_file))
                continue
            print('*******analyze {}*******'.format(bag_file_name))
            re = bag_file_name.split('_')
            vehicle_name = re[1]
            date = re[0].split('T')[0].split('/')[-1]  

            target_file_name = os.path.join(self._output_dir,'{}_data_original.csv'.format(bag_file_name))
            filtered_file_name = os.path.join(self._output_dir,'{}_data_filtered.csv'.format(bag_file_name))
            if(not os.path.exists(target_file_name)):
                if ".bag" in bag_file:
                    bag = rosbag.Bag(bag_file)
                elif ".db" in bag_file:
                    bag = fastbag.Reader(bag_file)
                    bag.open()
                for topic, msg, bag_time in bag.read_messages(topics=self._data_container.get_topic_list()):
                    self._data_container._topic_handlers[topic](msg, bag_time)
                new_df = self.assembly_data(target_file_name)
                self._data_container.reset_data_dict()
            else:
                new_df = pd.read_csv(target_file_name)
            plt.figure(count,figsize=(10,6))
            data_length = len(new_df)
            df = self.filter_data(new_df, filtered_file_name)
            plt.plot(df[COASTING_ACC],'r',label='coasting_acc')
            plt.plot(df[TARGET_ACC],'b',label='acc report')
            # plt.plot(df[A_CHASSIS],'b',label='acc chassis')
            # plt.plot(df[ACC_REPORT],'c',label='acc report')
            # plt.plot(df[A_IMU],'y',label='acc imu')
            # plt.plot(df[ACC_FROM_IMU_FILTERED],'m',label='acc filtered')
            re0 = self.param_fitting(df, vehicle_name,date,COASTING_ACC)
            coasting_acc_estimated = re0[0]['a_estimated']
            plt.plot(coasting_acc_estimated,'k',label='coasting_acc estimated')
            re1 = self.param_fitting(df, vehicle_name,date,TARGET_ACC)
            acc_estimated = re1[0]['a_estimated']
            plt.plot(acc_estimated,'g',label='acc report estimated')
            result = result.append(pd.DataFrame([{'bag_name':bag_file_name,'err':re1[3], 'data_length':data_length,'real_ca':re1[1],'real_cr':re1[2],'nominal_ca':re0[1],'nominal_cr':re0[2]}]),ignore_index=True)         
            plt.title(bag_file_name)
            plt.legend()
            plt.grid()
            plt.savefig(os.path.join(self._output_dir,'{}.png'.format(bag_file_name)))

        # df = self.filter_data(df, filtered_file_name)
        # self.param_fitting(df, vehicle_name,date,TARGET_ACC)
        result.to_csv(os.path.join(self._output_dir,'result.csv'))
            
        
        
def main(args):
    param_estimator = ParamEstimator(args)
    param_estimator.run()
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Param Estimator')
    parser.add_argument('bagdir', nargs='+', type=str, help="path to bag or db (can be more than one, separated by commas/spaces)")
    parser.add_argument('--output-dir', default=os.path.abspath(os.path.join(os.getcwd(),'..','report/default_report')))
    parser.add_argument('--time', default='0', type=str)
    args = parser.parse_args()
    main(args)