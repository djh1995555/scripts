#!/usr/bin/env python
import argparse
import json
import os
from collections import OrderedDict 
import pandas as pd
import yaml
from utils.report_plotter import ReportPlotter
import numpy as np
import math
from statistics import mode


class ReportGenerator:
    def __init__(self,args):
        self._args = args
        self._report_plotter = ReportPlotter('ReportGenerator')
        self._exclude_item = ["time", "timestamp", "time_stamp_loc", "delta_t"]
        self._figure_height = 600
        self._subplot_figure = None
        self._figure_list = []
        output_dir = os.path.dirname(self._args.file_path)
        self._output_filename = os.path.abspath(os.path.join(output_dir,"report.html"))
        self._data_selected = OrderedDict()
    
    def run(self):
        load_success = self.read_data(self._args.file_path)
        if(not load_success):
            return False
        self.generate_report(self._output_filename)
        print(f"generate report of {self._args.file_path} successfully!")

    def get_acc_response(self):
        status = []
        if "apa_statemachine" in self._all_data.keys():
            if "status" in self._all_data["apa_statemachine"].keys():
                status = self._all_data['apa_statemachine']['status']

        imu_acc = []
        if "control_debug" in self._all_data.keys():
            if "imu_acc_" in self._all_data["control_debug"].keys():
                imu_acc = self._all_data['control_debug']['imu_acc_']

        segment_stage = []
        if "control_debug" in self._all_data.keys():
            if "segment_stage" in self._all_data["control_debug"].keys():
                segment_stage = self._all_data['control_debug']['segment_stage']

        target_acc_mpss = []
        if "control_cmd" in self._all_data.keys():
            if "target_acc_mpss" in self._all_data["control_cmd"].keys():
                target_acc_mpss = self._all_data['control_cmd']['target_acc_mpss']  

        if(len(imu_acc)>0 and len(segment_stage)>0 and len(target_acc_mpss)>0):
            segments = []
            start = None
            
            for i in range(len(segment_stage)):
                if segment_stage[i] == 8:
                    if start is None:
                        start = i
                elif start is not None:
                    if i - start > 2:
                        segments.append((start, i))
                    start = None
            
            re = []
            
            # 对每个片段求出Blist中的众数和Clist中的峰值
            for segment in segments:
                start, end = segment
                target_stop_acc_segment = target_acc_mpss[start:end]
                acc_responce_segment = imu_acc[start:end]
                re.append((mode(target_stop_acc_segment),max(acc_responce_segment)))
            
            print(re)


    def read_data(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            self._all_data = json.load(file)

        if(self._all_data is None):
            print(f"load data from {file_path} failed!")
            return False
        
        imu_acc = []
        if "control_debug" in self._all_data.keys():
            if "imu_acc_" in self._all_data["control_debug"].keys():
                imu_acc = self._all_data['control_debug']['imu_acc_']
            if "acc_cmd_closeloop" in self._all_data["control_debug"].keys():
                acc_cmd_closeloop = self._all_data['control_debug']['acc_cmd_closeloop']
            if "previous_acceleration_reference" in self._all_data["control_debug"].keys():
                feedforward_acc = self._all_data['control_debug']['previous_acceleration_reference']
            if "slope_acc" in self._all_data["control_debug"].keys():
                feedback_acc = self._all_data['control_debug']['slope_acc']
            if "planned_speed" in self._all_data["control_debug"].keys():
                planned_speed = self._all_data['control_debug']['planned_speed']

        gear_data = []
        if "chassis" in self._all_data.keys():
            if "gear_position" in self._all_data["chassis"].keys():
                gear_data = self._all_data['chassis']['gear_position'] 

        if(len(imu_acc)>0 and len(gear_data)>0):
            min_length = min(len(imu_acc),len(gear_data))
            for i in range(min_length):
                if(gear_data[i] == 1):
                    imu_acc[i] *= -1
                    acc_cmd_closeloop[i] *= -1
                    feedforward_acc[i] *= -1
                    feedback_acc[i] *= -1
                    planned_speed[i] *= -1
            self._all_data['control_debug']['imu_acc_'] = imu_acc
            self._all_data['control_debug']['acc_cmd_closeloop'] = acc_cmd_closeloop
            self._all_data['control_debug']['previous_acceleration_reference'] = feedforward_acc
            self._all_data['control_debug']['slope_acc'] = feedback_acc
            self._all_data['control_debug']['planned_speed'] = planned_speed

        self.select_target_signal()
        
        # self.get_acc_response()

        return True
    
    def select_target_signal(self):
        with open(self._args.target_signal_filepath, 'r') as f:
            target_signals = yaml.load(f, Loader=yaml.Loader)
        
        target_panel = dict(target_signals["target_panel"])
        for sub_panel_name, sub_panel in target_panel.items():
            sub_dict = OrderedDict()
            for signal_sidplay_name, signal_full_name in sub_panel.items():
                channel, signal = signal_full_name.split(":")
                if channel in self._all_data.keys():
                    data = self._all_data[channel][signal]
                    time = self._all_data[channel]["timestamp"]
                    sub_dict[signal_sidplay_name] = (data, time)
            self._data_selected[sub_panel_name] = sub_dict

    def generate_report(self, output_filename):
        start_timestamp = math.inf
        for catagory, catagory_data in self._all_data.items():
            data_time = catagory_data["timestamp"]
            if(data_time[0]<start_timestamp):
                start_timestamp = data_time[0]

        for sub_panel_name, sub_dict in self._data_selected.items():
            time_list=[]
            value_list=[]
            legend_list = []
            
            for signal_full_name, (data, data_time) in sub_dict.items():
                if((data_time[0]-start_timestamp)/1000000000 > 10000):
                    time_list.append(np.array([(x - data_time[0])/1000000000 for x in data_time]))
                else:
                    time_list.append(np.array([(x - start_timestamp)/1000000000 for x in data_time]))
                df = pd.DataFrame({'data': data})
                df_interp = df.interpolate(method='linear')
                data_array = df_interp.T.to_numpy()
                value_list.append(data_array[0])
                legend_list.append(signal_full_name)
            
            subplot = self._report_plotter.plot_figure_plotly(x_list = time_list, 
                                                y_list = value_list,
                                                legend_list = legend_list,
                                                x_label = 'time / s',
                                                y_label = '',
                                                title = sub_panel_name,
                                                legend_prefix = '',
                                                figure_height=self._figure_height,)
            self._figure_list.append(subplot)  



        for catagory, catagory_data in self._all_data.items():
            time_list=[]
            value_list=[]
            legend_list = []
            data_time = catagory_data["timestamp"]
            
            for data_name, data in catagory_data.items():
                if(data[0] is None):
                    continue
                if(data_name not in self._exclude_item):
                    if((data_time[0]-start_timestamp)/10e8 > 10000):
                        time_list.append(np.array([(x - data_time[0])/10e8 for x in data_time]))
                    else:
                        time_list.append(np.array([(x - start_timestamp)/10e8 for x in data_time]))
                    df = pd.DataFrame({'data': data})
                    df_interp = df.interpolate(method='linear')
                    data_array = df_interp.T.to_numpy()
                    value_list.append(data_array[0])
                    legend_list.append(f"{catagory}:{data_name}")

            subplot = self._report_plotter.plot_figure_plotly(x_list = time_list, 
                                                y_list = value_list,
                                                legend_list = legend_list,
                                                x_label = 'time / s',
                                                y_label = '',
                                                title = catagory,
                                                legend_prefix = '',
                                                figure_height=self._figure_height,)
            self._figure_list.append(subplot)

        self._subplot_figure_list = [(i + 1, 1, fig) for i, fig in enumerate(self._figure_list)]
        self._subplot_figure = self._report_plotter.append_figure_to_subplot_plotly(self._subplot_figure_list, len(self._figure_list), 1, template="plotly_dark", subplot_fig=self._subplot_figure)
        plot_html_str = ""
        plot_html_str += self._report_plotter.get_fuel_fig_html_str({"Comparison": self._subplot_figure})
        html_str = self._report_plotter.generate_html_fuel_report(plot_html_str)
        with open(output_filename, 'w') as f:
            f.write(html_str)

def main(args):
    reprt_generator = ReportGenerator(args)
    success = reprt_generator.run()
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Report Generator')
    scripts_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--file-path', default="/home/mi/debug/scripts/record/test_record/2024-04-21/MT091/11-16-27/record.json", type=str)
    parser.add_argument('--output-dir', default=os.path.join(scripts_dirname,'report/default_report'))
    parser.add_argument('--time', default='0000', type=str)
    parser.add_argument('--target-signal-filepath', default=os.path.join(scripts_dirname,'config/target_signal_lon.yaml'), type=str)
    args = parser.parse_args()
    
    main(args)