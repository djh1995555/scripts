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
            if "planned_acc" in self._all_data["control_debug"].keys():
                planned_acc = self._all_data['control_debug']['planned_acc']
            if "acc_cmd_closeloop" in self._all_data["control_debug"].keys():
                acc_cmd_closeloop = self._all_data['control_debug']['acc_cmd_closeloop']
            if "previous_acceleration_reference" in self._all_data["control_debug"].keys():
                feedforward_acc = self._all_data['control_debug']['previous_acceleration_reference']
            if "slope_acc" in self._all_data["control_debug"].keys():
                feedback_acc = self._all_data['control_debug']['slope_acc']
            if "planned_speed" in self._all_data["control_debug"].keys():
                planned_speed = self._all_data['control_debug']['planned_speed']
            if "end_y_error" in self._all_data["control_debug"].keys():
                end_y_error = self._all_data['control_debug']['end_y_error']
            if "front_heading_error" in self._all_data["control_debug"].keys():
                distance_to_entrance_line = self._all_data['control_debug']['front_heading_error']
            if "vehiclestate_linear_velocity" in self._all_data["control_debug"].keys():
                vehicle_speed = self._all_data['control_debug']['vehiclestate_linear_velocity']
            if "vehiclestate_linear_velocity" in self._all_data["control_debug"].keys():
                vehicle_speed = self._all_data['control_debug']['vehiclestate_linear_velocity']


        gear_data = []
        if "chassis" in self._all_data.keys():
            # if "gear_position" in self._all_data["chassis"].keys():
            #     gear_data = self._all_data['chassis']['gear_position'] 
            # if "vehicle_speed" in self._all_data["chassis"].keys():
            #     chassis_speed = self._all_data['chassis']['vehicle_speed']
            if "target_gear" in self._all_data["control_cmd"].keys():
                target_gear = self._all_data['control_cmd']['target_gear']

        reverse_flag = False
        if ("chassis" in self._all_data.keys() and "control_debug" in self._all_data.keys()):
            if(len(imu_acc)>0 and len(target_gear)>0):
                min_length = min(len(imu_acc),len(target_gear))
                for i in range(min_length-1):
                    if(not reverse_flag and target_gear[i] == 1 and target_gear[i+1] == 0):
                        reverse_flag = True
                    if(reverse_flag and target_gear[i] == 3 and target_gear[i+1] == 0):
                        reverse_flag = False
                    # imu_acc[i] *= -1
                    if(reverse_flag):
                        imu_acc[i] *= -1
                        planned_acc[i] *= -1
                        acc_cmd_closeloop[i] *= -1
                        feedforward_acc[i] *= -1
                        feedback_acc[i] *= -1
                        # planned_speed[i] *= -1
                        # vehicle_speed[i] *= -1
                self._all_data['control_debug']['imu_acc_'] = imu_acc
                self._all_data['control_debug']['planned_acc'] = planned_acc
                self._all_data['control_debug']['acc_cmd_closeloop'] = acc_cmd_closeloop
                self._all_data['control_debug']['previous_acceleration_reference'] = feedforward_acc
                self._all_data['control_debug']['slope_acc'] = feedback_acc
                # self._all_data['control_debug']['planned_speed'] = planned_speed
                # self._all_data['control_debug']['vehiclestate_linear_velocity'] = vehicle_speed

        self._all_data['control_debug']['previous_acceleration_reference'] = [0.5 for x in self._all_data['control_debug']['previous_acceleration_reference']]
        self._all_data['control_debug']['slope_acc'] = [1.25 * x for x in self._all_data['control_debug']['planned_speed']]

        # self.compute_slope_position2()
        # self.compute_slope_position()

        self.select_target_signal()
        return True

    def compute_slope_position2(self):
        if "control_debug" in self._all_data.keys():
            if "stereo_slot_brake_position" in self._all_data["control_debug"].keys():
                rear_exit_slope_position = self._all_data['control_debug']['stereo_slot_brake_position']
            if "stereo_slot_increase_torque_position" in self._all_data["control_debug"].keys():
                rear_enter_slope_position = self._all_data['control_debug']['stereo_slot_increase_torque_position']
        else:
            return
        front_enter_slope_position = []
        front_exit_slope_position = []
        distance_to_front_exit_slope_position = []
        distance_to_entrance_line = self._all_data['control_debug']['front_heading_error']
        wheel_base = 3.00
        front_wheel_finish_climbing_offset = 0.0
        if(len(rear_exit_slope_position)>0 and len(rear_enter_slope_position)>0):
            for i in range(len(rear_enter_slope_position)):
                front_enter_slope_position.append(rear_enter_slope_position[i] - wheel_base + 0.3)
                front_exit_slope_position.append(rear_exit_slope_position[i] - wheel_base + front_wheel_finish_climbing_offset)
                distance_to_front_exit_slope_position.append(distance_to_entrance_line[i] - (rear_exit_slope_position[i] - wheel_base + front_wheel_finish_climbing_offset))

            self._all_data['control_debug']['front_enter_slope_position_'] = front_enter_slope_position
            self._all_data['control_debug']['front_exit_slope_position_'] = front_exit_slope_position
            self._all_data['control_debug']['rear_enter_slope_position_'] = rear_enter_slope_position
            self._all_data['control_debug']['rear_exit_slope_position_'] = rear_exit_slope_position
            self._all_data['control_debug']['distance_to_front_exit_slope_position'] = distance_to_front_exit_slope_position


    def compute_slope_position(self):
        if "control_debug" in self._all_data.keys():
            if "stereo_slot_stop_distance" in self._all_data["control_debug"].keys():
                stop_distance = self._all_data['control_debug']['stereo_slot_stop_distance']
            if "stereo_slot_brake_position" in self._all_data["control_debug"].keys():
                brake_position = self._all_data['control_debug']['stereo_slot_brake_position']
            if "stereo_slot_increase_torque_position" in self._all_data["control_debug"].keys():
                increase_torque_position = self._all_data['control_debug']['stereo_slot_increase_torque_position']
        else:
            return

        front_enter_slope_position = []
        front_exit_slope_position = []
        rear_enter_slope_position = []
        rear_exit_slope_position = []
        wheel_base = 3.00
        if(len(stop_distance)>0 and len(brake_position)>0 and len(increase_torque_position)>0):
            for i in range(len(stop_distance)):
                front_enter_slope_position.append(increase_torque_position[i] - 0.5)
                front_exit_slope_position.append(brake_position[i] + stop_distance[i])
                rear_enter_slope_position.append(increase_torque_position[i] - 0.5 + wheel_base)
                rear_exit_slope_position.append(brake_position[i] + stop_distance[i] + wheel_base - 0.1)

            self._all_data['control_debug']['front_enter_slope_position_'] = front_enter_slope_position
            self._all_data['control_debug']['front_exit_slope_position_'] = front_exit_slope_position
            self._all_data['control_debug']['rear_enter_slope_position_'] = rear_enter_slope_position
            self._all_data['control_debug']['rear_exit_slope_position_'] = rear_exit_slope_position

    def count_gear_switching_time(self,file_path):
        apa_status = self._all_data['apa_statemachine']['status']
        vehicle_speed = self._all_data['chassis']['vehicle_speed']
        gear_data = self._all_data['chassis']['gear_position'] 
        distance_to_entrance_line = self._all_data['control_debug']['front_heading_error']
        lengths = []
        count = 0
        total_count = 0
        for i in range(len(vehicle_speed)):
            if((gear_data[i] == 1 or gear_data[i] ==3) and apa_status[i] == 5):
                if vehicle_speed[i] == 0:
                    count += 1
                else:
                    if count > 0:
                        lengths.append(count * 0.02)
                        count = 0
                total_count += 1
        if count > 0:
            lengths.append(count * 0.02)
        
        gear_seitching_time = sum(lengths[1:-1])
        total_time = total_count * 0.02
        ratio = gear_seitching_time / total_time
        output_name = 'gear_switching_time.csv'
        data = {
            'file_path': file_path,
            'length': str(lengths),
            'gear_seitching_time': f'{gear_seitching_time:2f}',
            'total_time': f'{total_time:2f}',
            'ratio': f'{ratio:2f}',
        }

        df = pd.DataFrame(data, index=[0,])

        # 追加写入CSV文件
        df.to_csv(output_name, mode='a', header=False, index=False, encoding='utf-8')
        print(lengths)
        print(sum(lengths[1:-1]))
        print(ratio)
                
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

        def are_all_none(lst):
            return all(element is None for element in lst)

        for sub_panel_name, sub_dict in self._data_selected.items():
            time_list=[]
            value_list=[]
            legend_list = []
            
            for signal_full_name, (data, data_time) in sub_dict.items():
                if(are_all_none(data)):
                    continue
                if((data_time[0]-start_timestamp)/1000000000 > 10000):
                    time_list.append(np.array([(x - data_time[0])/1000000000 for x in data_time]))
                else:
                    time_list.append(np.array([(x - start_timestamp)/1000000000 for x in data_time]))
                df = pd.DataFrame({'data': data})
                # if(signal_full_name == "sliping_flag"):
                #     print(data)
                # print(signal_full_name)
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
        print(f"output_filename:{output_filename}")
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