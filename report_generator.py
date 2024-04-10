#!/usr/bin/env python
import argparse
import json
import os
from collections import OrderedDict 
import yaml
from report_plotter import ReportPlotter
import numpy as np
import math


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
        self.read_data(self._args.file_path)
        self.generate_report(self._output_filename)

    def read_data(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            self._all_data = json.load(file)

        self.select_target_signal()
    
    def select_target_signal(self):
        with open(self._args.target_signal_filepath, 'r') as f:
            target_signals = yaml.load(f)
        
        target_panel = dict(target_signals["target_panel"])
        for sub_panel_name, sub_panel in target_panel.items():
            sub_dict = OrderedDict()
            for signal_full_name in sub_panel:
                channel, signal = signal_full_name.split(":")
                if channel in self._all_data.keys():
                    data = self._all_data[channel][signal]
                    time = self._all_data[channel]["timestamp"]
                    sub_dict[signal_full_name] = (data, time)
            self._data_selected[sub_panel_name] = sub_dict


    def generate_report(self, output_filename):
        start_timestamp = math.inf
        for catagory, catagory_data in self._all_data.items():
            data_time = catagory_data["timestamp"]
            if(data_time[0]<start_timestamp):
                start_timestamp = data_time[0]

        print(f'start_time: {start_timestamp}')
        for sub_panel_name, sub_dict in self._data_selected.items():
            time_list=[]
            value_list=[]
            legend_list = []
            
            for signal_full_name, (data, data_time) in sub_dict.items():
                if((data_time[0]-start_timestamp)/1000000000 > 10000):
                    time_list.append(np.array([(x - data_time[0])/1000000000 for x in data_time]))
                else:
                    time_list.append(np.array([(x - start_timestamp)/1000000000 for x in data_time]))
                value_list.append(np.array(data))
                legend_list.append(signal_full_name)
                print(f"type1:{type(time_list[0])}")
            
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
                if(data_name not in self._exclude_item):
                    if((data_time[0]-start_timestamp)/1000000000 > 10000):
                        time_list.append(np.array([(x - data_time[0])/1000000000 for x in data_time]))
                    else:
                        time_list.append(np.array([(x - start_timestamp)/1000000000 for x in data_time]))
                    value_list.append(np.array(data))
                    legend_list.append(f"{catagory}:{data_name}")
                    print(f"type2:{type(time_list[0])}")
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
    reprt_generator.run()

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Report Generator')
    scripts_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--file-path', default="/home/mi/debug/scripts/record/replay_record/20240410171449/0408/88048/record.json", type=str)
    parser.add_argument('--output-dir', default=os.path.join(scripts_dirname,'report/default_report'))
    parser.add_argument('--time', default='0000', type=str)
    parser.add_argument('--target-signal-filepath', default=os.path.join(scripts_dirname,'target_signal.yaml'), type=str)
    args = parser.parse_args()
    
    main(args)