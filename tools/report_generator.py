#!/usr/bin/env python
import os
import sys
import time
import argparse
import logging
import subprocess
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

class ReportGenerator:
    def __init__(self,args):
        self._args = args
        self._data_container = DataContainer(False)
        self._report_plotter = ReportPlotter('ReportGenerator')
        self._figure_height = 600

    def time_delay_estimation(self, target_data_name1,target_data_name2):
        data_dict = self._data_container.get_data_dict()

        x = data_dict[target_data_name1]
        x = np.array([x[1] for x in x.data])
        y = data_dict[target_data_name2]
        y = np.array([y[1] for y in y.data])
        print('type:{}'.format(type(y)))
        n = x.shape[0]
        corr = signal.correlate(y, x, mode='same') / np.sqrt(signal.correlate(x, x, mode='same')[int(n/2)] * signal.correlate(y, y, mode='same')[int(n/2)])
        delay_arr = np.linspace(-0.5*n, 0.5*n, n)
        delay = delay_arr[np.argmax(corr)]
        print('delay:{}'.format(delay))
        return delay

    def generate_histogram(self,target_data_names):
        for target_data_name in target_data_names:
            data_dict = self._data_container.get_data_dict()
            plot_data = data_dict[target_data_name]
            data = np.array([x[1] for x in plot_data.data])
            hist, bins = np.histogram(data, bins=50)
            fig = plt.figure(figsize=(8, 6))
            ax = fig.subplots()
            ax.bar(bins[:-1], hist, width=(bins[1]-bins[0]), align='edge')

            ax.set_title('Distribution of Random Data')
            ax.set_xlabel('Value')
            ax.set_ylabel('Frequency')
            output_filename = os.path.join(self._output_dir,'{}_histogram.png'.format(target_data_name))
            plt.savefig(output_filename, dpi=100)
    
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
                   
        for count, bag_file in enumerate(baglist):
            if ".bag" in bag_file:
                bag_file_name = os.path.basename(bag_file).split('.bag')[0]
                bag = rosbag.Bag(bag_file)
            elif ".db" in bag_file:
                bag_file_name = os.path.basename(bag_file).split('.db')[0]
                bag = fastbag.Reader(bag_file)
                bag.open()
            else:
                print("incorrect bag {} to read, because it should be .db or .bag ended".format(bag_file))
                continue
            for topic, msg, bag_time in bag.read_messages(topics=self._data_container.get_topic_list()):
                self._data_container._topic_handlers[topic](msg, bag_time)
                
            self.generate_histogram([V_ESTIMATED])
            # self.time_delay_estimation(THROTTLE_CMD, ACC_REPORT)
            self.generate_report(bag_file_name)
            self._data_container.reset_data_dict()
            
        
        
def main(args):
    reprt_generator = ReportGenerator(args)
    reprt_generator.run()
    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser('Bag Report Generator')
    parser.add_argument('bagdir', nargs='+', type=str, help="path to bag or db (can be more than one, separated by commas/spaces)")
    parser.add_argument('--output-dir', default=os.path.abspath(os.path.join(os.getcwd(),'..','report/default_report')))
    parser.add_argument('--time', default='0', type=str)
    args = parser.parse_args()
    
    # log_dir = os.path.join(args.output_dir, args.time)
    # set_logger(logger, log_dir, "bag_report_generator.log", os.environ.get("LOGLEVEL", "DEBUG"), os.environ.get("LOGLEVEL", "DEBUG"))
    
    main(args)