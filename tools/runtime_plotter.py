#!/usr/bin/env python
import os
import sys
import time
import rospy
import argparse
import subprocess
import numpy as np
from subprocess import PIPE, STDOUT
import matplotlib.pyplot as plt
from collections import OrderedDict
from utils.runtime_index_dict import *
from utils.data_container import DataContainer

_MAXLEN_RECORD = 30 # seconds
_PAGE_HEIGHT = 20
_LINE_WIDTH = 1
_MAX_STOP_COUNT = 5

process_list = []
class RuntimePlotter:
    def __init__(self,args):
        self._data_container = DataContainer(True)
        self._figures = OrderedDict()
        self._args = args

    def update_figures(self,sub_figures):
        data_dict = self._data_container.get_data_dict()
        for signal_name, signal_figure in sub_figures[0].items():
            self.update_data(signal_figure, data_dict[signal_name])

    def set_range(self,item):
        low_limit = 999
        high_limit = -999
        plot_dict = item[0]
        ax = item[1]
        for fig in plot_dict.items():
            data = fig[1].get_ydata()
            if(len(data)>0):
                low_limit = min(low_limit,min(data))
                high_limit = max(high_limit,max(data))
        margin = (high_limit - low_limit)*0.1
        low_limit -= margin 
        high_limit += margin
        ax.set_ylim([low_limit,high_limit])

    def update_data(self, signal_figure, original_data):
        if(len(original_data.data) == 0):
            return
        length = min(len(original_data.data), _MAXLEN_RECORD * original_data.freq)
        data = np.array([x[1] for x in original_data.data[-length:]])
        data_time = np.array([x[0] - original_data.data[-1][0] for x in original_data.data[-length:]])
        signal_figure.set_data(data_time, data)

    def run(self):
        plt.ion()
        x = np.zeros(1)
        y = np.zeros(1)

        page_width = _MAXLEN_RECORD
        page_height = _PAGE_HEIGHT
        plot_width = page_width/2
        plot_height = page_height/5
        fig1 = plt.figure(num=1,figsize=(page_height, page_width))
        i=0
        for subtitle, subdict in RUNTIME_INDEX_DICT.items():
            ax = plt.subplot2grid((page_height,page_width), (plot_height*(i%5), plot_width*(i/5)), rowspan=plot_height,colspan=plot_width)
            # print("title:%s"%subtitle)
            ax.set_title(subtitle)
            ax.set_xlim([-page_width, 0])
            ax.set_xticks(np.arange(-page_width, 0, 1))
            j = 0
            sub_figures = OrderedDict()
            for signal_name in subdict:
                # print("     signal_name:%s"%signal_name)
                signal_figure, = ax.plot(x, y, COLOR_LIST[j], linewidth = _LINE_WIDTH, label=signal_name)
                sub_figures[signal_name] = signal_figure
                j += 1  
            ax.legend(loc='upper left')
            ax.set_axisbelow(True)
            ax.yaxis.grid(color='gray', linestyle='dashed')
            ax.xaxis.grid(color='gray', linestyle='dashed')         
            i+=1 
            self._figures[subtitle] = ((sub_figures,ax))
            if(i == 10):
                break
    
        plt.tight_layout(pad=0.05, w_pad=0.001, h_pad=2.0)
        ui_freq = 20 #in hz
        
        # rosnode_process = []
        # topic_process = subprocess.Popen(['rostopic', 'list'],
        #                             stdout=PIPE,
        #                             stderr=STDOUT)
        # result, err = topic_process.communicate()
        # if result == 'ERROR: Unable to communicate with master!\n':
        #     ros_process = subprocess.Popen('roscore')
        #     rosnode_process.append(ros_process)
        #     time.sleep(3)
        # # set use_sim_time true
        # subprocess.Popen(['rosparam', 'set', 'use_sim_time', 'true'])
        
        if(self._args.enable_output):
            output_dir = os.path.join(self._args.output_dir, self._args.time)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            output_bag = os.path.join(output_dir, self._args.vehicle_name+'_'+self._args.time + '.bag')
            if not rospy.is_shutdown():
                record_process = subprocess.Popen(['rosbag', 'record', '-O', output_bag, '-a', "--lz4","__name:=my_bag"])
                process_list.append(record_process)
                time.sleep(1.0)

        stop_count = 0
        last_subscribe_count = 0
        while not rospy.is_shutdown():
            try:
                data_subscribe_count = self._data_container.get_subscribe_count()
                if(last_subscribe_count == data_subscribe_count):
                    stop_count += 1
                    # print("stop_count = %d"%stop_count)
                else:
                    stop_count = 0
                last_subscribe_count = data_subscribe_count
                
                if(stop_count > _MAX_STOP_COUNT and data_subscribe_count != 0):
                    time.sleep(2)
                    subprocess.Popen(['rosnode', 'kill', '-a'], stdout=PIPE, stderr=STDOUT)
                    time.sleep(2)
                    sys.exit()
                
                for subtitle, sub_figures in self._figures.items():
                    self.update_figures(sub_figures)
                    self.set_range(sub_figures)

                # draw the picture
                fig1.canvas.flush_events()
                
                now = time.time()
                elapsed = time.time() - now
                if elapsed < 1./ui_freq:
                    time.sleep(1./ui_freq - elapsed)
                # else:
                #     print ('warning, UI rendering slower than expected', elapsed)
                
            except KeyboardInterrupt:
                sys.exit()


def main(args):
    runtime_plotter = RuntimePlotter(args)
    runtime_plotter.run()

if __name__ == '__main__':
    rospy.init_node('runtime_plotter', anonymous=True)
    parser = argparse.ArgumentParser('Runtime Plotter')
    parser.add_argument('--output-dir', default=os.path.abspath(os.path.join(os.getcwd(),'..','report/default_report')))
    parser.add_argument('--vehicle-name', default='j7-l4e-sim', type=str)
    parser.add_argument('--time', default='0', type=str)
    parser.add_argument('--enable-output', default=False, type=bool)
    
    args = parser.parse_args()
    main(args)