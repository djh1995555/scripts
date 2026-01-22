#!/usr/bin/env python
import os
import pandas as pd
import datetime
import wget
from utils.adrn_record_downloader import AdrnRecordDownloader
from utils.color_string import *
from rich.console import Console
console = Console()
print = console.print

class JinShanYunRecordDownloader():
    def __init__(self, common_config, filesystem_kws, downloader_config = None):
        self._downloader_config = downloader_config
        if(len(common_config["output_dir"]) == 0):
            current_directory = os.getcwd()
            self._output_dir = os.path.join(current_directory,"record")
        else:
            self._output_dir = common_config["output_dir"]
        self._jinshanyun_url = self._downloader_config['jinshanyun_url']
        self._vehicle_type = self._downloader_config['vehicle_type']

    def download_record(self, target_url, output_dir = None):
        full_record_name = target_url.split("/")[-1]
        output_path = f"{output_dir}/{full_record_name}"
        if(not os.path.exists(output_path)):
            print(f"    Downloading {full_record_name}...")
            wget.download(target_url, out=output_path)
        else:
            print(f"    {full_record_name} has existed!")

    
    def download(self):
        target_times_filepath = self._downloader_config["target_times"]
        all_full_records_filepath = self._downloader_config["all_full_records"]
        target_times = pd.read_csv(target_times_filepath)
        dtype_dict = {
                'vehicle_id': str
        }
        all_full_records = pd.read_csv(all_full_records_filepath,dtype = dtype_dict)

        self._date = all_full_records.iloc[0, all_full_records.columns.get_loc('date')]
        fast_idx = 0
        slow_idx = 0
        while slow_idx < target_times.shape[0] and fast_idx < all_full_records.shape[0]:
            start_time = self._date + '-' + target_times.iloc[slow_idx, target_times.columns.get_loc('数据开始时间')]
            start_time = datetime.datetime.strptime(start_time, '%Y-%m-%d-%H:%M:%S')
            end_time = self._date + '-' + target_times.iloc[slow_idx, target_times.columns.get_loc('数据结束时间')]
            end_time = datetime.datetime.strptime(end_time, '%Y-%m-%d-%H:%M:%S')

            print(f'Start_time: {start_time}    End_time: {end_time}')
            issue = target_times.iloc[slow_idx, target_times.columns.get_loc('问题')]
            while fast_idx < all_full_records.shape[0]:
                full_record_name = all_full_records.iloc[fast_idx, all_full_records.columns.get_loc('full_record_name')]
                full_record_time = datetime.datetime.strptime(full_record_name.split('.')[1], '%Y-%m-%d-%H-%M-%S')
                if(full_record_time > start_time):
                    # full_records_idx_list.append(fast_idx - 1)
                    target_idx = fast_idx - 1
                    vehicle_id = all_full_records.iloc[target_idx, all_full_records.columns.get_loc('vehicle_id')]
                    date = all_full_records.iloc[target_idx, all_full_records.columns.get_loc('date')]
                    time = all_full_records.iloc[target_idx, all_full_records.columns.get_loc('time')]
                    full_record_name = all_full_records.iloc[target_idx, all_full_records.columns.get_loc('full_record_name')]
                    target_url = f'{self._jinshanyun_url}/{self._vehicle_type}/{vehicle_id}/{date}/{time}/{full_record_name}'
                    dir_name = self._date + '-' + target_times.iloc[slow_idx, target_times.columns.get_loc('数据开始时间')] + issue
                    output_dir = os.path.join(self._output_dir, str(vehicle_id), dir_name)
                    os.makedirs(output_dir, exist_ok=True)
                    self.download_record(target_url, output_dir)
                    if(full_record_time > end_time):
                        break
                fast_idx += 1
            slow_idx += 1
            
            
