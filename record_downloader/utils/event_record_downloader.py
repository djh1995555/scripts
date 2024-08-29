#!/usr/bin/env python
import os
import datetime
from utils.adrn_record_downloader import AdrnRecordDownloader
from utils.color_string import *
from rich.console import Console
console = Console()
print = console.print

class EventRecordDownloader():
    def __init__(self, common_config, filesystem_kws, downloader_config = None):
        self._downloader_config = downloader_config
        if(len(common_config["output_dir"]) == 0):
            current_directory = os.getcwd()
            self._output_dir = os.path.join(current_directory,"record")
        else:
            self._output_dir = common_config["output_dir"]

        self._adrn_record_downloader = AdrnRecordDownloader(common_config, filesystem_kws)

    def download_record(self, event_id:str,  output_dir = None):
        from ad_cloud.event import seeker
        event_seeker=seeker.EventSearcher(event_id,event_type=seeker.EVENT_SOURCE_ENUM.PROD)
        detail = event_seeker.query_detail()
        start_timestamp = int(detail['start_timestamp']) / 10e8
        date = datetime.datetime.fromtimestamp(start_timestamp).strftime('%Y-%m-%d %H-%M-%S')
        date, time = date.split(" ")
        vehicle_name = detail["car_no"]
        if(output_dir is None):
            output_dir = os.path.join(self._output_dir, date, vehicle_name, time)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        detail_filepath = os.path.join(output_dir, f"{vehicle_name}_{date}-{time}_{event_id}_detail.log")
        with open(detail_filepath, "w") as query_detail:
            for name, data in vars(detail).items():
                if(name == "start_timestamp" or name == "end_timestamp"):
                    data = datetime.datetime.fromtimestamp(int(data)/ 10e8).strftime('%Y-%m-%d %H-%M-%S')
                query_detail.write(f"{name} = {data}\n")

        
        output_filename = os.path.join(output_dir, f"{vehicle_name}_{date}-{time}_{event_id}.record")
        if(not os.path.exists(output_filename) or os.path.getsize(output_filename) < 100):
            try:
                if(len(detail["data_clip"]) == 0):
                    print(f"Downloading Event[{event_id}] failed because data clip is missing!")
                    return 
                print(f"Downloading Event[{event_id}] to {color_path(output_filename)}")
                event_seeker.download_and_save(output_filename)
                if(os.path.getsize(output_filename) < 100):
                    print("Download record by event id failed! Retry downloading by event id again...")
                    event_seeker.download_and_save(output_filename)
                if(os.path.getsize(output_filename) < 100):
                    print("Download record by event id failed! Retry downloading by adrn...")
                    self._adrn_record_downloader.download_record(detail['data_clip'])
                   
            except:
                print(f"Downloading Event[{event_id}] failed!")
                return
        else:
            print(f"{color_path(output_filename)} has existed!")

        print("*************************************************************************")


    
    def download(self):
        event_ids = self._downloader_config["event_ids"]
        for event_id in event_ids:
            self.download_record(event_id)