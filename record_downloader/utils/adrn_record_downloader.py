#!/usr/bin/env python
import datetime
import os
from utils.path_record_downloader import PathRecordDownloader
from utils.record_generator import RecordGenerator

class AdrnRecordDownloader():
    def __init__(self, common_config, filesystem_kws, downloader_config = None):
        self._downloader_config = downloader_config
        self._filesystem_kws =  filesystem_kws
        if(len(common_config["output_dir"]) == 0):
            current_directory = os.getcwd()
            self._output_dir = os.path.join(current_directory,"record")
        else:
            self._output_dir = common_config["output_dir"]

        self._watch_channel = common_config["watch_channel"]

        self._record_generator = RecordGenerator()

        self._path_record_downloader = PathRecordDownloader(common_config, filesystem_kws)

    
    def download(self):
        self.download_record(self._downloader_config["adrn"])
    
    def download_record(self, adrn, output_name = None):
        if(len(adrn) == 0):
            return False
        if(self._downloader_config is not None and self._downloader_config['download_method'] == "data_flow"):
            self.download_record_by_data_flow(adrn, output_name)
        else:
            from ad_cloud.adrn.data_seeker.records import DataclipReader
            reader = DataclipReader(adrn, filesystem_kws=self._filesystem_kws)
            target_filepath = []
            for file in reader.record_files:
                if("full_record." in file.path):
                    print(file.path)
                    target_filepath.append(file.path)
            
            for filepath in target_filepath:
                self.download_record_by_path(filepath, output_name)
            return True

    def download_record_by_path(self, filepath, output_name = None):
        self._path_record_downloader.download_record(filepath, output_name)

    def download_record_by_data_flow(self, adrn:str, output_name = None):
        from ad_cloud.adrn.data_seeker.records import DataclipReader
        if(output_name is None):
            re = adrn.split("::")
            time_range = re[-1]
            vehicle = re[-2]

            start_timestamp = int(time_range.split(":")[0]) / 10e8
            date = datetime.datetime.fromtimestamp(start_timestamp).strftime('%Y-%m-%d %H-%M-%S')
            date, time = date.split(" ")

            vehicle_name = vehicle.split(".")[-1]
            output_dir = os.path.join(self._output_dir , date, vehicle_name, time)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            output_name = os.path.join(output_dir , f"{vehicle_name}_{date}-{time}.record")

        reader = DataclipReader(adrn, filesystem_kws=self._filesystem_kws)

        self._record_generator.write_record(reader, self._watch_channel, output_name, adrn)


