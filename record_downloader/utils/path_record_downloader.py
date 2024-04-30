#!/usr/bin/env python
import os
from utils.record_generator import RecordGenerator

class PathRecordDownloader():
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

    def download_record(self, filepath:str, output_name = None):
        from ad_cloud.adrn.data_seeker.records import RecordReader
        if(output_name is None):
            re = filepath.split("/")
            vehicle_name = re[-4]
            date = re[-3]
            time = re[-2]
            output_dir = os.path.join(self._output_dir , date, vehicle_name, time)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            output_name = os.path.join(output_dir , re[-1])
        
        reader = RecordReader(filepath, filesystem_kws=self._filesystem_kws)
        self._record_generator.write_record(reader, self._watch_channel, output_name, filepath)

    def download(self):
        self.download_record(self._downloader_config["path"])