#!/usr/bin/env python

import argparse
import os
import yaml
from utils.event_record_downloader import EventRecordDownloader
from utils.path_record_downloader import PathRecordDownloader
from utils.adrn_record_downloader import AdrnRecordDownloader
from utils.time_range_record_downloader import TimeRangeRecordDownloader
from utils.jira_record_downloader import JiraRecordDownloader

class RecordDownloader():
    def __init__(self, args):
        self._args = args
        with open(self._args.config, 'r') as f:
            config = yaml.load(f, Loader=yaml.Loader)

        env_kws, filesystem_kws = self.get_env_and_filesystem_kws(config)
        for env_name, env_value in env_kws.items():
            os.environ[env_name] = env_value
            
        if(config["query_config"]['query_method'] == "path"):
            self._record_downloader = PathRecordDownloader(config["query_config"]["common_config"], 
                                                           filesystem_kws,
                                                           config["query_config"]["path_record_downloader_config"])
        elif(config["query_config"]['query_method'] == "event_id"):
            self._record_downloader = EventRecordDownloader(config["query_config"]["common_config"], 
                                                            filesystem_kws,
                                                            config["query_config"]["event_record_downloader_config"])
        elif(config["query_config"]['query_method'] == "adrn"):
            self._record_downloader = AdrnRecordDownloader(config["query_config"]["common_config"], 
                                                           filesystem_kws,
                                                           config["query_config"]["adrn_record_downloader_config"])
        elif(config["query_config"]['query_method'] == "time_range"):
            self._record_downloader = TimeRangeRecordDownloader(config["query_config"]["common_config"], 
                                                                filesystem_kws,
                                                                config["query_config"]["time_range_record_downloader_config"])
        elif(config["query_config"]['query_method'] == "jira"):
            self._record_downloader = JiraRecordDownloader(config["query_config"]["common_config"], 
                                                           filesystem_kws,
                                                           config["query_config"]["jira_record_downloader_config"], )
    
    def get_env_and_filesystem_kws(self, config):
        if(config["query_config"]["filesystem"]== "ks3"):
            env = config["ks3_config"]["env"]
            filesystem_kws = config["ks3_config"]["filesystem_kws"]
        elif(config["query_config"]["filesystem"] == "fds"):
            env = config["fds_config"]["env"]
            filesystem_kws = config["fds_config"]["filesystem_kws"]
        else:
            env = config["fds_config"]["env"]
            filesystem_kws = config["fds_config"]["filesystem_kws"]
        
        common_env = config["common_env"]
        common_env.update(env)

        filesystem_kws["key"] = f"{common_env['XIAOMI_ACCESS_KEY_ID']}"
        filesystem_kws["secret"] = f"{common_env['XIAOMI_SECRET_ACCESS_KEY']}"
        filesystem_kws["default_block_size"] *= 1024 * 1024
        

        return common_env, filesystem_kws
    
    def download(self):
        self._record_downloader.download()
        


def main(args):
    record_downloader = RecordDownloader(args)
    record_downloader.download()


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Report Generator')
    root = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--config', default=os.path.join(root,"config/config.yaml"), type=str)
    args = parser.parse_args()
    
    main(args)