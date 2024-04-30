#!/usr/bin/env python
import os
import re
import pandas as pd
from datetime import datetime, timedelta
from utils.event_record_downloader import EventRecordDownloader
from utils.adrn_record_downloader import AdrnRecordDownloader
from utils.jira_scanner.global_config import JIRA_PROJ
from utils.jira_scanner.jira_api import JiraApi

class JiraRecordDownloader():
    def __init__(self, common_config, filesystem_kws, downloader_config = None):
        self._downloader_config = downloader_config
        if(len(common_config["output_dir"]) == 0):
            current_directory = os.getcwd()
            self._output_dir = os.path.join(current_directory,"record")
        else:
            self._output_dir = common_config["output_dir"]

        self._event_record_downloader = EventRecordDownloader(common_config, filesystem_kws)
        self._adrn_record_downloader = AdrnRecordDownloader(common_config, filesystem_kws)
        
    def download_from_jira_api(self):
        jira_api = JiraApi(JIRA_PROJ)
        res = jira_api.search_issue_by_jql(event_id)
        if res is None:
            print("[red] jql search failed.[/]")
            return
        issues = res["issues"]

        for issue in issues:
            issue_id = issue["key"]
            description = issue["fields"]["description"]
            if re.search("【Event ID】", description):
                event_id = self.get_event_id(description)
                self._event_record_downloader.download_record(event_id)
            else:
                for line in description.split("\n"):
                    if "【测试车次】: " in line:
                        line = re.sub("{.*?}", "", line)
                        search_res = re.search(":(.*?\.\d+)", line)
                        vehicle_type = search_res.group(1).strip()
                    if "【测试时间】:" in line or "【测试起止时间】: " in line:
                        line = re.sub("{.*?}", "", line)
                        search_res = re.search(
                            "(\d{4}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?(\d{4}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?(\d{2})",
                            line,
                        )
                        start_datetime_str = f"{search_res.group(1)}-{search_res.group(2)}-{search_res.group(3)} {search_res.group(4)}:{search_res.group(5)}:{search_res.group(6)}"
                        end_datetime_str = f"{search_res.group(7)}-{search_res.group(8)}-{search_res.group(9)} {search_res.group(10)}:{search_res.group(11)}:{search_res.group(12)}"

                        start_datetime = datetime.strptime(
                            start_datetime_str, "%Y-%m-%d %H:%M:%S"
                        )
                        end_datetime = datetime.strptime(
                            end_datetime_str, "%Y-%m-%d %H:%M:%S"
                        )

                        start_timestamp = int(start_datetime.timestamp() * 1e9)
                        end_timestamp = int(end_datetime.timestamp() * 1e9)
                        
                adrn = f"adr::dataclip:{vehicle_type}::{start_timestamp}:{end_timestamp}"
                self._adrn_record_downloader.download_record(adrn)

    def download_from_jira_csv(self):
        
        jira_csv_filepath = self._downloader_config["jira_csv"]
        issues = pd.read_csv(jira_csv_filepath)
        target_issue_ids = self._downloader_config["issue_ids"]
        target_issues = issues[issues['问题关键字'].isin(target_issue_ids)]

        # Extract the 'description' column data from the filtered rows
        for index, row in target_issues.iterrows():
            description = row['描述']
            issue_id = row['问题关键字']
            print(issue_id)
            position = description.find("【Event ID】")
            if position == -1:
                print(f"get event id of {issue_id} failed!") 
                continue
            tmp = description[position: position + 50]
            event_id = tmp.split('[')[1].split('|')[0]
            print(f"Event id of {issue_id} is {event_id}")
            output_dir = os.path.join(self._output_dir, "issue_record", f"{issue_id}")
            self._event_record_downloader.download_record(event_id, output_dir)

            description_filepath = os.path.join(output_dir, f"{issue_id}_description.csv")
            row.to_csv(description_filepath)
    

    def download(self):
        self.download_from_jira_csv()