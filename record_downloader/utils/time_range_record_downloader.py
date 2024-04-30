#!/usr/bin/env python
from datetime import datetime, timedelta
import os
import time
from utils.event_record_downloader import EventRecordDownloader
from utils.adrn_record_downloader import AdrnRecordDownloader

VEHICLE_MAP = {
    "adr::car:prod:mo1-mt:0060":"adr::car:prod:MS11-1:LVEQ4HEVTV9AAFNF6",
    "adr::car:prod:mo1-mt:0227":"adr::car:prod:MS11-1:LNBQZJRM7UYKNPRB4",
    "adr::car:prod:mo5-mt:0257":"adr::car:prod::LVEQU3CPRGPVB3RD1",
    "adr::car:prod:mo5-mt:0091":"adr::car:prod:MS11-5:LVEQ38JLL7YXZ8PD1"
}

class TimeRangeRecordDownloader():
    def __init__(self, common_config, filesystem_kws, downloader_config = None):
        self._downloader_config = downloader_config

        if(self._downloader_config["download_method"] == "event"):
            self._record_downloader = EventRecordDownloader(common_config, filesystem_kws)
        elif(self._downloader_config["download_method"] == "adrn"):
            self._record_downloader = AdrnRecordDownloader(common_config, filesystem_kws)


    def query_event_id(self, vehicle, event_key, start_timestamp, end_timestamp):
        from ad_cloud.event import seeker
        from ad_cloud.event import ProdEventQueryParam
        from ad_cloud.event import TimeRange
        prodquery = ProdEventQueryParam(
            event_key=event_key,
            car_adrn=vehicle,
            trigger_time_range=TimeRange(
                start_timestamp=start_timestamp, 
                end_timestamp=end_timestamp
            ),
        )
        return seeker.query_prod_event(prodquery)
    
    def download(self):
        if(self._downloader_config['download_method'] == "event"):
            vehicle_name = VEHICLE_MAP[self._downloader_config['vehicle']]
            for time_range in self._downloader_config['time_range']:
                event_ids = self.query_event_id(vehicle_name, 
                                                self._downloader_config['event_key'],
                                                time_range[0], 
                                                time_range[1])
                for event_id in event_ids:
                    self._record_downloader.download_record(event_id)
        elif(self._downloader_config['download_method'] == "adrn"):
            re = self._downloader_config['vehicle'].split(":")

            def datetime2timestamp(ts):
                t = ts.timetuple() 
                timestamp = int(time.mktime(t))
                return timestamp

            for time_range in self._downloader_config['time_range']:
                start_ts = datetime.strptime(self._downloader_config['time_range'][0], '%Y-%m-%d %H:%M:%S')
                end_ts = datetime.strptime(self._downloader_config['time_range'][1], '%Y-%m-%d %H:%M:%S')
                start_timestamp = datetime2timestamp(start_ts)
                end_timestamp = datetime2timestamp(end_ts)

                adrn = f"adr::dataclip:{re[-3]}.{re[-2]}.{re[-1]}::{str(start_timestamp) + '000000000'}:{str(end_timestamp) + '000000000'}"

                self._record_downloader.download_record(adrn)