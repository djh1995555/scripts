from utils import *
set_output_dir()

import datetime

def get_event_data():
    from ad_cloud.event import seeker
    from ad_cloud.event import ProdEventQueryParam
    from ad_cloud.event import TESTEventQueryParam
    from ad_cloud.event import TimeRange
    from ad_cloud.utils.logging import logger
    vehicle_type = "adr::car:prod:mo1-mt:0060"
    vehicle_type = "adr::car:prod:MS11-1:LVEQ4HEVTV9AAFNF6"
    start_timestamp = "2024-04-17 16:55:00"
    end_timestamp = "2024-04-17 17:30:00"
    # event_id = seeker.query_event(vehicle_type,start_timestamp,end_timestamp)

    prodquery = ProdEventQueryParam(
        event_key="EVENT_KEY_TRIGGER_APA_PARKING_EXCEPTION",
        car_adrn=vehicle_type,
        trigger_time_range=TimeRange(
            start_timestamp=start_timestamp, end_timestamp=end_timestamp
        ),
        page=1,
        size=10,
    )
    total = seeker.count_prod_event(prodquery)
    event_ids = seeker.query_prod_event(prodquery)
    print(total)
    print(event_ids)
    
    event_id = 16482574
    print("event_id:{}".format(event_id))
    event_seeker=seeker.EventSearcher(str(event_id),event_type=seeker.EVENT_SOURCE_ENUM.PROD)
    print("*****************************************")
    print(event_seeker.query_dataclip())
    print("*****************************************")
    print(event_seeker.query_detail())
    details = event_seeker.query_detail()
    with open('query_detail.log', "w") as query_detail:
        for name, data in vars(details).items():
            if(name == "start_timestamp" or name == "end_timestamp"):
                data = datetime.datetime.fromtimestamp(int(data)/ 10e8).strftime('%Y-%m-%d %H-%M-%S')
            query_detail.write(f"{name} = {data}\n")
    # event_seeker.download_and_save('data.record')
    
if __name__ == '__main__':
    get_event_data()