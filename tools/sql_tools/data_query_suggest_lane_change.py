import os
from datetime import datetime
from turtle import right
import rospy
from pluspy import db_utils
import argparse
import munch
import sys
import rosbag
import rospy
import pytz
import logging
import pandas as pd
from google.protobuf.json_format import Parse
import time
from datetime import datetime
import json
import sys
import munch
import numpy as np
from pluspy import db_utils
from sqlalchemy import create_engine, text, update
from pdb import set_trace as bp

logger = logging.getLogger(__name__)


def unixTimeMillis(dt):
    epoch = datetime.utcfromtimestamp(0)
    return (dt - epoch).total_seconds()


def LaneChangeFilter(start, end, click_db_conf, output_folder):
    print('Now collecting data from ' + start + ' to ' + end)

    query_lc = ("SELECT "
                "        bag_name ,"
                "        name ,"
                "        mode ,"
                "        vehicle ,"
                "        utc_start,"
                "        ROUND(toUnixTimestamp64Milli(utc_start)/1000, 1) as cur_time,"
                "        toTimeZone(utc_start, 'Asia/Shanghai') as time_bj,"
                "        toUnixTimestamp(utc_start) as start_time,"
                "        (toUnixTimestamp64Milli(utc_end) - toUnixTimestamp64Milli(utc_start))/ 1000 as duration,"
                "        JSONExtractString(metrics, 'direction') as direction,"
                "        JSONExtractString(metrics, 'reason') as reason,"
                "        JSONExtractString(metrics, 'accept') as accept,"
                "        JSONExtractString(metrics, 'source') as source,"
                "        JSONExtractString(metrics, 'cancel_reason') as cancel_reason,"
                "        if(equals(accept, 'false'), 0., 1.) as accept_bool"
                "    from"
                "        bag_events be"
                "    where"
                "        name = 'suggest_lane_change_analysis'"
                "        and (vehicle = 'pdb-l4e-b0006')"
                "        and road_type = 'highway'"
                "        and category = 'Behavior'"
                "        and be.mode ='auto'"
                "        and direction = 'left'"
                "        and tag = 'Suggest'"
                "        and label = 'LaneChange'"
                "        and utc_start BETWEEN :start_t and :end_t"
                "    ORDER BY start_time")

    with db_utils.db_session_open_close(click_db_conf.get('clickhouse')) as cdb_session:
        bag_ids = []
        bags = []
        res = cdb_session.execute(query_lc, {'start_t': str(start), 'end_t': str(end)})

        # for ts, utc, bag_name, vehicle, ego_lane_change_intention, egoRoadPosition in res:
        #     bag_link = "https://bagdb-cn.plus.ai/plusview/{}?seek={}".format(bag_name, utc)
        #     bags.append([bag_name, bag_link])
        columns = [
            "bag_name", "name ", "mode ", "vehicle ", "utc_start", " cur_time", " time_bj", "start_time", "duration", "direction", "reason",
            "accept", "source", " cancel_reason", " accept_bool"
        ]

        df = pd.DataFrame(res.fetchall(), columns=columns)

    for utc, bag_name in df.loc[:, ["start_time", "bag_name"]].values:
        bag_link = "https://bagdb-cn.plus.ai/plusview/{}?seek={}".format(bag_name, utc)
        bags.append([bag_name, bag_link])

    columns = ['bag_name', 'bag_link']
    result_bags_df = pd.DataFrame(bags, columns=columns)
    result_bags_df.to_csv(os.path.join(output_folder, "planning_lane_change.csv"))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--start_time', action="store", default='2022-12-26 00:00:00')
    parser.add_argument('-e', '--end_time', action="store", default='2022-12-26 24:00:00')
    parser.add_argument('--time_zone', action="store", default="Asia/Shanghai")
    parser.add_argument('--output_folder',
                        action="store",
                        default='/home/plusai/Projects/tools/fuel_economy/engine_operation_points_analysis/B6')
    parser.add_argument('--area', action="store", default='sz')  # us or sz
    parser.add_argument('--password_clickhouse', action="store", default='ex4u1balSAeR68uC')

    args = parser.parse_args()
    if args.password_clickhouse == 'none':
        clickhouse_password = os.environ.get('CLICKHOUSE_PASS')
    else:
        clickhouse_password = args.password_clickhouse

    if args.area == 'sz':
        url = 'sz-typostgres:5432/bagdb'
    elif args.area == 'us':
        url = 'data1.corp.plus.ai:5432/bagdb'
    else:
        print('no valid area option')
        sys.exit(0)

    username = 'bagdb_guest'

    # engine = create_engine('postgresql://%s:%s@' % (username, password) + url)
    # connect to click house
    click_db_conf = {
        'clickhouse': {
            'driver': 'clickhouse+native',
            'port': 9000,
            'database': 'bagdb',
            'host': 'clickhouse-cn',
            'user': 'plus_viewer',
            'password': clickhouse_password,
            'connect_timeout': 30,
            'keep_alive_timeout': 90
        }
    }
    click_db_conf = munch.munchify(click_db_conf)

    LaneChangeFilter(args.start_time, args.end_time, click_db_conf, args.output_folder)
