import os
import argparse
from datetime import datetime

os.environ["XIAOMI_ACCESS_KEY_ID"] = "CAKINBIXSTXQBDEDYGC"
os.environ["XIAOMI_SECRET_ACCESS_KEY"] = "wpBPiQQgKAYA6YsO2Fyf8jCsEDfhEY9CqqnywzPh"
os.environ["XIAOMI_IAM_ACCESS_KEY_ID"] = "CAKINBIXSTXQBDEDYGC"
os.environ["XIAOMI_IAM_SECRET_ACCESS_KEY"] = "wpBPiQQgKAYA6YsO2Fyf8jCsEDfhEY9CqqnywzPh"
os.environ['XIAOMI_USERNAME'] = "fangshu"
os.environ['XIAOMI_DEPARTMENT'] = "泊车感知"
os.environ['AD_CLOUD_DATASET_CALL_ENV'] = "tjv1"
os.environ['AD_CLOUD_XIAOMI_DEPARTMENT'] = "泊车感知"
os.environ['EVENT_TYPE'] = "prod"

from ad_cloud.event import EventSearcher
from ad_cloud.event.models.client_models.event_model import QueryReq
from ad_cloud.event.client.prod_event_client import EventClient
from ad_cloud.event.client.event_client import EventClient as TestEventClient
from ad_cloud.event.models.model import ENVParams
from typing import List, Optional
from glob import glob
import pandas as pd
from tqdm.contrib.concurrent import thread_map, process_map
from tqdm import tqdm

class TestQueryReq(QueryReq):
    jira_ids: Optional[List[str]]


class JiraDownloader(object):
    def __init__(self, jira_names=None, save_path=None):
        self.jira_names = jira_names
        self.save_path = save_path
        os.makedirs(self.save_path, exist_ok=True)

    def process(self):
        if isinstance(self.jira_names, str):
            jira_ids = self.jira_names.split(",")
        else:
            jira_ids = self.jira_names
        res_all = process_map(
            self.sub_process,
            jira_ids,
            total=len(jira_ids),
            desc="Download Jira Files",
            chunksize=10, max_workers=10,
        )

    def sub_process(self, jira_id):
        print("Processing: ", jira_id)
        save_jira_path = os.path.join(self.save_path, jira_id + ".record")
        if os.path.exists(save_jira_path):
            print(f"File already exists: {save_jira_path}")
            return
        save_jira_path = os.path.join(self.save_path, jira_id + ".mcap")
        if os.path.exists(save_jira_path):
            print(f"File already exists: {save_jira_path}")
            return

        try:
            req = QueryReq(jira_id=jira_id)
            client = EventClient()
            resp = client.query_event_list(req)
            event = None
            if resp.data:
                # 处理量产的数据
                event = resp.data[0]
            else:
                # 处理路测的数据
                req = TestQueryReq(jira_ids=[jira_id])
                os.environ['EVENT_TYPE'] = 'test'
                resp = TestEventClient().query_event_list(req)
                event = resp.data[0] if resp.data else None
            if event:
                searcher = EventSearcher(str(event.event_id))
                searcher.download_and_save(save_jira_path)
                print(f"Successfully downloaded {jira_id} to {save_jira_path}")
            else:
                print(f"No event found for {jira_id}")
        except Exception as e:
            print(f"Error processing {jira_id}")
            print(e)


def main():
    parser = argparse.ArgumentParser(
        description="Download Jira files from AD Cloud",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Download single Jira
  python jira_download.py ADM2-112597

  # Download multiple Jiras (comma-separated)
  python jira_download.py ADM2-112597,ADM2-112592

  # Download with custom output directory
  python jira_download.py ADM2-112597,ADM2-112592 -o /custom/path
        """
    )
    parser.add_argument(
        "--jira-ids",
        help="Jira ID(s) to download. Can be comma-separated (e.g., ADM2-112597,ADM2-112592)"
    )
    parser.add_argument(
        "-o", "--output-dir",
        default=None,
        help="Directory to save downloaded Jira files (default: /media/lw/Samsung_T5/jiras/YYYYMMDD_HHMMSS)"
    )

    args = parser.parse_args()

    # 解析 Jira IDs（支持逗号分隔或空格分隔）
    if "," in args.jira_ids:
        jira_list = [jira.strip() for jira in args.jira_ids.split(",")]
    else:
        jira_list = [jira.strip() for jira in args.jira_ids.split()]

    # 设置默认输出路径：/media/lw/Samsung_T5/jiras/日期与时间
    if args.output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = f"/media/lw/Samsung_T5/jiras/{timestamp}"

    print(f"Downloading {len(jira_list)} Jira files to {args.output_dir}")
    print(f"Jira IDs: {jira_list}")

    jira_downloader = JiraDownloader(jira_names=jira_list, save_path=args.output_dir)
    jira_downloader.process()
    print("Download completed!")


if __name__ == "__main__":
    main()