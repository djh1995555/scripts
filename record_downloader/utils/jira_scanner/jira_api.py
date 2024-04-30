import json
import re

from utils.jira_scanner.cas_auto_login_v2_token import (
    HTTPBearerAuth,
    TokenManger,
    create_session,
)
from  utils.jira_scanner.global_config import *


class JiraApi:
    fake_jira = False

    def __init__(self, proj):
        self._proj = proj
        self._url = "https://viim.ev.mi.com"
        self._key_list = []
        self._record_list = []
        self.session = create_session(self._url)

    def construct_request_headers(self):
        return {"Content-Type": "application/json"}

    def load_issue_text(self, path):
        fr = open(path, "r+", encoding="utf-8")
        for line in fr:
            self.issue_text_ = line
        fr.close()

    def save_issue_text(self, path):
        fw = open(path, "w+", encoding="utf-8")
        fw.write(self.issue_text_)
        fw.close()

    def save_issue_output(self, path):
        fw = open(path, "w+", encoding="utf-8")
        for line in self._key_list:
            fw.write(line + "\n")
        fw.write("-----\n")
        for line in self._record_list:
            fw.write(line + "\n")
        fw.close()

    def save_issue_ids_txt(self, issue_ids, path):
        fw = open(path, "w+", encoding="utf-8")
        for line in issue_ids:
            fw.write(line + "\n")
        fw.close()

    def get_record_list(self):
        return self._record_list

    def export_issue_info(self):
        to_print = True

        item = json.loads(self.issue_text_)["fields"]

        res_summary = item["summary"]
        res_description = item["description"]
        if to_print:
            print("------------summary------------")
            print(res_summary)

            res = item["issuetype"]
            print("------------issuetype------------")
            print(res)

            res = item["project"]
            print("------------project------------")
            print(res)

            res = item["labels"]
            print("------------labels------------")
            print(res)

            res = item["assignee"]
            print("------------assignee------------")
            print(res)

            res = item["reporter"]
            print("------------reporter------------")
            print(res)

            print("------------description------------")
            print(res_description)
            print("-----------------------------------")

        i = res_summary.find("【")
        j = res_summary.find("】")
        if to_print:
            print(res_summary[i + 1 : j])
        self._key_list.append(res_summary[i + 1 : j])

        i = res_description.find("【未切分数据下载】:")
        j = res_description.find("【未切分解析数据地址】")

        # Parse record list
        for line in res_description.split("\n"):
            if "原始数据切片下载" in line or "未切分数据下载" in line:
                print(line)
                res = re.findall("cnbj1-fds.api.xiaomi.net(.*?)\?", line)
                self._record_list = res
                break

        return self._key_list, self._record_list

    def get_issue_data(self, issue_key):
        session = create_session(self._url)

        url = self._url + "/rest/api/2/issue/{}".format(issue_key)
        resp = session.get(url, auth=HTTPBearerAuth(CONFIG_PATH))

        print(resp.status_code)
        print(resp.url)

        self.issue_text_ = resp.text
        return resp.text
        # item = json.loads(resp.text)["fields"]
        # self.export_issue_info()

    def list_issue_description(self, issue_key):
        session = create_session(self._url)

        url = self._url + "/rest/api/2/issue/{}".format(issue_key)
        resp = session.get(url, auth=HTTPBearerAuth(CONFIG_PATH))

        print(resp.status_code)
        print(resp.url)

        res = json.loads(resp.text)["fields"]["description"]
        print(res)

    def list_issue_summary(self, issue_key):
        session = create_session(self._url)

        url = self._url + "/rest/api/2/issue/{}".format(issue_key)
        resp = session.get(url, auth=HTTPBearerAuth(CONFIG_PATH))

        print(resp.status_code)
        print(resp.url)

        res = json.loads(resp.text)["fields"]["summary"]
        print(res)

    def filter_issue(
        self, past_days=1, max_results=55, vehicle_name="bhx", path="./issue_ids.txt"
    ):
        session = create_session(self._url)

        url = self._url + "/rest/api/2/search"

        payload = json.dumps(
            {
                "jql": "project = PILOT AND created > -"
                + str(past_days)
                + "d AND reporter = micar-infra-data",
                "maxResults": max_results,
                "fields": [
                    "summary",
                    "description",
                ],
                "startAt": 0,
            }
        )

        headers = {"content-type": "application/json"}

        response = session.post(
            url, payload, headers=headers, auth=HTTPBearerAuth(CONFIG_PATH)
        )
        print(response.status_code)

        # print(json.dumps(json.loads(response.text), sort_keys=True, indent=4, separators=(",", ": ")))

        tesx_raw = json.loads(response.text)
        self._issues = tesx_raw["issues"]

        print("Max Output Results: ", tesx_raw["maxResults"])
        print("Total Fetch  Results: ", tesx_raw["total"])

        res_issue_ids = []
        for issue in self._issues:
            lines = issue["fields"]["description"].split("\n")
            if len(lines) > 0:
                if vehicle_name in lines[0]:
                    res_issue_ids.append(issue["key"])

        print(res_issue_ids)
        print("Total Filter  Results: ", len(res_issue_ids))

        self.save_issue_ids_txt(res_issue_ids, path)

    def search_issue_by_key(self, key):
        url = f"{self._url}/rest/api/latest/search"
        headers = self.construct_request_headers()
        body = {
            "jql": f"key = {key}",
            "startAt": 0,
            "maxResults": 1,
        }
        resp = self.session.post(
            url=url,
            headers=headers,
            auth=HTTPBearerAuth(CONFIG_PATH),
            data=json.dumps(body),
        )
        if resp.status_code != 200:
            print(f"{key} NOT FOUND!")
            return None
        return json.loads(resp.text)["issues"][0]

    def search_issue_by_jql(self, jql_str):
        url = f"{self._url}/rest/api/latest/search"
        headers = self.construct_request_headers()
        body = {"jql": jql_str, "startAt": 0, "maxResults": 100000}
        resp = self.session.post(
            url=url,
            headers=headers,
            auth=HTTPBearerAuth(CONFIG_PATH),
            data=json.dumps(body),
        )
        if resp.status_code != 200:
            print("Search Failed!")
            return None
        return resp.json()
