import influxdb_client
from influxdb_client.client.write_api import (SYNCHRONOUS)
import xml.etree.ElementTree as ET
import os
import re
from urllib.parse import quote
import json
import sys

bucket = os.getenv('INFLUXDB_TEST_REPORT_BUCKET')
org = os.getenv('INFLUXDB_ORG', '')
token = os.getenv('INFLUXDB_TOKEN', '')
url = os.getenv('INFLUXDB_URL')

GITHUB_REF = os.getenv('GITHUB_REF', '')
GITHUB_SHA = os.getenv('GITHUB_SHA', '')
GITHUB_RUN_ID = os.getenv('GITHUB_RUN_ID', '')
GITHUB_HEAD_REF = os.getenv('GITHUB_HEAD_REF', '')

client = influxdb_client.InfluxDBClient(
   url=url,
   token=token,
   org=org
)
read_api = client.query_api()

def getGithubRefUrl(ref: str) -> str:
    clean_ref = ref.strip()
    if clean_ref.upper() == "HEAD":
        return ""

    # 1. Handle Pull Requests (refs/pull/<pr_number>/merge or /head)
    pr_match = re.match(r"^refs/pull/(\d+)/(?:merge|head)$", ref)
    if pr_match:
        pr_number = pr_match.group(1)
        return f"pull/{pr_number}"

    # Helper function to match quote(s, safe='') in JavaScript's encodeURIComponent
    def encode_ref_path(path: str) -> str:
        return "/".join(quote(segment, safe="") for segment in path.split("/"))

    # 2. Handle standard branch refs (refs/heads/<branch_name>)
    if ref.startswith("refs/heads/"):
        branch_name = ref[len("refs/heads/") :]
        return f"tree/{encode_ref_path(branch_name)}"

    # 3. Handle tag refs (refs/tags/<tag_name>)
    if ref.startswith("refs/tags/"):
        tag_name = ref[len("refs/tags/") :]
        return f"releases/tag/{encode_ref_path(tag_name)}"

    # 4. Fallback for Commit SHAs or un-prefixed custom refs
    return encode_ref_path(ref)

def statusToEmoji(status):
    emojiList = ['exclamation', 'x', 'white_square', 'white_check_mark']
    return emojiList[status]

def getTestHistory(ref) -> list[list[dict]]:
    got = read_api.query(f'''
statuses = from(bucket: "{bucket}")
    |> range(start: 1970)
    |> filter(fn: (r) =>
        r._measurement == "test" and
        r._field == "status" and
        r.GITHUB_REF == "{ref}"
    )

latestRun =
    statuses
        |> group()
        |> max(column: "_time")
        |> findRecord(fn: (key) => true, idx: 0)

failedTestNames =
    statuses
        |> filter(fn: (r) =>
            r._time == latestRun._time and
            r._value == 1
        )
        |> keep(columns: ["testName","config","context","os"])
        |> group(columns: ["testName","config","context","os"])

history =
    statuses
        |> group(columns: ["testName", "config","context","os"])
        |> sort(columns: ["_time"], desc: true)
        |> limit(n: 5)

join(
    tables: {{
        history: history,
        currentlyFailed: failedTestNames
    }},
    on: ["testName", "config","context","os"],
    method: "inner"
)''', org=org)
    history = list(map(lambda r: r.records, got))
    return history

def writeTestHistory(history):
    topBlock = {
        "type": "section",
        "text": {
            "type": "mrkdwn",
            "text": f'''ref: <https://github.com/luxonis/depthai-core/{getGithubRefUrl(GITHUB_REF)}|{GITHUB_REF}> {f"({GITHUB_HEAD_REF})" if GITHUB_HEAD_REF != '' else ''}
Commit: <https://github.com/luxonis/depthai-core/commit/{GITHUB_SHA}|{GITHUB_SHA[0:8]}>
Run: <https://github.com/luxonis/depthai-core/actions/runs/{GITHUB_RUN_ID}|{GITHUB_RUN_ID}>

'''
        }
    }

    tableBlock = {
        "type": "section",
        "text": {
            "type": "mrkdwn",
            "text": "All test passed! :white_check_mark:"
        }
    }
    if len(history) > 0:
        tableRows = []
        headerRow: list[dict] = [{"type": "raw_text", "text": "test name and config"}]
        columnsDict = {}
        for i in history:
            for j in i:
                columnsDict[j["_time"]] = j['GITHUB_SHA']
        columns = list(columnsDict.items())
        columns.sort(key=lambda a: a[0], reverse=True)
        for i in columns:
            headerRow.append({
                "type": "rich_text",
                "elements": [
                    {
                        "type": "rich_text_section",
                        "elements": [
                            {
                                "text": i[1][0:8],
                                "type": "link",
                                "url": f"https://github.com/luxonis/depthai-core/commit/{i[1]}"
                            },
                        ]
                    }
                ]
            })
        tableRows.append(headerRow)

        for i in history:
            row: list[dict] = []
            columnIdx = 0
            for j in i:
                while columnIdx < len(columns) and columns[columnIdx][0] != j["_time"]:
                    row.append({"type": "raw_text", "text": "not ran"})
                    columnIdx += 1
                row.append({
                    "type": "rich_text",
                    "elements": [
                        {
                            "type": "rich_text_section",
                            "elements": [
                                {
                                    "name": statusToEmoji(j["_value"]),
                                    "type": "emoji",
                                },
                            ]
                        }
                    ]
                })
                columnIdx += 1
            row.reverse()
            tableRows.append([{
                "type": "raw_text",
                "text": f'{i[0]["testName"]} ({i[0]["config"]}@{i[0]["context"]}/{i[0]["os"]})'
            }, *row])

        tableBlock = {
            "type": "table",
            "rows": tableRows
        }
    ret = {
        "text": f"{GITHUB_SHA[0:8]} test run summary",
        "channel": os.getenv("SLACK_BOT_CHANNEL_ID", ""),
        "blocks": [topBlock, tableBlock]
    }
    return json.dump(ret, fp=open(sys.argv[1], mode='w'))

history = getTestHistory(GITHUB_REF)
writeTestHistory(history)
