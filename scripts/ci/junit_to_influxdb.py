import influxdb_client
from influxdb_client.client.write_api import (SYNCHRONOUS)
import xml.etree.ElementTree as ET
import datetime
import os
import sys

bucket = os.getenv('INFLUXDB_TEST_REPORT_BUCKET')
org = os.getenv('INFLUXDB_ORG')
token = os.getenv('INFLUXDB_TOKEN')
url = os.getenv('INFLUXDB_URL')

timeGlobal = datetime.datetime.now().isoformat()
GITHUB_SHA = os.getenv('GITHUB_SHA', 'null')
GITHUB_REF = os.getenv('GITHUB_REF', 'null')
GITHUB_RUN_ID = os.getenv('GITHUB_RUN_ID', 'null')
global workflowGlobal
workflowGlobal = "null"

client = influxdb_client.InfluxDBClient(
   url=url,
   token=token,
   org=org
)
# #client.delete_api().delete("2020-01-01T00:00:00.000Z", "2028-01-01T00:00:00.000Z", "", bucket, org)
# # exit(0)
write_api = client.write_api(write_options=SYNCHRONOUS)
# def getPrevFailures(
    
# ):
#     return []

# def writeTestsuite(
#     context: str,
#     config: str,
#     labels: str,
#     protocol: str,
#     platform: str,
        
#     tests: int,
#     passed: int,
#     failed: int,
#     fixed: int,
#     broken: int
# ):
#     testsuite = (
#         influxdb_client
#         .Point("testsuite")
#         .time(timeGlobal)
#         .tag("workflow", workflowGlobal)
#         .tag("GITHUB_SHA", GITHUB_SHA)
#         .tag("GITHUB_REF", GITHUB_REF)
#         .tag("GITHUB_RUN_ID", GITHUB_RUN_ID)
#         .tag("config", config)
#         .tag("context", context)
#         .tag("labels", labels)
#         .tag("DEPTHAI_PLATFORM", platform)
#         .tag("DEPTHAI_PROTOCOL", protocol)
#         .field("tests", tests)
#         .field("skipped", 0)
#         .field("errored", 0)
#         .field("failed", failed)
#         .field("passed", passed)
#         .field("fixed", fixed)
#         .field("broken", broken)
#     )
#     write_api.write(bucket=bucket, org=org, record=testsuite)
#     print(f"WRITTEN TESTSUITE {passed}/{tests} PASSED")
#     return

# def statusToNumber(status):
#     match status:
#         case 'failed':
#             return 1
#         case 'skippen':
#             return 2
#         case 'errored':
#             return 0
#         case 'passed':
#             return 3
        

# def writeSingleTest(
#     context: str,
#     config: str,
#     labels: str,
#     protocol: str,
#     platform: str,

#     testname: str,
#     status: str,
#     logs: str | None,
# ):
#     test = (influxdb_client
#         .Point("test")
#         .time(timeGlobal)
#         .tag("workflow", workflowGlobal)
#         .tag("GITHUB_SHA", GITHUB_SHA)
#         .tag("GITHUB_REF", GITHUB_REF)
#         .tag("GITHUB_RUN_ID", GITHUB_RUN_ID)
#         .tag("config", config)
#         .tag("context", context)
#         .tag("labels", labels)
#         .tag("DEPTHAI_PLATFORM", platform)
#         .tag("DEPTHAI_PROTOCOL", protocol)
#         .tag("testName", testname)
#         .field("status", statusToNumber(status)))
    
#     if (logs != None):
#         if(logs.__len__() > 1_000_000):
#             print(f"{testname} LOGS ARE TOO LARGE AT {logs.__len__()//1024}, TRUNCATING START DOWN TO 1M chars")
#             logs = f"START OF LOGS WAS TRUCATED BECAUSE IT WAS OVER A MB!!!\n\n{logs[-1_000_000:]}"
#         test.field("logs", logs)
    
#     print(f"WRITTING TESTCASE '{testname}', {status}")
#     write_api.write(bucket=bucket, org=org, record=test)
#     return

# def parseTesstSummary():
#     if len(sys.argv) < 3:
#         print("COULD NOT UPLOAD JUNITS TO INFLUXDB, BECAUSE SCRIPT WAS CALLED INCORRECTLY")
#         return
#     workflowGlobal = sys.argv[1]
#     junits = ET.parse(sys.argv[2])
#     testsuites = junits.getroot()
#     for testsuite in testsuites:
#         name = testsuite.get('name', '')
#         [context, config] = name.split(' / ') if ' / ' in name else ['null', name]
#         labels = testsuite.get('labels', '')
#         protocol = testsuite.get('DEPTHAI_PROTOCOL', 'null')
#         platform = testsuite.get('DEPTHAI_PLATFORM', 'null')
#         print("Processing testsuite", name)
#         props = testsuite.find('properties')
#         if (props == None):
#             print("Could not find testsuite properties, skipping!")
#             continue
#         failed = -1; passed = -1; total = -1; brokenCt = 0; fixedCt = 0
#         for prop in props:
#             name = prop.get('name')
#             match name:
#                 case 'ctest.summary':
#                     #presumed format: Passed=n, Failed=m, Total=nm
#                     summary = prop.get('value')
#                     if (type(summary) != str):
#                         print("Invalid ctest.summary, skipping!")
#                         continue
#                     s=summary.split("=")
#                     passed = int(s[1][0:s[1].index(',')])
#                     failed = int(s[2][0:s[2].index(',')])
#                     total = int(s[3])
#                 case _:
#                     print("Unknown property:", name)
#         prevFailureList = getPrevFailures()
#         for testcase in testsuite:
#             if (testcase.tag != 'testcase'): continue
#             summary = testcase.get('name') or " unkown_test"
#             testname = summary.split(' ')[1] or 'unkown_test'
#             failure = testcase.find("failure")
#             if failure is not None:
#                 broken = False
#                 if (prevFailureList.count(testname)):
#                     broken = True
#                     brokenCt += 1
#                     prevFailureList.remove(testname)
#                 logs = ''
#                 for t in failure.itertext():
#                     s = t.strip()
#                     if len(s)>10:
#                         logs += s+"\n\n"
#                 if (len(logs) < 16): logs = "Could not find logs"
#                 writeSingleTest(
#                     labels=labels, platform=platform, protocol=protocol, context=context, config=config,
#                     testname=testname, logs=logs, status="failed"
#                 )
#                 continue
#             if testcase.find("success") is not None:
#                 writeSingleTest(
#                     labels=labels, platform=platform, protocol=protocol, context=context, config=config,
#                     testname=testname, status="passed", logs=None
#                 )
#                 continue
#         fixedCt = prevFailureList.__len__()
#         writeTestsuite(
#             labels=labels, platform=platform, protocol=protocol, context=context, config=config,
#             passed=passed, failed=failed, tests=total, fixed=fixedCt, broken=brokenCt,
#         )

# parseTesstSummary()

test = (influxdb_client
    .Point("test")
    .time(timeGlobal)
    .tag("workflow", "workflowGlobal")
    .tag("GITHUB_SHA", GITHUB_SHA)
    .tag("GITHUB_REF", GITHUB_REF)
    .tag("GITHUB_RUN_ID", GITHUB_RUN_ID)
    .tag("config", "config")
    .tag("context", "context")
    .tag("labels", "labels")
    .tag("DEPTHAI_PLATFORM", "platform")
    .tag("DEPTHAI_PROTOCOL", "protocol")
    .tag("testName", "testname")
    .field("status", 1))

print(f"WRITTING TEST")
write_api.write(bucket=bucket, org=org, record=test)