import influxdb_client
from influxdb_client.client.write_api import (SYNCHRONOUS)
import xml.etree.ElementTree as ET
import os
import sys

bucket = os.getenv('INFLUXDB_TEST_REPORT_BUCKET')
org = os.getenv('INFLUXDB_ORG')
token = os.getenv('INFLUXDB_TOKEN')
url = os.getenv('INFLUXDB_URL')

GITHUB_SHA = os.getenv('GITHUB_SHA', 'null')
GITHUB_REF = os.getenv('GITHUB_REF', 'null')
GITHUB_HEAD_REF = os.getenv('GITHUB_HEAD_REF', 'null')
GITHUB_RUN_ID = os.getenv('GITHUB_RUN_ID', 'null')
GITHUB_WORKFLOW = os.getenv("GITHUB_WORKFLOW", 'null')
global contextGlobal, timeGlobal, osGlobal
contextGlobal = "null"
osGlobal = 'null'
timeGlobal = '1970-01-01T00:00:00'

client = influxdb_client.InfluxDBClient(
   url=url,
   token=token,
   org=org
)
#client.delete_api().delete("2020-01-01T00:00:00.000Z", "2028-01-01T00:00:00.000Z", "", bucket, org)
# exit(0)
write_api = client.write_api(write_options=SYNCHRONOUS)
read_api = client.query_api()
def getPrevFailures(config):
    got = read_api.query(f'''from(bucket:"{bucket}")
        |> range(start: 1970)
        |> filter(fn: (r) => r._measurement == "test")
        |> filter(fn: (r) => r.config == "{config}")
        |> filter(fn: (r) => r._field == "status")
        |> filter(fn: (r) => r._value < 3)
        |> last()
        |> group()
    ''', org=org)
    if (len(got) < 1): return []
    fails = list(map(lambda r: r['testName'], got[0].records))
    # print(fails)
    return fails

def writeTestsuite(
    context: str,
    config: str,
    labels: str,
    protocol: str,
    platform: str,
        
    tests: int,
    passed: int,
    failed: int,
    fixed: int,
    broken: int,
    skipped: int
):
    testsuite = (
        influxdb_client
        .Point("testsuite")
        .time(timeGlobal)
        .tag("workflow", GITHUB_WORKFLOW)
        .tag("GITHUB_SHA", GITHUB_SHA)
        .tag("GITHUB_REF", GITHUB_REF)
        .tag("GITHUB_HEAD_REF", GITHUB_HEAD_REF)
        .tag("GITHUB_RUN_ID", GITHUB_RUN_ID)
        .tag("os", osGlobal)
        .tag("config", config)
        .tag("context", context)
        .tag("labels", labels)
        .tag("DEPTHAI_PLATFORM", platform)
        .tag("DEPTHAI_PROTOCOL", protocol)
        .field("tests", tests)
        .field("skipped", skipped)
        .field("errored", 0)
        .field("failed", failed)
        .field("passed", passed)
        .field("fixed", fixed)
        .field("broken", broken)
    )
    write_api.write(bucket=bucket, org=org, record=testsuite)
    print(f"WRITTEN TESTSUITE {passed}/{tests} PASSED")
    return

def statusToNumber(status):
    match status:
        case 'failed':
            return 1
        case 'skipped':
            return 2
        case 'errored':
            return 0
        case 'passed':
            return 3
        
def writeSingleTest(
    context: str,
    config: str,
    labels: str,
    protocol: str,
    platform: str,

    testname: str,
    status: str,
    logs: str | None,
):
    test = (influxdb_client
        .Point("test")
        .time(timeGlobal)
        .tag("workflow", GITHUB_WORKFLOW)
        .tag("GITHUB_SHA", GITHUB_SHA)
        .tag("GITHUB_REF", GITHUB_REF)
        .tag("GITHUB_HEAD_REF", GITHUB_HEAD_REF)
        .tag("GITHUB_RUN_ID", GITHUB_RUN_ID)
        .tag("os", osGlobal)
        .tag("config", config)
        .tag("context", context)
        .tag("labels", labels)
        .tag("DEPTHAI_PLATFORM", platform)
        .tag("DEPTHAI_PROTOCOL", protocol)
        .tag("testName", testname)
        .field("status", statusToNumber(status)))
    
    if (logs != None):
        if(logs.__len__() > 1_000_000):
            print(f"{testname} LOGS ARE TOO LARGE AT {logs.__len__()//1024}, TRUNCATING START DOWN TO 1M chars")
            logs = f"START OF LOGS WAS TRUNCATED BECAUSE IT WAS OVER A MB!!!\n\n{logs[-1_000_000:]}"
        test.field("logs", logs)
    
    print(f"WRITING TESTCASE '{testname}', {status}")
    write_api.write(bucket=bucket, org=org, record=test)
    return

def parseTestSummary():
    if len(sys.argv) < 3:
        print("COULD NOT UPLOAD JUNITS TO INFLUXDB, BECAUSE SCRIPT WAS CALLED INCORRECTLY")
        return
    global timeGlobal, osGlobal
    timeGlobal = sys.argv[1]
    osGlobal = sys.argv[2]
    junits = ET.parse(sys.argv[3])
    testsuites = junits.getroot()
    for testsuite in testsuites:
        name = testsuite.get('name', '')
        [context, config] = name.split(' / ') if ' / ' in name else ['null', name]
        labels = testsuite.get('labels', '')
        protocol = testsuite.get('DEPTHAI_PROTOCOL', 'null')
        platform = testsuite.get('DEPTHAI_PLATFORM', 'null')
        skipped = int(testsuite.get('skipped', '-1'))
        print("Processing testsuite", name)
        props = testsuite.find('properties')
        if (props == None):
            print("Could not find testsuite properties, skipping!")
            continue
        failed = -1; passed = -1; total = -1; brokenCt = 0; fixedCt = 0
        for prop in props:
            name = prop.get('name')
            match name:
                case 'ctest.summary':
                    #presumed format: Passed=n, Failed=m, Total=nm
                    summary = prop.get('value')
                    if (type(summary) != str):
                        print("Invalid ctest.summary, skipping!")
                        continue
                    s=summary.split("=")
                    passed = int(s[1][0:s[1].index(',')])
                    failed = int(s[2][0:s[2].index(',')])
                    total = int(s[3])
                case _:
                    print("Unknown property:", name)
        prevFailureList = getPrevFailures(config=config)
        for testcase in testsuite:
            if (testcase.tag != 'testcase'): continue
            summary = testcase.get('name') or " unknown_test"
            testname = summary.split(' ')[1] or 'unknown_test'

            failure = testcase.find("failure")
            if failure is not None:
                broken = False
                if (prevFailureList.count(testname) == 0): #not present means it just broke
                    broken = True
                    brokenCt += 1
                else: prevFailureList.remove(testname) #means it's been broken
                logs = ''
                for t in failure.itertext():
                    s = t.strip()
                    if len(s)>10:
                        logs += s+"\n\n"
                if (len(logs) < 16): logs = "Could not find logs"
                writeSingleTest(
                    labels=labels, platform=platform, protocol=protocol, context=context, config=config,
                    testname=testname, logs=logs, status="failed"
                )
                continue
            if testcase.find("success") is not None:
                writeSingleTest(
                    labels=labels, platform=platform, protocol=protocol, context=context, config=config,
                    testname=testname, status="passed", logs=None
                )
                continue
            if testcase.find("skip") is not None:
                writeSingleTest(
                    labels=labels, platform=platform, protocol=protocol, context=context, config=config,
                    testname=testname, status="skipped", logs=None
                )
                continue
        fixedCt = len(prevFailureList)
        writeTestsuite(
            labels=labels, platform=platform, protocol=protocol, context=context, config=config,
            passed=passed, failed=failed, tests=total, fixed=fixedCt, broken=brokenCt,
        )

parseTestSummary()