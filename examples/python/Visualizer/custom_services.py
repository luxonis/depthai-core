#!/usr/bin/env python3
import depthai as dai
import signal
import sys
import time

isRunning = True
def stop():
    global isRunning
    isRunning = False

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

remoteConnection = dai.RemoteConnection()

def testService(input):
    print("Test service called with input:", input)
    return {"result": "testService result"}

remoteConnection.registerService("myService", testService)
while isRunning:
    time.sleep(1)
