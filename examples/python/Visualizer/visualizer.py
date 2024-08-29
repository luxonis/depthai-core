import depthai as dai
import time

remoteConnector = dai.RemoteConnector()
remoteConnector.initServer()

while True:
    time.sleep(1)
    print("Still alive")