import depthai as dai
import contextlib
from typing import Optional, Dict
from datetime import timedelta, datetime
import signal
import time
import threading
import cv2

# This example only works on devices that are connected with M8 cables with FSYNC Y splitters

deviceInfos = dai.Device.getAllAvailableDevices()
targetFps = 30
resolution = (640, 480)
syncThresholdSec = 1e-3  # 1ms
running = True

def interruptHandler(sig, frame):
    global running
    if running:
        print("Interrupted! Exiting...")
        running = False
    else:
        print("Exiting now!")
        exit(0)

signal.signal(signal.SIGINT, interruptHandler)

def getDeviceName(device : dai.Device) -> str:
    info = device.getDeviceInfo()
    name = info.deviceId
    if info.name is not None and info.name != "":
        name += "[" + info.name + "]"
    return name

with contextlib.ExitStack() as stack:
    # Variables to keep track of master and slave pipelines and outputs
    masterPipeline: Optional[dai.Pipeline] = None
    masterNode: Optional[Dict[str, dai.Node.Output]] = None
    masterName: Optional[str] = None

    slavePipelines: Dict[str, dai.Pipeline] = {}
    slaveQueues: Dict[str, Dict[str, dai.MessageQueue]] = {}

    # keep track of sync node inputs for slaves
    inputQueues = {}

    # keep track of all sync node output names
    outputNames = []

    for deviceInfo in deviceInfos:
        # Create pipeline for each device
        devicePipeline = stack.enter_context(dai.Pipeline(dai.Device(deviceInfo)))
        device = devicePipeline.getDefaultDevice()
        deviceName = getDeviceName(device)
        fsyncRole = device.getExternalFrameSyncRole()

        for socket in device.getConnectedCameras():
            # create a queue for each camera on the device
            if fsyncRole == dai.ExternalFrameSyncRole.MASTER:
                cam = devicePipeline.create(dai.node.Camera).build(socket, sensorFps=targetFps)
            else:
                # slaves will lock to the master's FPS
                cam = devicePipeline.create(dai.node.Camera).build(socket)
            outputNode = cam.requestOutput(resolution, dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP)

            # Master cameras will be linked to the sync node directly
            if fsyncRole == dai.ExternalFrameSyncRole.MASTER:
                if masterNode is None:
                    masterNode = {}
                
                masterNode[socket.name] = outputNode
            
            # Gather all slave camera outputs
            elif fsyncRole == dai.ExternalFrameSyncRole.SLAVE:
                if slaveQueues.get(deviceName) is None:
                    slaveQueues[deviceName] = {}
                slaveQueues[deviceName][socket.name] = outputNode.createOutputQueue()
            
        if fsyncRole == dai.ExternalFrameSyncRole.MASTER:
            device.setExternalStrobeEnable(True)
            print(f"{device.getDeviceId()} is master")

            if masterPipeline is not None:
                raise RuntimeError("Only one master pipeline is supported")
            
            masterPipeline = devicePipeline
            masterName = deviceName
        elif fsyncRole == dai.ExternalFrameSyncRole.SLAVE:
            slavePipelines[deviceName] = devicePipeline
            print(f"{device.getDeviceId()} is slave")

    if masterPipeline is None or masterNode is None:
        raise RuntimeError("No master detected!")

    if len(slavePipelines) < 1:
        raise RuntimeError("No slaves detected!")

    # Create sync node
    syncNode = masterPipeline.create(dai.node.Sync)

    # Sync node will run on the host, since it needs to sync multiple devices
    syncNode.setRunOnHost(True)
    # group frames into pairs that are within 1/2 frame period
    syncNode.setSyncThreshold(timedelta(milliseconds=1000 / (2 * targetFps)))

    # Link master camera outputs to the sync node
    for socketName, camOutput in masterNode.items():
        name = f"master_{masterName}_{socketName}"
        camOutput.link(syncNode.inputs[name])
        outputNames.append(name)

    # For slaves, we must create an input queue for each output
    # We will then manually forward the frames from each input queue to the output queue
    # This is because slave devices have separate pipelines from the master
    for deviceName, sockets in slaveQueues.items():
        for socketName, _ in sockets.items():
            name = f"slave_{deviceName}_{socketName}"
            outputNames.append(name)
            input_queue = syncNode.inputs[name].createInputQueue()
            inputQueues[name] = input_queue

    syncedGroups = syncNode.out.createOutputQueue()

    # thread worker for forwarding slave queues to sync node
    def data_collector(deviceName, socketName):
        # Send frames from slave output queues to sync node input queues
        camOutputQueue = slaveQueues[deviceName][socketName]
        while running:
            if camOutputQueue.has():
                inputQueues[f"slave_{deviceName}_{socketName}"].send(camOutputQueue.get())
            else:
                time.sleep(0.001)

    # Start pipelines
    masterPipeline.start()
    for _, slavePipeline in slavePipelines.items():
        slavePipeline.start()

    # Start threads
    threads = {}
    for deviceName, sockets in slaveQueues.items():
        for socketName, camOutputQueue in sockets.items():
            threads[f"slave_{deviceName}_{socketName}"] = threading.Thread(target=data_collector, args=(deviceName, socketName))
            threads[f"slave_{deviceName}_{socketName}"].start()

    # main display loop
    latestFrameGroup = None
    while running:
        # Get frames from sync node output queue
        while syncedGroups.has():
            latestFrameGroup = syncedGroups.get()

        if latestFrameGroup is not None and latestFrameGroup.getNumMessages() == len(outputNames):
            tsValues = {}
            for name in outputNames:
                tsValues[name] = latestFrameGroup[name].getTimestamp(dai.CameraExposureOffset.END).total_seconds()

            delta = max(tsValues.values()) - min(tsValues.values())
            syncStatus = abs(delta) < syncThresholdSec

            if not syncStatus:
                print(f"Sync error: Sync lost, threshold exceeded {delta * 1e6} us")
                continue

            for outputName in outputNames:
                msg = latestFrameGroup[outputName]
                frame = msg.getCvFrame()
                cv2.imshow(f"synced_view_{outputName}", frame)
            
            latestFrameGroup = None  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            running = False
            break
    
    for t in threads.keys():
        threads[t].join()
    cv2.destroyAllWindows()