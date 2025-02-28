import depthai as dai
import time
import datetime
DEBUG=True


# from lib_testbed.power.PowerFactory import PowerFactory
# from lib_testbed.config.Config import Config
# from lib_testbed.utils.ssh_util import *
# from time import sleep

# def powercycle():
#     testbed_name=os.getenv("SET_HIL_TESTBED")
#     config=Config(testbed_name)
#     print(config.cameras)

#     for camera in config.cameras:
#         if camera.platform == "rvc2":
#             print(camera.name)
#             power_control=PowerFactory.get_power_object_for_device(config, camera.name)
#             power_control[0].off()
#             sleep(2)
#             power_control[0].on()
#     for camera in config.cameras:
#         if camera.platform == "rvc4":
#             power_control=PowerFactory.get_power_object_for_device(config, camera.name)
#             if len(power_control) == 2:
#                 power_control[1].off()
#                 power_control[0].off()
#                 sleep(2)
#                 power_control[0].on()
#                 power_control[1].on()
#             elif len(power_control) == 1:
#                 power_control[0].off()
#                 sleep(2)
#                 power_control[0].on()
#             else:
#                 print("No devices available in power_control.")


def checkStreaming():
    tStart = time.time()
    device = dai.Device()
    if DEBUG: print("Time to create a device: ", time.time() - tStart)
    with dai.Pipeline(device) as pipeline:
        outputQueues = {}
        sockets = device.getConnectedCameras()
        for socket in sockets:
            cam = pipeline.create(dai.node.Camera).build(socket)
            outputQueues[str(socket)] = cam.requestFullResolutionOutput().createOutputQueue()

        pipeline.start()
        for name in outputQueues.keys():
            queue = outputQueues[name]
            videoIn = queue.get()
            assert isinstance(videoIn, dai.ImgFrame)
        tStop = time.time()
    print("Time to close the pipeline", time.time() - tStop)
    return time.time() - tStart


for i in range(10000):
    print(f"Checking stream for the {i+1} time")
    # powercycle()
    for i in range(5):
        t = checkStreaming()
        if t is None:
            print("Timed out!")
            exit(1)
    print("Whole loop took ", t)
