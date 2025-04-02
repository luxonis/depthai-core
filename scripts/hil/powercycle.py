from lib_testbed.power.PowerFactory import PowerFactory
from lib_testbed.config.Config import Config
from lib_testbed.utils.ssh_util import *
from time import sleep
import os


testbed_name=os.getenv("SET_HIL_TESTBED")
config=Config(testbed_name)
print(config.cameras)

for camera in config.cameras:
    if camera.platform == "rvc2":
        print(camera.name)
        power_control=PowerFactory.get_power_object_for_device(config, camera.name)
        power_control[0].off()
        sleep(2)
        power_control[0].on()
for camera in config.cameras:
    if camera.platform == "rvc4":
        power_control=PowerFactory.get_power_object_for_device(config, camera.name)
        if len(power_control) == 2:
            power_control[1].off()
            power_control[0].off()
            sleep(2)
            power_control[0].on()
            power_control[1].on()
        elif len(power_control) == 1:
            power_control[0].off()
            sleep(2)
            power_control[0].on()
        else:
            print("No devices available in power_control.")
