import os
import time
from hil_framework.lib_testbed.config.Config import Config
from hil_framework.lib_testbed.camera.base.AdbCapableBase import AdbCapableBase
from hil_framework.lib_testbed.utils.Testbed import Testbed

def enable_ptp(camera: AdbCapableBase, isMaster: bool):
    role = 'master' if isMaster else 'slave'
    ret = camera.adb_command([f'luxonis-ptp-config mode {role}'], capture_output=True, check=True)
    if ret.returncode != 0:
        raise RuntimeError(f'Failed to set PTP role on {camera.adb_id}, error: {ret.stderr}')
    
    ret = camera.adb_command(['luxonis-ptp-config enable'], capture_output=True, check=True)
    if ret.returncode != 0:
        raise RuntimeError(f'Failed to enable PTP on {camera.adb_id}, error: {ret.stderr}')

def main():
    testbed_name = os.environ.get('HIL_TESTBED')
    config = Config(testbed_name)
    testbed = Testbed(config)
    
    cameras = [cam for cam in testbed.cameras if cam.platform == 'rvc4']
    print(f'Found {len(cameras)} cameras, enabling PTP on them')
    for idx, camera in enumerate(cameras):
        isMaster = idx == 0
        assert isinstance(camera, AdbCapableBase)
        enable_ptp(camera, isMaster)

    # Wait for a minute for PTP to sync
    print('Waiting for PTP to sync')
    time.sleep(60)

if __name__ == '__main__':
    main()