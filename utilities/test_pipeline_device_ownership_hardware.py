"""Run with DEPTHAI_TEST_DEVICE=<id> and DEPTHAI_DEVICE_ID_LIST=<same id>."""

import gc
import os
import time
import unittest


@unittest.skipUnless(os.environ.get("DEPTHAI_TEST_DEVICE"), "Requires a selected hardware device")
class DeviceOwnershipTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import depthai as dai
        cls.dai = dai
        cls.device_id = os.environ["DEPTHAI_TEST_DEVICE"]

    def stream_camera(self, pipeline):
        dai = self.dai
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=15)
        queue = camera.requestOutput((640, 400), fps=15).createOutputQueue(maxSize=4, blocking=False)
        pipeline.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if queue.tryGet() is not None:
                return queue
            time.sleep(0.005)
        self.fail("No camera output received")

    def assert_closed_and_rebooted(self, device):
        self.assertTrue(device.isClosed())
        if device.getPlatform() == self.dai.Platform.RVC2:
            available = {info.getDeviceId(): info for info in self.dai.Device.getAllAvailableDevices()}
            self.assertIn(self.device_id, available)
            self.assertIn(available[self.device_id].state, (
                self.dai.XLinkDeviceState.X_LINK_UNBOOTED,
                self.dai.XLinkDeviceState.X_LINK_BOOTLOADER,
            ))

    def test_implicit_device_closes_on_stop(self):
        with self.dai.Pipeline() as pipeline:
            device = pipeline.getDefaultDevice()
            try:
                self.assertEqual(device.getDeviceId(), self.device_id)
                self.stream_camera(pipeline)
                pipeline.stop()
                pipeline.wait()
                self.assert_closed_and_rebooted(device)
            finally:
                device.close()

    def test_device_created_from_info_closes_on_stop(self):
        with self.dai.Pipeline(False) as pipeline:
            device = pipeline.addDevice(self.dai.DeviceInfo(self.device_id))
            try:
                self.stream_camera(pipeline)
                pipeline.stop()
                pipeline.wait()
                self.assert_closed_and_rebooted(device)
            finally:
                device.close()

    def test_caller_owned_device_is_preserved(self):
        with self.dai.Device(self.dai.DeviceInfo(self.device_id)) as device:
            with self.dai.Pipeline(device) as pipeline:
                if device.getPlatform() == self.dai.Platform.RVC2:
                    with self.assertRaisesRegex(RuntimeError, "requires RVC4 firmware"):
                        self.stream_camera(pipeline)
                else:
                    self.stream_camera(pipeline)
                    pipeline.stop()
                    pipeline.wait()
                    self.assertFalse(device.isPipelineRunning())
                self.assertFalse(device.isClosed())

    def test_collecting_stopped_pipelines_preserves_active_stream(self):
        with self.dai.Device(self.dai.DeviceInfo(self.device_id)) as device:
            if device.getPlatform() != self.dai.Platform.RVC4:
                self.skipTest("Reusing one open connection requires RVC4")
            stopped = []
            for _ in range(3):
                pipeline = self.dai.Pipeline(device)
                self.stream_camera(pipeline)
                pipeline.stop()
                pipeline.wait()
                stopped.append(pipeline)
            with self.dai.Pipeline(device) as pipeline:
                queue = self.stream_camera(pipeline)
                stopped.clear()
                gc.collect()
                count = 0
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    count += queue.tryGet() is not None
                    time.sleep(0.005)
                self.assertGreaterEqual(count, 10, "Collecting old pipelines interrupted camera output")
                self.assertFalse(device.isClosed())


if __name__ == "__main__":
    unittest.main()
