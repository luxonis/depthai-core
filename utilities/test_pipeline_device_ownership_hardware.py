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

    def check_created_device_lifetime(self, factory):
        pipeline = factory()
        device = pipeline.getDefaultDevice()
        try:
            self.assertEqual(device.getDeviceId(), self.device_id)
            if device.getPlatform() == self.dai.Platform.RVC2:
                with self.assertRaisesRegex(RuntimeError, "requires RVC4 firmware"):
                    self.stream_camera(pipeline)
            else:
                self.stream_camera(pipeline)
                pipeline.stop()
                pipeline.wait()
                self.assertFalse(device.isPipelineRunning())
            self.assertFalse(device.isClosed())
            del pipeline
            gc.collect()
            self.assertFalse(device.isClosed())
            self.assertEqual(device.getDeviceId(), self.device_id)
            if device.getPlatform() == self.dai.Platform.RVC4:
                with self.dai.Pipeline(device) as pipeline:
                    self.stream_camera(pipeline)
        finally:
            device.close()

    def test_implicit_device_survives_stop_and_pipeline_destruction(self):
        self.check_created_device_lifetime(self.dai.Pipeline)

    def test_added_device_survives_stop_and_pipeline_destruction(self):
        for from_info in (False, True):
            with self.subTest(from_info=from_info):
                def factory():
                    pipeline = self.dai.Pipeline(False)
                    argument = self.dai.DeviceInfo(self.device_id) if from_info else self.device_id
                    pipeline.addDevice(argument)
                    return pipeline
                self.check_created_device_lifetime(factory)

    def test_last_device_reference_is_released_with_pipeline(self):
        def run():
            with self.dai.Pipeline() as pipeline:
                if pipeline.getDefaultDevice().getPlatform() == self.dai.Platform.RVC4:
                    self.stream_camera(pipeline)
        run()
        gc.collect()
        # Reopening the same device requires the previous connection to have closed.
        with self.dai.Device(self.dai.DeviceInfo(self.device_id)) as device:
            self.assertEqual(device.getDeviceId(), self.device_id)

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
