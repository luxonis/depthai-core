"""Host-only checks: python3 -m unittest discover -s utilities -p 'test_pipeline_restart_stress.py'."""

from contextlib import redirect_stdout
from datetime import timedelta
import io
import random
import os
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock

import pipeline_restart_stress as stress


class CameraOutputsTest(unittest.TestCase):
    def test_feature_tracker_receives_hardware_aligned_dimensions(self):
        nodes = {name: MagicMock() for name in ("FeatureTracker", "ImageManip")}
        run = MagicMock()
        run.camera.return_value = (Mock(), Mock())
        run.create.side_effect = nodes.__getitem__
        stress.image_pipeline(run, "FeatureTracker")
        width, height = nodes["ImageManip"].initialConfig.setOutputSize.call_args.args
        self.assertEqual(width % 128, 0)
        self.assertEqual(height % 32, 0)
        self.assertLessEqual(width, 1920)
        self.assertLessEqual(height, 1080)

    def test_camera_builders_leave_no_unused_requested_outputs(self):
        class Camera:
            def __init__(self):
                self.outputs = []

            def build(self, *args, **kwargs):
                return self

            def requestOutput(self, *args, **kwargs):
                output = Mock()
                self.outputs.append(output)
                return output

            requestFullResolutionOutput = requestOutput

        builds = [
            (stress.inference_pipeline, (kind,))
            for kind in ("NeuralNetwork", "DetectionNetwork", "SegmentationParser",
                         "SpatialDetectionNetwork", "ObjectTracker", "Align")
        ] + [
            (stress.beta_parser_pipeline, ("ClassificationParser", "model")),
            (stress.rgbd_pipeline, ()),
            (stress.fusion_pipeline, ()),
        ]
        for factory, arguments in builds:
            with self.subTest(factory=factory.__name__, arguments=arguments):
                cameras = []

                def create(name):
                    if name == "Camera":
                        camera = Camera()
                        cameras.append(camera)
                        return camera
                    node = MagicMock()

                    def build(*inputs):
                        if name == "beta.ToFStereoFusion":
                            self.assertEqual(inputs[2], run.args.fps)
                        # SDK builders taking Camera nodes request and link their own outputs.
                        for source in inputs:
                            if isinstance(source, Camera):
                                source.requestOutput((640, 400)).link(node.input)
                        return node

                    node.build.side_effect = build
                    return node

                run = stress.Workload(
                    MagicMock(), Mock(), SimpleNamespace(fps=15),
                    {"camera_sockets": ["color"], "color_socket": "color",
                     "stereo_pair": ("left", "right")},
                    stress.Case("test", factory, (), ""),
                )
                run.create = create
                run.depth = Mock()
                factory(run, *arguments)
                self.assertTrue(cameras)
                for camera in cameras:
                    self.assertTrue(camera.outputs)
                    for output in camera.outputs:
                        self.assertTrue(output.link.called or output.createOutputQueue.called)


class DeviceCapabilitiesTest(unittest.TestCase):
    def inspect(self, *, stereo=False, tof=False, rvc4=True, models=()):
        dai = SimpleNamespace(
            Platform=SimpleNamespace(RVC4="RVC4"),
            CameraSensorType=SimpleNamespace(COLOR="COLOR", MONO="MONO", TOF="TOF"))
        device = Mock()
        device.getPlatform.return_value = "RVC4" if rvc4 else "RVC2"
        device.getConnectedCameraFeatures.return_value = [
            SimpleNamespace(socket="CAM_C", supportedTypes=["TOF"])] if tof else []
        device.getStereoPairs.return_value = [SimpleNamespace(left="CAM_A", right="CAM_B")] if stereo else []
        device.getConnectedIMU.return_value = "NONE"
        device.isNeuralDepthSupported.return_value = rvc4
        device.isGpuStereoSupported.return_value = rvc4
        device.getSupportedDeviceModels.return_value = models
        return stress.inspect_device(dai, device, "/tmp"), device

    def test_tof_only_device_does_not_advertise_stereo(self):
        state, device = self.inspect(tof=True)
        self.assertEqual(state["algorithms"], {"TOF"})
        device.getSupportedDeviceModels.assert_not_called()

    def test_stereo_models_and_tof_enable_matching_backends(self):
        state, _ = self.inspect(stereo=True, tof=True, models=["model"])
        self.assertEqual(state["algorithms"], {
            "STEREO", "NEURAL", "NEURAL_ASSISTED_STEREO", "GPU_STEREO", "TOF", "TOF_STEREO_FUSION"})

    def test_neural_models_are_required_for_neural_depth(self):
        state, _ = self.inspect(stereo=True)
        self.assertNotIn("NEURAL", state["algorithms"])
        self.assertIn("NEURAL_ASSISTED_STEREO", state["algorithms"])

    def test_rvc2_never_queries_rvc4_models(self):
        state, device = self.inspect(stereo=True, rvc4=False)
        self.assertEqual(state["algorithms"], {"STEREO"})
        device.getSupportedDeviceModels.assert_not_called()


class SchedulingTest(unittest.TestCase):
    def test_import_preserves_the_callers_firmware_overrides(self):
        env = dict(os.environ, DEPTHAI_DEVICE_RVC4_FWP="caller-rvc4.tar.xz", DEPTHAI_DEVICE_BINARY="caller-rvc2.mvcmd")
        subprocess.run(
            [sys.executable, "-c", "import os; import pipeline_restart_stress; "
             "assert os.environ['DEPTHAI_DEVICE_RVC4_FWP'] == 'caller-rvc4.tar.xz'; "
             "assert os.environ['DEPTHAI_DEVICE_BINARY'] == 'caller-rvc2.mvcmd'"],
            cwd=stress.Path(stress.__file__).parent, env=env, check=True,
        )

    def test_every_workload_runs_before_repeating_and_dependencies_come_first(self):
        cases = stress.cases()
        self.assertEqual(len(cases), len({case.name for case in cases}))
        pending, completed = list(cases), set()
        rng = random.Random(4)
        while pending:
            case = stress.next_case(pending, completed, rng)
            self.assertTrue(set(case.depends) <= completed)
            self.assertNotIn(case.name, completed)
            completed.add(case.name)
        self.assertEqual(completed, {case.name for case in cases})

    def test_inventory_covers_all_example_nodes_or_explains_exclusion(self):
        covered = {node for case in stress.cases() for node in case.nodes}
        self.assertGreater(len(stress.example_inventory()), 60)
        self.assertFalse(set(stress.example_inventory()) - covered - stress.EXCLUDED.keys())
        self.assertTrue({"Camera", "IMU", "Sync", "Vpp", "ToF", "NeuralDepth"} <= covered)

    def test_missing_dependency_does_not_spin(self):
        case = stress.Case("replay", Mock(), (), "", depends=("record",))
        with self.assertRaisesRegex(RuntimeError, "dependencies"):
            stress.next_case([case], set(), random.Random(1))


class LifecycleTest(unittest.TestCase):
    def setUp(self):
        self.pipeline = Mock()
        self.pipeline.getAllNodes.return_value = []
        self.pipeline.isRunning.return_value = True
        self.connection = object()
        self.device = Mock()
        self.device.isClosed.return_value = False
        self.device.isPipelineRunning.return_value = False
        self.device.getDeviceId.return_value = "device"
        self.dai = SimpleNamespace(Pipeline=Mock(return_value=self.pipeline))
        self.args = SimpleNamespace(runtime=0.001)
        self.state = {"runs": []}
        self.queue = Mock()
        self.queue.tryGet.return_value = object()

    def execute(self, build):
        case = stress.Case("test", build, (), "")
        with redirect_stdout(io.StringIO()):
            return stress.run_random_pipeline(self.device, case, self.args, self.state, self.dai)

    def build(self, run):
        run.streams.append(["image", self.queue, 0, None])

    def assert_stopped(self):
        self.pipeline.stop.assert_called_once()
        self.pipeline.wait.assert_called_once()

    def test_success_preserves_caller_device_and_records_output(self):
        self.execute(self.build)
        self.dai.Pipeline.assert_called_once_with(self.device)
        self.device.close.assert_not_called()
        self.assert_stopped()
        self.assertEqual(self.state["runs"][0]["status"], "passed")
        self.assertGreater(self.state["runs"][0]["outputs"]["image"], 0)

    def test_builder_failure_still_stops_and_waits(self):
        with self.assertRaisesRegex(ValueError, "building"):
            self.execute(Mock(side_effect=ValueError("building")))
        self.assert_stopped()
        self.assertEqual(self.state["runs"][0]["status"], "failed")

    def test_start_failure_still_stops_and_waits(self):
        self.pipeline.start.side_effect = RuntimeError("starting")
        with self.assertRaisesRegex(RuntimeError, "starting"):
            self.execute(self.build)
        self.assert_stopped()

    def test_missing_output_is_failure_not_coverage(self):
        self.queue.tryGet.return_value = None
        with self.assertRaisesRegex(AssertionError, "No output received"):
            self.execute(self.build)
        self.assert_stopped()
        self.assertEqual(self.state["runs"][0]["status"], "failed")

    def test_output_validation_failure_still_stops(self):
        def build(run):
            self.build(run)
            run.streams[0][3] = Mock(side_effect=AssertionError("bad timestamp"))

        with self.assertRaisesRegex(AssertionError, "bad timestamp"):
            self.execute(build)
        self.assert_stopped()

    def test_closed_connection_is_failure(self):
        self.device.isClosed.return_value = True
        with self.assertRaisesRegex(AssertionError, "closed the caller-owned"):
            self.execute(self.build)
        self.assertEqual(self.state["runs"][0]["status"], "failed")

    def test_changed_connection_is_failure(self):
        self.device.getDeviceId.side_effect = ["device", "other-device"]
        with self.assertRaisesRegex(AssertionError, "identity changed"):
            self.execute(self.build)

    def test_wait_runs_when_stop_throws(self):
        self.pipeline.stop.side_effect = RuntimeError("stopping")
        with self.assertRaisesRegex(RuntimeError, "stopping"):
            self.execute(self.build)
        self.assert_stopped()

    def test_configuration_cleanup_runs_when_stop_throws(self):
        cleanup = Mock()

        def build(run):
            self.build(run)
            run.cleanup.append(cleanup)

        self.pipeline.stop.side_effect = RuntimeError("stopping")
        with self.assertRaisesRegex(RuntimeError, "stopping"):
            self.execute(build)
        cleanup.assert_called_once()

    def test_interrupt_still_stops(self):
        self.queue.tryGet.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.execute(self.build)
        self.assert_stopped()


class FsyncTest(unittest.TestCase):
    def setUp(self):
        self.device = Mock()
        self.device.getExternalFrameSyncRole.return_value = "master"
        self.device.setExternalFrameSyncRole.return_value = (True, "")
        self.run = SimpleNamespace(
            device=self.device,
            dai=SimpleNamespace(
                ExternalFrameSyncRole=SimpleNamespace(MASTER="master", SLAVE="slave"),
                CameraControl=SimpleNamespace(FrameSyncMode=SimpleNamespace(INPUT="input")),
                ImgFrame=SimpleNamespace(Fsync=SimpleNamespace(INPUT="input"), Type=SimpleNamespace(NV12="nv12")),
                CameraExposureOffset=SimpleNamespace(END="end"),
            ),
            args=SimpleNamespace(fsync_role="MASTER", fps=20, fsync_settle=0,
                                 fsync_tolerance_ms=2, fsync_mean_tolerance_ms=1, fsync_min_samples=2),
            state={"fsync_sockets": ["left", "right"], "imu": False},
            cleanup=[], callbacks=[], start_actions=[], metrics={}, create=MagicMock(), watch=Mock(),
        )
        stress.fsync_pipeline(self.run)
        self.check = self.run.watch.call_args.args[2]

    def frame(self, seconds, fsync="input"):
        frame = Mock()
        frame.getFsync.return_value = fsync
        frame.getTimestamp.return_value = timedelta(seconds=seconds)
        return frame

    def group(self, seconds, skew=0.0001, fsync="input"):
        return {"0": self.frame(seconds, fsync), "1": self.frame(seconds + skew, fsync)}

    def test_correct_sync_and_rate_pass(self):
        self.check(self.group(1))
        self.check(self.group(1.05))
        self.run.callbacks[0]()
        self.assertAlmostEqual(self.run.metrics["max_skew_ms"], 0.1)
        self.assertAlmostEqual(self.run.metrics["measured_fps"], 20)

    def test_large_skew_fails(self):
        self.check(self.group(1, skew=0.004))
        self.check(self.group(1.05, skew=0.004))
        with self.assertRaisesRegex(AssertionError, "skew"):
            self.run.callbacks[0]()

    def test_isolated_jitter_within_existing_fsync_limits_passes(self):
        for index in range(10):
            self.check(self.group(1 + index * 0.05, skew=0.00137 if index == 9 else 0.0001))
        self.run.callbacks[0]()
        self.assertLess(self.run.metrics["mean_skew_ms"], 1)
        self.assertAlmostEqual(self.run.metrics["p99_skew_ms"], 1.37)

    def test_p99_skew_above_existing_limit_fails(self):
        for index in range(10):
            self.check(self.group(1 + index * 0.05, skew=0.003 if index == 9 else 0.0001))
        with self.assertRaisesRegex(AssertionError, "P99"):
            self.run.callbacks[0]()

    def test_missing_lock_fails(self):
        with self.assertRaisesRegex(AssertionError, "not locked"):
            self.check(self.group(1, fsync="none"))

    def test_wrong_frame_rate_fails(self):
        self.check(self.group(1))
        self.check(self.group(1.1))
        with self.assertRaisesRegex(AssertionError, "rate"):
            self.run.callbacks[0]()

    def test_insufficient_samples_fail(self):
        self.check(self.group(1))
        with self.assertRaisesRegex(AssertionError, "settled FSYNC groups"):
            self.run.callbacks[0]()

    def test_controller_is_restored(self):
        self.run.cleanup[0]()
        self.device.setExternalStrobeEnable.assert_called_with(False)
        self.device.setExternalFrameSyncRole.assert_called_with("master")


if __name__ == "__main__":
    unittest.main()
