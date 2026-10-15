"""Opt-in hardware roundtrips for newly exposed message metadata.

Set DEPTHAI_BINDINGS_DEVICE_TESTS=1 and select a device using
DEPTHAI_DEVICE_ID_LIST. Run on both RVC2 and RVC4 when available.
"""
from datetime import timedelta
import os
import threading

import depthai as dai
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("DEPTHAI_BINDINGS_DEVICE_TESTS") != "1",
    reason="Set DEPTHAI_BINDINGS_DEVICE_TESTS=1 to run on hardware",
)


@pytest.fixture
def device_roundtrip(request):
    with dai.Device() as device, dai.Pipeline(device) as pipeline:
        kind = request.node.callspec.params["kind"]
        if device.getPlatform() == dai.Platform.RVC2 and kind in ("vpp", "neural_depth"):
            pytest.skip(f"RVC2 firmware does not support {kind} messages")
        sync = pipeline.create(dai.node.Sync)
        sync.setRunOnHost(False)
        input_queue = sync.inputs["message"].createInputQueue()
        output_queue = sync.out.createOutputQueue()
        pipeline.start()
        yield input_queue, output_queue, device.getPlatform()


@pytest.mark.parametrize("kind", ["tracker", "vpp", "neural_depth", "events", "encoded", "frame", "thermal", "control", "tensor", "mask", "map"])
def test_message_metadata_roundtrip_on_device(device_roundtrip, kind):
    if kind == "tensor":
        message = dai.NNData()
        message.addTensor("values", np.arange(16, dtype=np.int32))
    elif kind == "mask":
        message = dai.SegmentationMask(list(range(12)), 4, 3)
    elif kind == "map":
        if not hasattr(dai, "beta"):
            pytest.skip("Requires beta bindings")
        message = dai.beta.Map2D()
        message.setMap(np.arange(12, dtype=np.float32).reshape(3, 4))
    elif kind == "tracker":
        message = dai.ObjectTrackerConfig().forceRemoveIDs([7, 11])
    elif kind == "vpp":
        message = dai.VppConfig()
        message.setBlending(0.25)
        message.injectionParameters.setKernelSize(7)
    elif kind == "neural_depth":
        message = dai.NeuralDepthConfig()
        message.algorithmControl.depthUnit = dai.DepthUnit.CUSTOM
        message.algorithmControl.customDepthUnitMultiplier = 250.0
    elif kind == "events":
        message = dai.PipelineEventAggregationConfig()
        node = dai.NodeEventAggregationConfig()
        node.nodeId = 7
        node.inputs = ["left"]
        message.nodes = [node]
        message.repeatIntervalSeconds = 2
    elif kind == "encoded":
        message = dai.EncodedFrame()
        message.setInstanceNum(7)
        message.setSize(4, 3)
        message.cam.sensitivityIso = 400
        message.frameOffset = 3
        message.frameSize = 4
    elif kind == "thermal":
        message = dai.ThermalConfig()
        message.imageParams.orientation = dai.ThermalImageOrientation.MirrorFlip
    elif kind == "control":
        message = dai.CameraControl()
        message.setManualExposure(1000, 200)
        message.setAutoFocusRegion(1, 2, 30, 40)
    else:
        message = dai.ImgFrame()
        message.setSize(4, 3)
        message.setType(dai.ImgFrame.Type.GRAY8)
        message.setData(list(range(12)))
        message.cam.sensitivityIso = 400
    message.setSequenceNum(42)
    message.setTimestamp(dai.Clock.now())
    input_queue, output_queue, platform = device_roundtrip
    input_queue.send(message)
    group = output_queue.get(timedelta(seconds=10))
    assert group is not None
    received = group["message"]
    assert type(received) is type(message)
    # These configuration types serialize only their configuration fields.
    if kind not in ("vpp", "neural_depth", "thermal", "control") and not (kind == "tracker" and platform == dai.Platform.RVC2):
        assert received.getSequenceNum() == 42
    assert received.getDatatype() == message.getDatatype()
    if kind == "tensor":
        assert list(received.getTensor("values").ravel()) == list(range(16))
    elif kind == "mask":
        assert list(received.getMaskData().ravel()) == list(range(12))
    elif kind == "map":
        assert list(received.getMap().ravel()) == list(range(12))
    elif kind == "tracker":
        assert list(received.trackletIdsToRemove) == [7, 11]
    elif kind == "vpp":
        assert received.getBlending() == 0.25
        assert received.injectionParameters.getKernelSize() == 7
    elif kind == "neural_depth":
        assert received.getDepthUnit() == dai.DepthUnit.CUSTOM
        assert received.getCustomDepthUnitMultiplier() == 250.0
    elif kind == "events":
        assert received.nodes[0].nodeId == 7
        assert list(received.nodes[0].inputs) == ["left"]
        assert received.repeatIntervalSeconds == 2
    elif kind == "thermal":
        assert received.imageParams.orientation == dai.ThermalImageOrientation.MirrorFlip
    elif kind == "control":
        assert received.expManual.exposureTimeUs == 1000
        assert received.expManual.sensitivityIso == 200
        assert received.afRegion.width == 30
    else:
        assert received.cam.sensitivityIso == 400
        assert (received.getWidth(), received.getHeight()) == (4, 3)
        if kind == "encoded":
            assert received.frameOffset == 3
            assert received.frameSize == 4
        else:
            assert list(received.getFrame().ravel()) == list(range(12))


def test_device_calibration_queries():
    with dai.Device() as device:
        calibration = device.tryGetCalibration()
        available = device.isCalibrationAvailable()
        assert isinstance(available, bool)
        if available:
            assert isinstance(calibration, dai.CalibrationHandler)


def test_runtime_calibration_overloads():
    with dai.Device() as device:
        original = device.tryGetCalibration()
        if original is None:
            pytest.skip("Requires device calibration")
        try:
            device.setCalibration(eepromData=original.getEepromData())
            assert device.getCalibration().getEepromData().version == original.getEepromData().version
            device.setCalibration(eepromData=None)
        finally:
            device.setCalibration(original)
        assert device.isCalibrationAvailable()


@pytest.mark.skipif(not hasattr(dai.node, "AutoCalibration"), reason="Dynamic calibration is disabled")
def test_auto_calibration_execution_target():
    with dai.Device() as device, dai.Pipeline(device) as pipeline:
        node = pipeline.create(dai.node.AutoCalibration)
        node.setRunOnHost(True)
        assert node.runOnHost()
        node.setRunOnHost(False)
        assert not node.runOnHost()


def test_model_zoo_shave_count_overload():
    with dai.Device() as device, dai.Pipeline(device) as pipeline:
        network = pipeline.create(dai.node.DetectionNetwork)
        description = dai.NNModelDescription("yolov6-nano")
        if device.getPlatform() != dai.Platform.RVC2:
            with pytest.raises(RuntimeError, match="not SUPERBLOB"):
                network.setFromModelZoo(description, numShaves=6, useCached=True)
            return
        network.setFromModelZoo(description, numShaves=6, useCached=True)
        assert network.getClasses()
        assets = network.neuralNetwork.getAssetManager().getAll()
        assert len(assets) == 1
        assert dai.OpenVINO.Blob(assets[0].getData()).numShaves == 6


def test_color_camera_scaled_size_validation():
    with dai.Device() as device, dai.Pipeline(device) as pipeline:
        camera = pipeline.create(dai.node.ColorCamera)
        assert camera.getScaledSize(640, 1, 2) == 320
        assert camera.getScaledSize(641, 1, 2) == 321
        with pytest.raises(ValueError, match="denominator must not be zero"):
            camera.getScaledSize(640, 1, 0)
        assert camera.getScaledSize(2**31 - 1, 2, 2) == 2**31 - 1
        with pytest.raises(OverflowError, match="does not fit"):
            camera.getScaledSize(2**31 - 1, 2, 1)


def test_camera_requests_and_debugging():
    with dai.Device() as device, dai.Pipeline(device) as pipeline:
        pipeline.enablePipelineDebugging()
        camera = pipeline.create(dai.node.Camera).build()
        assert camera.getMaxWidth() > 0
        assert camera.getMaxHeight() > 0
        output = camera.requestOutput((320, 240), fps=10)
        assert camera.getMaxRequestedWidth() >= 320
        assert camera.getMaxRequestedHeight() >= 240
        assert camera.getMaxRequestedFps() >= 10
        queue = output.createOutputQueue()
        pipeline.start()
        frame = queue.get(timedelta(seconds=10))
        assert frame is not None
        assert (frame.getWidth(), frame.getHeight()) == (320, 240)

        state_api = pipeline.getPipelineState()
        events = state_api.nodes(camera.id).events()
        assert all(isinstance(event, dai.NodeState.DurationEvent) for event in events)
        states = []
        ready = threading.Event()

        def on_state(state):
            states.append(state)
            ready.set()

        state_api.stateAsync(on_state)
        assert ready.wait(10)
        assert camera.id in states[0].nodeStates
        # A synchronous query must let the asynchronous callback acquire the GIL.
        pipeline.getPipelineStateOut().tryGetAll()
        assert camera.id in state_api.nodes().detailed().nodeStates
        pipeline.getPipelineStateOut().tryGetAll()
        assert isinstance(state_api.nodes(camera.id).summary(), dai.NodeState)


@pytest.mark.parametrize("source", ["list", "bytes", "bytearray", "path"])
def test_custom_model_assets(tmp_path, source):
    data = b"custom model bytes"
    model = {"list": list(data), "bytes": data, "bytearray": bytearray(data)}.get(source)
    if source == "path":
        model = tmp_path / "network.dlc"
        model.write_bytes(data)
    with dai.Device() as device, dai.Pipeline(device) as pipeline:
        network = pipeline.create(dai.node.NeuralNetwork)
        network.setOtherModelFormat(model)
        assets = network.getAssetManager().getAll()
        assert len(assets) == 1
        assert bytes(assets[0].getData()) == data
