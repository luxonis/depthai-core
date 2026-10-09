"""Exercise Python access to the public C++ APIs covered by the binding audit."""
from datetime import timedelta
import subprocess
import sys

import depthai as dai
import numpy as np
import pytest


def test_cpp_default_arguments():
    frame = dai.ImgFrame()
    frame.setType(dai.ImgFrame.Type.GRAY8)
    frame.setSize(8, 6)
    assert frame.getPlaneStride() == frame.getPlaneStride(0)
    control = dai.CameraControl()
    control.setStrobeSensor()
    assert control.strobeConfig.activeLevel == 1
    control.setStrobeExternal(1)
    assert control.strobeConfig.activeLevel == 1
    control.setCommand(dai.CameraControl.Command.START_STREAM)
    assert control.getCommand(dai.CameraControl.Command.START_STREAM)
    config = dai.ImageManipConfig()
    config.addCrop(dai.Rect(0, 0, 8, 6))
    config.addCropRotatedRect(dai.RotatedRect(dai.Point2f(4, 3), dai.Size2f(8, 6), 0))
    points = [dai.Point2f(0, 0), dai.Point2f(8, 0), dai.Point2f(8, 6), dai.Point2f(0, 6)]
    config.addTransformFourPoints(points, points)
    assert config.setOutputCenter() is config
    spatial = dai.SpatialLocationCalculatorConfig()
    spatial.setDepthThresholds()
    assert spatial.globalLowerThreshold == 100
    assert spatial.globalUpperThreshold == 65535
    bootloader = dai.DeviceBootloader.Config()
    bootloader.setDnsIPv4("1.1.1.1")
    assert bootloader.getDnsIPv4() == "1.1.1.1"
    assert len(dai.DeviceBootloader.getEmbeddedBootloaderBinary()) > 0


def test_pipeline_schema_and_state_json():
    with dai.Pipeline(False) as pipeline:
        first = pipeline.create(dai.node.ImageManip)
        second = pipeline.create(dai.node.ImageManip)
        first.setRunOnHost()
        second.setRunOnHost()
        first.out.link(second.inputImage)
        schema = pipeline.getPipelineSchema()
        assert isinstance(schema, dai.PipelineSchema)
        assert schema.nodes[first.id].name == "ImageManip"
        assert schema.nodes[first.id].ioInfo[("", "out")].name == "out"
        assert any(c.node1Id == first.id and c.node2Id == second.id for c in schema.connections)
        assert isinstance(pipeline.getDevicePipelineSchema(), dai.PipelineSchema)
        assert isinstance(pipeline.serializeToJson(), dict)
    assert dai.PipelineState().toJson() == {"nodeStates": []}


def test_device_properties_copy():
    source = dai.DeviceProperties()
    source.xlinkChunkSize = 2048
    source.eepromId = 17
    source.calibData = dai.EepromData()
    target = dai.DeviceProperties()
    assert target.setFrom(source) is target
    assert target.xlinkChunkSize == 2048
    assert target.eepromId == 17
    target.setFrom(dai.DeviceProperties())
    assert target.xlinkChunkSize == -1
    assert target.eepromId == 17
    assert target.calibData is not None


@pytest.mark.skipif(not hasattr(dai.node, "AutoCalibration"), reason="Dynamic calibration is disabled")
def test_auto_calibration_requires_device():
    subprocess.run([sys.executable, "-c", """
import depthai as dai
with dai.Pipeline(False) as pipeline:
    try:
        pipeline.create(dai.node.AutoCalibration)
    except ValueError as error:
        assert "requires a device" in str(error)
    else:
        raise AssertionError("Expected missing-device validation")
"""], check=True, capture_output=True, text=True, timeout=15)


@pytest.mark.parametrize("tracklet", [False, True])
def test_spatial_transform_defaults_to_millimeters(tracklet):
    source = dai.ImgTransformation(640, 480)
    target = dai.ImgTransformation(640, 480)
    target.addScale(0.5, 0.5)
    box = dai.RotatedRect(dai.Point2f(160, 120), dai.Size2f(80, 60), 0)
    if tracklet:
        detection = dai.Tracklet()
        detection.roi = dai.Rect(120, 90, 80, 60)
        detection.srcImgDetection = dai.ImgDetection(box, 0.8, 1)
    else:
        detection = dai.SpatialImgDetection(box, dai.Point3f(0, 0, 0), 0.8, 1)
    detection.transform(source, target)
    result = detection.srcImgDetection.getBoundingBox() if tracklet else detection.getBoundingBox()
    assert (result.center.x, result.center.y) == pytest.approx((80, 60))


def test_object_tracker_config_is_registered_and_mutable():
    config = dai.ObjectTrackerConfig()
    assert config.forceRemoveID(3) is config
    assert config.forceRemoveIDs([7, 11]) is config
    assert list(config.trackletIdsToRemove) == [3, 7, 11]
    assert config.getDatatype() == dai.DatatypeEnum.ObjectTrackerConfig
    queue = dai.MessageQueue()
    queue.send(config)
    assert list(queue.get().trackletIdsToRemove) == [3, 7, 11]


@pytest.mark.parametrize("factory, datatype", [
    (dai.Buffer, dai.DatatypeEnum.Buffer),
    (dai.ImgFrame, dai.DatatypeEnum.ImgFrame),
    (dai.EncodedFrame, dai.DatatypeEnum.EncodedFrame),
    (dai.GateControl, dai.DatatypeEnum.GateControl),
    (dai.PipelineEventAggregationConfig, dai.DatatypeEnum.PipelineEventAggregationConfig),
])
def test_virtual_datatype_dispatch(factory, datatype):
    assert factory().getDatatype() == datatype


@pytest.mark.parametrize("field, value", [
    ("blending", 0.25), ("distanceGamma", 0.75), ("maxPatchSize", 7),
    ("uniformPatch", False), ("maxNumThreads", 4), ("maxFPS", 25),
    ("patchColoringType", dai.VppConfig.PatchColoringType.MAXDIST),
])
def test_vpp_setters_update_fields(field, value):
    config = dai.VppConfig()
    suffix = field[0].upper() + field[1:]
    getattr(config, "set" + suffix)(value)
    assert getattr(config, field) == value
    assert getattr(config, "get" + suffix)() == value


@pytest.mark.parametrize("field, value, getter", [
    ("useInjection", False, "getUseInjection"), ("kernelSize", 7, "getKernelSize"),
    ("textureThreshold", 0.25, "getTextureThreshold"),
    ("confidenceThreshold", 0.75, "getConfidenceThreshold"),
    ("morphologyIterations", 2, "getMorphologyIterations"),
    ("useMorphology", False, "isUseMorphology"),
])
def test_vpp_injection_config(field, value, getter):
    params = dai.VppConfig.InjectionParameters()
    getattr(params, "set" + field[0].upper() + field[1:])(value)
    config = dai.VppConfig()
    config.setInjectionParameters(params)
    assert getattr(config.getInjectionParameters(), getter)() == value
    assert getattr(config.injectionParameters, field) == value


def test_neural_depth_algorithm_control():
    config = dai.NeuralDepthConfig()
    config.algorithmControl.depthUnit = dai.DepthUnit.CUSTOM
    config.algorithmControl.customDepthUnitMultiplier = 250.0
    assert config.getDepthUnit() == dai.DepthUnit.CUSTOM
    assert config.getCustomDepthUnitMultiplier() == 250.0
    config.setDepthUnit(dai.DepthUnit.METER)
    assert config.algorithmControl.depthUnit == dai.DepthUnit.METER


def test_encoded_frame_metadata():
    frame = dai.EncodedFrame()
    assert frame.setInstanceNum(instance=7) is frame
    frame.setSize(4, 3)
    frame.cam.exposureTimeUs = 1234
    frame.cam.sensitivityIso = 400
    frame.cam.fsync = dai.ImgFrame.Fsync.INPUT
    frame.frameOffset = 9
    frame.frameSize = 12
    assert frame.getInstanceNum() == 7
    assert frame.getExposureTime() == timedelta(microseconds=1234)
    assert frame.getSensitivity() == 400
    assert frame.getFsync() == dai.ImgFrame.Fsync.INPUT
    metadata = frame.getImgFrameMeta()
    assert metadata.getInstanceNum() == 7
    assert (metadata.getWidth(), metadata.getHeight()) == (4, 3)
    assert metadata.cam.sensitivityIso == 400


def test_imgframe_clone_and_metadata_copy():
    source = dai.ImgFrame()
    source.setSize(4, 3)
    source.setType(dai.ImgFrame.Type.GRAY8)
    source.setSequenceNum(42)
    source.setData(np.arange(12, dtype=np.uint8))
    clone = source.clone()
    assert clone.getSequenceNum() == 42
    np.testing.assert_array_equal(clone.getData(), source.getData())
    target = dai.ImgFrame()
    assert target.setMetadata(sourceFrame=source) is target
    assert (target.getWidth(), target.getHeight()) == (4, 3)
    assert target.copyDataFrom(sourceFrame=source) is target
    np.testing.assert_array_equal(target.getData(), source.getData())
    target.setSourceSize((8, 6))
    assert (target.sourceFb.width, target.sourceFb.height) == (8, 6)
    target.setSourceSize(16, 12)
    assert (target.sourceFb.width, target.sourceFb.height) == (16, 12)


def test_image_type_helpers():
    interleaved, planar = dai.ImgFrame.Type.RGB888i, dai.ImgFrame.Type.RGB888p
    assert dai.ImgFrame.isInterleaved(interleaved)
    assert not dai.ImgFrame.isInterleaved(planar)
    assert dai.ImgFrame.toPlanar(interleaved) == planar
    assert dai.ImgFrame.toInterleaved(planar) == interleaved
    assert dai.ImgFrame.typeToBpp(interleaved) == 3


def test_prepare_segmentation_mask_returns_writable_storage():
    mask = dai.SegmentationMask()
    data = mask.prepareMask(width=3, height=2)
    data[:] = [1, 1, 2, 2, 3, 3]
    np.testing.assert_array_equal(mask.getMaskData(), [[1, 1, 2], [2, 3, 3]])
    mask.setSize(width=2, height=3)
    assert (mask.getWidth(), mask.getHeight()) == (2, 3)


def test_pipeline_event_configuration_keeps_mutable_nodes():
    node = dai.NodeEventAggregationConfig()
    node.nodeId = 12
    node.inputs = ["left", "right"]
    node.events = True
    config = dai.PipelineEventAggregationConfig()
    config.nodes = [node]
    config.nodes[0].outputs = ["out"]
    config.repeatIntervalSeconds = 2
    assert list(config.nodes[0].inputs) == ["left", "right"]
    assert list(config.nodes[0].outputs) == ["out"]
    assert config.nodes[0].events
    config.nodes.append(node)
    assert len(config.nodes) == 2
    config.repeatIntervalSeconds = None
    assert config.repeatIntervalSeconds is None


def test_board_camera_and_imu_configuration():
    config = dai.BoardConfig()
    camera = dai.BoardConfig.Camera()
    camera.name = "color"
    camera.sensorType = dai.CameraSensorType.AUTO
    camera.orientation = dai.CameraImageOrientation.ROTATE_180_DEG
    config.camera = {dai.CameraBoardSocket.CAM_I: camera}
    config.imu = dai.BoardConfig.IMU()
    config.imu.bus = 2
    config.nonExclusiveMode = True
    assert config.camera[dai.CameraBoardSocket.CAM_I].name == "color"
    assert config.imu.bus == 2
    assert config.nonExclusiveMode
    config.imu = None
    assert config.imu is None


@pytest.mark.parametrize("factory, field, value", [
    (dai.CameraSensorConfig, "hdr", True), (dai.CameraSensorConfig, "hfr", True),
    (dai.CameraFeatures, "additionalNames", ["color", "left"]),
    (dai.CameraInfo, "lensPosition", 80),
    (dai.VideoEncoderProperties, "frameRate", 25.0),
    (dai.VideoEncoderProperties, "lossless", True),
    (dai.RectificationProperties, "enableRectification", False),
    (dai.ToFProperties, "fps", 15.0),
    (dai.ToFProperties, "cameraName", "tof"),
    (dai.ColorCameraProperties, "rawPacked", False),
    (dai.MonoCameraProperties, "cameraName", "left"),
    (dai.StereoDepthProperties, "enableFrameSync", False),
    (dai.ObjectTrackerProperties, "trackingPerClass", True),
    (dai.ObjectTrackerProperties, "trackletMaxLifespan", 15),
    (dai.ObjectTrackerProperties, "trackletBirthThreshold", 4),
    (dai.ObjectTrackerProperties, "occlusionRatioThreshold", 0.25),
    (dai.SpatialLocationCalculatorConfigData, "stepSize", 3),
    (dai.WarpProperties, "outputWidth", 320),
    (dai.WarpProperties, "outputHeight", 240),
    (dai.WarpProperties, "interpolation", dai.Interpolation.AUTO),
])
def test_configuration_field_roundtrip(factory, field, value):
    config = factory()
    setattr(config, field, value)
    result = getattr(config, field)
    assert list(result) == value if isinstance(value, list) else result == value


def test_detection_option_aliases():
    options = dai.DetectionParserOptions()
    options.nKeypoints = 17
    assert options.numKeypoints == 17
    options.outputNamesToUse = ["output"]
    assert list(options.outputNames) == ["output"]
    options.classNames = ["person", "car"]
    assert list(options.classNames) == ["person", "car"]


def test_transform_constructors():
    matrix = np.eye(4)
    matrix[:3, 3] = [1, 2, 3]
    transform = dai.Transform()
    transform.matrix = matrix.tolist()
    for message in (dai.TransformData(transform), dai.TransformData(matrix.tolist()),
                    dai.TransformData(1, 2, 3, 0, 0, 0, 1), dai.TransformData(1, 2, 3, 0, 0, 0)):
        translation = message.getTranslation()
        assert (translation.x, translation.y, translation.z) == (1, 2, 3)


def test_node_io_access_and_aliases():
    class Host(dai.node.ThreadedHostNode):
        def __init__(self):
            super().__init__()
            self.input = self.createInput("in")
            self.output = self.createOutput("out")

        def run(self):
            pass

    with dai.Pipeline(False) as pipeline:
        source = pipeline.create(Host)
        sink = pipeline.create(Host)
        source.setAlias("source")
        assert source.getAlias() == "source"
        output = source.output
        input_ = sink.input
        assert not input_.isConnected()
        output.link(input_)
        assert input_.isConnected()
        assert output.getType() == dai.Node.Output.Type.MSender
        assert source.getOutputRef("out").getName() == "out"
        assert sink.getInputRef("in").getName() == "in"
        assert len(pipeline.getConnections()) == 1
        output.unlink(input_)
        assert not input_.isConnected()


def test_lazy_assets(tmp_path):
    path = tmp_path / "asset.bin"
    path.write_bytes(b"depthai")
    manager = dai.AssetManager()
    manager.setRootPath("assets")
    asset = manager.setLazy(key="model", path=path)
    assert asset.getSize() == 7
    assert bytes(asset.getData()) == b"depthai"
    assert manager.getSerializedSize() >= 7
    assert manager.getRootPath() == "assets"
    asset.setData([1, 2, 3])
    assert bytes(asset.getData()) == b"\x01\x02\x03"


def test_get_layer_datatype_output_parameter():
    message = dai.NNData()
    message.addTensor("tensor", np.array([1, 2], dtype=np.int32))
    assert message.getLayerDatatype("tensor") == dai.TensorInfo.DataType.INT
    assert message.getLayerDatatype("missing") is None


def test_zero_timeout_queue_operations():
    queue = dai.MessageQueue(maxSize=1, blocking=True)
    message = dai.Buffer()
    message.setSequenceNum(42)
    zero = timedelta(0)
    assert queue.send(message, zero)
    assert not queue.send(dai.Buffer(), zero)
    assert dai.MessageQueue.waitAny([queue], zero)
    assert dai.MessageQueue.getAny({"queue": queue}, zero)["queue"].getSequenceNum() == 42
    assert not dai.MessageQueue.waitAny([queue], zero)
    assert dai.MessageQueue.getAny({"queue": queue}, zero) == {}
    assert queue.get(zero) is None
    queue.send(message)
    assert queue.get(zero).getSequenceNum() == 42


def test_asset_data_copy_survives_replacement():
    asset = dai.Asset("data")
    original = np.arange(64, dtype=np.uint8)
    asset.data = original
    asset.data[0] = 9
    original[0] = 9
    assert asset.data[0] == 9
    snapshot = asset.getData()
    asset.data = np.full(4096, 7, dtype=np.uint8)
    np.testing.assert_array_equal(snapshot, original)
    assert asset.data.shape == (4096,)


def test_image_coordinate_remapping():
    source = dai.ImgFrame()
    source.setSize(640, 480)
    source.setSourceSize(640, 480)
    target = dai.ImgFrame()
    target.setSize(320, 240)
    target.setSourceSize(640, 480)
    transform = dai.ImgTransformation(640, 480)
    transform.addScale(0.5, 0.5)
    target.setTransformation(transform)
    point = dai.Point2f(160, 120, False)
    remapped = target.remapPointFromSource(point)
    assert (remapped.x, remapped.y) == pytest.approx((80, 60))
    restored = target.remapPointToSource(remapped)
    assert (restored.x, restored.y) == pytest.approx((160, 120))
    between = dai.ImgFrame.remapPointBetweenFrames(point, source, target)
    assert (between.x, between.y) == pytest.approx((80, 60))


@pytest.mark.skipif(not hasattr(dai.node, "HostCamera"), reason="OpenCV support is disabled")
def test_host_camera_and_display_connections():
    with dai.Pipeline(False) as pipeline:
        camera = pipeline.create(dai.node.HostCamera)
        display = dai.node.Display("binding test")
        camera.out.link(display.input)
        assert display.input.isConnected()
        assert len(pipeline.getConnections()) == 1
        camera.out.unlink(display.input)
        assert not display.input.isConnected()


def test_prepare_mask_preserves_previous_allocation():
    mask = dai.SegmentationMask()
    previous = mask.prepareMask(2, 2)
    previous[:] = [1, 2, 3, 4]
    current = mask.prepareMask(1024, 1024)
    current[:] = 7
    np.testing.assert_array_equal(previous, [1, 2, 3, 4])
    assert mask.getMaskData()[0, 0] == 7

def test_record_configuration_enums():
    config = dai.RecordConfig()
    assert config.state == dai.RecordConfig.RecordReplayState.NONE
    assert config.compressionLevel == dai.RecordConfig.CompressionLevel.DEFAULT
    config.state = dai.RecordConfig.RecordReplayState.RECORD
    config.compressionLevel = dai.RecordConfig.CompressionLevel.FASTEST
    assert config.state == dai.RecordConfig.RecordReplayState.RECORD
    assert config.compressionLevel == dai.RecordConfig.CompressionLevel.FASTEST


def test_binary_asset_overloads():
    manager = dai.AssetManager()
    for value in ([1, 2, 3], b"\x01\x02\x03", bytearray([1, 2, 3])):
        # The data keyword distinguishes bytes from existing byte-string paths.
        asset = manager.set("binary", data=value)
        assert bytes(asset.getData()) == b"\x01\x02\x03"
        asset.setData(value)
        assert bytes(asset.data) == b"\x01\x02\x03"


def test_version_construction_and_comparison():
    prerelease = dai.Version(1, 2, 3, dai.Version.PreReleaseType.RC, 2, "build")
    release = dai.Version(1, 2, 3)
    assert prerelease.toStringSemver() == "1.2.3-rc.2"
    assert prerelease.getBuildInfo() == "build"
    assert prerelease < release
    assert prerelease <= release
    assert release >= prerelease
    assert release <= release
    assert release >= release
    assert release.toString() == str(release)
    assert dai.Version(1, 2, 3, "build").getBuildInfo() == "build"


def test_thermal_orientation():
    config = dai.ThermalConfig()
    assert config.imageParams.orientation is None
    config.imageParams.orientation = dai.ThermalImageOrientation.MirrorFlip
    assert config.imageParams.orientation == dai.ThermalImageOrientation.MirrorFlip


def test_color_camera_isp_scale():
    properties = dai.ColorCameraProperties()
    scale = dai.ColorCameraProperties.IspScale()
    scale.horizNumerator = 2
    scale.horizDenominator = 3
    scale.vertNumerator = 4
    scale.vertDenominator = 5
    properties.ispScale = scale
    assert properties.ispScale.horizNumerator == 2
    assert properties.ispScale.horizDenominator == 3
    assert properties.ispScale.vertNumerator == 4
    assert properties.ispScale.vertDenominator == 5


def test_camera_control_parameters():
    control = dai.CameraControl()
    control.setManualExposure(1000, 200)
    assert control.expManual.exposureTimeUs == 1000
    assert control.expManual.sensitivityIso == 200
    control.setAutoFocusRegion(1, 2, 30, 40)
    assert (control.afRegion.x, control.afRegion.y, control.afRegion.width, control.afRegion.height) == (1, 2, 30, 40)
    control.setAutoExposureRegion(5, 6, 70, 80)
    assert (control.aeRegion.x, control.aeRegion.y, control.aeRegion.width, control.aeRegion.height) == (5, 6, 70, 80)
    control.setStrobeExternal(3, 1)
    assert control.strobeConfig.enable == 1
    assert control.strobeConfig.activeLevel == 1
    assert control.strobeConfig.gpioNumber == 3
    control.strobeTimings.durationUs = 100
    assert control.strobeTimings.durationUs == 100
