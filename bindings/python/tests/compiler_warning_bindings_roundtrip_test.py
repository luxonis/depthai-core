import depthai as dai
import pytest


def normalize(value):
    if value is None or isinstance(value, (bool, int, str, float)):
        return value

    if isinstance(value, list):
        return [normalize(item) for item in value]

    if isinstance(value, tuple):
        return tuple(normalize(item) for item in value)

    if isinstance(value, dai.Point2f):
        return {"x": value.x, "y": value.y}

    if isinstance(value, dai.Point3f):
        return {"x": value.x, "y": value.y, "z": value.z}

    if isinstance(value, dai.Rect):
        return {"x": value.x, "y": value.y, "width": value.width, "height": value.height}

    if isinstance(value, dai.MemoryInfo):
        return {"remaining": value.remaining, "used": value.used, "total": value.total}

    if isinstance(value, dai.CpuUsage):
        return {"average": value.average, "msTime": value.msTime}

    if isinstance(value, dai.ChipTemperatureRVC4):
        return {
            "cpuss": value.cpuss,
            "gpuss": value.gpuss,
            "mdmss": value.mdmss,
            "video": value.video,
            "ddr": value.ddr,
            "camera": value.camera,
            "average": value.average,
        }

    if isinstance(value, dai.SpatialLocationCalculatorConfigThresholds):
        return {"lowerThreshold": value.lowerThreshold, "upperThreshold": value.upperThreshold}

    if isinstance(value, dai.SpatialLocationCalculatorConfigData):
        return {
            "roi": normalize(value.roi),
            "depthThresholds": normalize(value.depthThresholds),
            "calculationAlgorithm": str(value.calculationAlgorithm),
        }

    if isinstance(value, dai.AprilTag):
        return {
            "id": value.id,
            "hamming": value.hamming,
            "decisionMargin": value.decisionMargin,
            "topLeft": normalize(value.topLeft),
            "topRight": normalize(value.topRight),
            "bottomRight": normalize(value.bottomRight),
            "bottomLeft": normalize(value.bottomLeft),
        }

    if isinstance(value, dai.IMUReportAccelerometer):
        return {"x": value.x, "y": value.y, "z": value.z}

    if isinstance(value, dai.IMUReportGyroscope):
        return {"x": value.x, "y": value.y, "z": value.z}

    if isinstance(value, dai.IMUReportMagneticField):
        return {"x": value.x, "y": value.y, "z": value.z}

    if isinstance(value, dai.IMUReportRotationVectorWAcc):
        return {
            "i": value.i,
            "j": value.j,
            "k": value.k,
            "real": value.real,
            "rotationVectorAccuracy": value.rotationVectorAccuracy,
        }

    if isinstance(value, dai.IMUPacket):
        return {
            "acceleroMeter": normalize(value.acceleroMeter),
            "gyroscope": normalize(value.gyroscope),
            "magneticField": normalize(value.magneticField),
            "rotationVector": normalize(value.rotationVector),
        }

    if isinstance(value, dai.ImgDetection):
        return {
            "label": value.label,
            "labelName": value.labelName,
            "confidence": value.confidence,
            "xmin": value.xmin,
            "ymin": value.ymin,
            "xmax": value.xmax,
            "ymax": value.ymax,
        }

    if isinstance(value, dai.SpatialImgDetection):
        return {
            "label": value.label,
            "labelName": value.labelName,
            "confidence": value.confidence,
            "xmin": value.xmin,
            "ymin": value.ymin,
            "xmax": value.xmax,
            "ymax": value.ymax,
            "spatialCoordinates": normalize(value.spatialCoordinates),
            "boundingBoxMapping": normalize(value.boundingBoxMapping),
        }

    if isinstance(value, dai.SpatialLocations):
        return {
            "config": normalize(value.config),
            "depthAverage": value.depthAverage,
            "depthMode": value.depthMode,
            "depthMedian": value.depthMedian,
            "depthMin": value.depthMin,
            "depthMax": value.depthMax,
            "depthAveragePixelCount": value.depthAveragePixelCount,
            "spatialCoordinates": normalize(value.spatialCoordinates),
        }

    if isinstance(value, dai.TrackedFeature):
        return {
            "position": normalize(value.position),
            "id": value.id,
            "age": value.age,
            "harrisScore": value.harrisScore,
            "trackingError": value.trackingError,
        }

    if isinstance(value, dai.Tracklet):
        return {
            "roi": normalize(value.roi),
            "id": value.id,
            "label": value.label,
            "age": value.age,
            "status": str(value.status),
            "srcImgDetection": normalize(value.srcImgDetection),
            "spatialCoordinates": normalize(value.spatialCoordinates),
            "velocity": normalize(value.velocity),
            "speed": normalize(value.speed),
        }

    raise TypeError(f"Unsupported value type for normalization: {type(value)!r}")

def test_april_tags_binding_roundtrip():
    april_tags = dai.AprilTags()
    expected_tags = []

    for seed in (1, 2):
        tag = dai.AprilTag()
        tag.id = 100 + seed
        tag.hamming = seed
        tag.decisionMargin = 1.5 * seed
        tag.topLeft = dai.Point2f(1.0 * seed, 2.0 * seed)
        tag.topRight = dai.Point2f(3.0 * seed, 2.0 * seed)
        tag.bottomRight = dai.Point2f(3.0 * seed, 4.0 * seed)
        tag.bottomLeft = dai.Point2f(1.0 * seed, 4.0 * seed)
        expected_tags.append(tag)

    april_tags.aprilTags = expected_tags

    assert normalize(april_tags.aprilTags) == normalize(expected_tags)


def test_benchmark_report_binding_readonly():
    benchmark_report = dai.BenchmarkReport()

    assert benchmark_report.fps == 0.0
    assert benchmark_report.timeTotal == 0.0
    assert benchmark_report.numMessagesReceived == 0.0
    assert benchmark_report.averageLatency == 0.0
    assert benchmark_report.latencies == []

    with pytest.raises((AttributeError, TypeError)):
        benchmark_report.fps = 123.0
    with pytest.raises((AttributeError, TypeError)):
        benchmark_report.timeTotal = 456.0
    with pytest.raises((AttributeError, TypeError)):
        benchmark_report.numMessagesReceived = 789.0
    with pytest.raises((AttributeError, TypeError)):
        benchmark_report.averageLatency = 3.0
    with pytest.raises((AttributeError, TypeError)):
        benchmark_report.latencies = [1.0, 2.0]

    latencies = benchmark_report.latencies
    latencies.append(1.0)

    assert latencies == [1.0]
    assert benchmark_report.latencies == []

    latencies_again = benchmark_report.latencies
    assert latencies_again == []
    assert latencies_again != latencies


def test_imu_data_binding_roundtrip():
    imu_data = dai.IMUData()
    expected_packets = []

    for seed in (1, 2, 3):
        accel = dai.IMUReportAccelerometer()
        accel.x = 1.0 * seed
        accel.y = 2.0 * seed
        accel.z = 3.0 * seed

        gyro = dai.IMUReportGyroscope()
        gyro.x = 4.0 * seed
        gyro.y = 5.0 * seed
        gyro.z = 6.0 * seed

        magnetic = dai.IMUReportMagneticField()
        magnetic.x = 7.0 * seed
        magnetic.y = 8.0 * seed
        magnetic.z = 9.0 * seed

        rotation = dai.IMUReportRotationVectorWAcc()
        rotation.i = 0.25 * seed
        rotation.j = 0.5 * seed
        rotation.k = 0.75 * seed
        rotation.real = 1.0 * seed
        rotation.rotationVectorAccuracy = 1.25 * seed

        packet = dai.IMUPacket()
        packet.acceleroMeter = accel
        packet.gyroscope = gyro
        packet.magneticField = magnetic
        packet.rotationVector = rotation
        expected_packets.append(packet)

    imu_data.packets = expected_packets

    assert normalize(imu_data.packets) == normalize(expected_packets)


def test_img_detections_binding_roundtrip():
    img_detections = dai.ImgDetections()
    expected_detections = []

    for seed in (1, 2, 3):
        detection = dai.ImgDetection()
        detection.label = 10 + seed
        detection.labelName = f"img_label_{seed}"
        detection.confidence = 0.5 + (0.25 * seed)
        detection.xmin = 0.125 * seed
        detection.ymin = 0.0625 * seed
        detection.xmax = detection.xmin + 0.25
        detection.ymax = detection.ymin + 0.375
        expected_detections.append(detection)

    img_detections.detections = expected_detections
    img_detections.segmentationMaskWidth = 32
    img_detections.segmentationMaskHeight = 24

    assert normalize(img_detections.detections) == normalize(expected_detections)
    assert img_detections.segmentationMaskWidth == 32
    assert img_detections.segmentationMaskHeight == 24


def test_spatial_img_detections_binding_roundtrip():
    spatial_img_detections = dai.SpatialImgDetections()
    expected_detections = []

    for seed in (1, 2, 3):
        thresholds = dai.SpatialLocationCalculatorConfigThresholds()
        thresholds.lowerThreshold = 100 + seed
        thresholds.upperThreshold = 1000 + seed

        config = dai.SpatialLocationCalculatorConfigData()
        config.roi = dai.Rect(0.125 * seed, 0.0625 * seed, 0.25, 0.5)
        config.depthThresholds = thresholds
        config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN

        detection = dai.SpatialImgDetection()
        detection.label = 20 + seed
        detection.labelName = f"spatial_label_{seed}"
        detection.confidence = 0.5 + (0.125 * seed)
        detection.xmin = 0.0625 * seed
        detection.ymin = 0.125 * seed
        detection.xmax = detection.xmin + 0.375
        detection.ymax = detection.ymin + 0.25
        detection.spatialCoordinates = dai.Point3f(100.0 * seed, 200.0 * seed, 300.0 * seed)
        detection.boundingBoxMapping = config
        expected_detections.append(detection)

    spatial_img_detections.detections = expected_detections

    assert normalize(spatial_img_detections.detections) == normalize(expected_detections)


def test_spatial_location_calculator_data_binding_roundtrip():
    spatial_location_data = dai.SpatialLocationCalculatorData()
    expected_locations = []

    for seed in (1, 2):
        thresholds = dai.SpatialLocationCalculatorConfigThresholds()
        thresholds.lowerThreshold = 100 + seed
        thresholds.upperThreshold = 1000 + seed

        config = dai.SpatialLocationCalculatorConfigData()
        config.roi = dai.Rect(0.125 * seed, 0.0625 * seed, 0.25, 0.5)
        config.depthThresholds = thresholds
        config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN

        location = dai.SpatialLocations()
        location.config = config
        location.depthAverage = 1000.0 * seed
        location.depthMode = 900.0 * seed
        location.depthMedian = 950.0 * seed
        location.depthMin = 800 + seed
        location.depthMax = 1200 + seed
        location.depthAveragePixelCount = 50 + seed
        location.spatialCoordinates = dai.Point3f(10.0 * seed, 20.0 * seed, 30.0 * seed)
        expected_locations.append(location)

    spatial_location_data.spatialLocations = expected_locations

    assert normalize(spatial_location_data.spatialLocations) == normalize(expected_locations)


def test_system_information_rvc4_binding_roundtrip():
    system_information = dai.SystemInformationRVC4()
    expected_ddr = dai.MemoryInfo()
    expected_ddr.remaining = 4096
    expected_ddr.used = 2048
    expected_ddr.total = 6144

    expected_cpu_avg = dai.CpuUsage()
    expected_cpu_avg.average = 0.5
    expected_cpu_avg.msTime = 100

    expected_process_cpu_avg = dai.CpuUsage()
    expected_process_cpu_avg.average = 0.25
    expected_process_cpu_avg.msTime = 200

    expected_cpu_usages = []
    for average, ms_time in ((0.125, 10), (0.75, 20)):
        usage = dai.CpuUsage()
        usage.average = average
        usage.msTime = ms_time
        expected_cpu_usages.append(usage)

    expected_chip_temperature = dai.ChipTemperatureRVC4()
    expected_chip_temperature.cpuss = 40.0
    expected_chip_temperature.gpuss = 41.0
    expected_chip_temperature.mdmss = 42.0
    expected_chip_temperature.video = 43.0
    expected_chip_temperature.ddr = 44.0
    expected_chip_temperature.camera = 45.0
    expected_chip_temperature.average = 42.5

    system_information.ddrMemoryUsage = expected_ddr
    system_information.processMemoryUsage = 123456789
    system_information.cpuAvgUsage = expected_cpu_avg
    system_information.processCpuAvgUsage = expected_process_cpu_avg
    system_information.cpuUsages = expected_cpu_usages
    system_information.chipTemperature = expected_chip_temperature

    assert normalize(system_information.ddrMemoryUsage) == normalize(expected_ddr)
    assert system_information.processMemoryUsage == 123456789
    assert normalize(system_information.cpuAvgUsage) == normalize(expected_cpu_avg)
    assert normalize(system_information.processCpuAvgUsage) == normalize(expected_process_cpu_avg)
    assert normalize(system_information.cpuUsages) == normalize(expected_cpu_usages)
    assert normalize(system_information.chipTemperature) == normalize(expected_chip_temperature)


def test_tracked_features_binding_roundtrip():
    tracked_features = dai.TrackedFeatures()
    expected_features = []

    for seed in (1, 2):
        feature = dai.TrackedFeature()
        feature.position = dai.Point2f(10.0 * seed, 20.0 * seed)
        feature.id = 100 + seed
        feature.age = 200 + seed
        feature.harrisScore = 1.5 * seed
        feature.trackingError = 0.5 * seed
        expected_features.append(feature)

    tracked_features.trackedFeatures = expected_features

    assert normalize(tracked_features.trackedFeatures) == normalize(expected_features)


def test_tracklets_binding_roundtrip():
    tracklets = dai.Tracklets()
    expected_tracklets = []

    for seed in (1, 2):
        src_img_detection = dai.ImgDetection()
        src_img_detection.label = 10 + seed
        src_img_detection.labelName = f"img_label_{seed}"
        src_img_detection.confidence = 0.5 + (0.25 * seed)
        src_img_detection.xmin = 0.125 * seed
        src_img_detection.ymin = 0.0625 * seed
        src_img_detection.xmax = src_img_detection.xmin + 0.25
        src_img_detection.ymax = src_img_detection.ymin + 0.375

        tracklet = dai.Tracklet()
        tracklet.roi = dai.Rect(0.125 * seed, 0.25 * seed, 0.375, 0.5)
        tracklet.id = 300 + seed
        tracklet.label = 400 + seed
        tracklet.age = 500 + seed
        tracklet.status = dai.Tracklet.TrackingStatus.TRACKED
        tracklet.srcImgDetection = src_img_detection
        tracklet.spatialCoordinates = dai.Point3f(5.0 * seed, 6.0 * seed, 7.0 * seed)
        tracklet.velocity = dai.Point3f(1.0 * seed, 2.0 * seed, 3.0 * seed)
        tracklet.speed = 4.0 * seed
        expected_tracklets.append(tracklet)

    tracklets.tracklets = expected_tracklets

    assert normalize(tracklets.tracklets) == normalize(expected_tracklets)
