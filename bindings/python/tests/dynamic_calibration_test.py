import depthai as dai
import pytest

DCC = dai.DynamicCalibrationControl
Cmds = DCC.Commands


def test_performance_mode_enum_all_values():
    # Ensure all enum entries are present and usable
    modes = [
        DCC.PerformanceMode.DEFAULT,
        DCC.PerformanceMode.STATIC_SCENERY,
        DCC.PerformanceMode.OPTIMIZE_SPEED,
        DCC.PerformanceMode.OPTIMIZE_PERFORMANCE,
        DCC.PerformanceMode.SKIP_CHECKS,
    ]

    # Sanity: all are distinct
    assert len(set(modes)) == len(modes)

    # Sanity: they are actually enums (int-like)
    for mode in modes:
        assert isinstance(mode, DCC.PerformanceMode)


def test_command_calibrate_rw_and_owner_init():
    cmd = Cmds.Calibrate(force=True)
    assert cmd.force is True

    cmd.force = False
    assert cmd.force is False

    # Owner init from command
    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


def test_command_calibration_quality_rw_and_owner_init():
    cmd = Cmds.CalibrationQuality(force=True)
    assert cmd.force is True

    cmd.force = False
    assert cmd.force is False

    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


def test_command_start_calibration_rw_and_owner_init():
    cmd = Cmds.StartCalibration(loadImagePeriod=0.25, calibrationPeriod=3.0)
    assert cmd.loadImagePeriod == 0.25
    assert cmd.calibrationPeriod == 3.0

    cmd.loadImagePeriod = 0.5
    cmd.calibrationPeriod = 5.0
    assert cmd.loadImagePeriod == 0.5
    assert cmd.calibrationPeriod == 5.0

    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


def test_command_stop_calibration_and_owner_init():
    cmd = Cmds.StopCalibration()
    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


def test_command_load_image_and_owner_init():
    cmd = Cmds.LoadImage()
    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


def test_command_apply_calibration_rw_and_owner_init():
    # Default handler is enough for binding smoke-test
    handler = dai.CalibrationHandler()
    cmd = Cmds.ApplyCalibration(handler)

    another_handler = dai.CalibrationHandler()
    cmd.calibration = another_handler

    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


def test_command_reset_data_and_owner_init():
    cmd = Cmds.ResetData()
    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


def test_command_set_performance_mode_rw_and_owner_init():
    cmd = Cmds.SetPerformanceMode(DCC.PerformanceMode.DEFAULT)
    assert cmd.performanceMode == DCC.PerformanceMode.DEFAULT

    cmd.performanceMode = DCC.PerformanceMode.OPTIMIZE_PERFORMANCE
    assert cmd.performanceMode == DCC.PerformanceMode.OPTIMIZE_PERFORMANCE

    ctrl = DCC(cmd)
    assert isinstance(ctrl, DCC)


# -------- Static factory helpers on the owner type --------


def test_static_factory_calibrate_all_args():
    c1 = DCC.calibrate()
    c2 = DCC.calibrate(force=True)
    assert isinstance(c1, DCC)
    assert isinstance(c2, DCC)


def test_static_factory_calibration_quality_all_args():
    c1 = DCC.calibrationQuality()
    c2 = DCC.calibrationQuality(force=True)
    assert isinstance(c1, DCC)
    assert isinstance(c2, DCC)


def test_static_factory_start_calibration_all_args():
    c1 = DCC.startCalibration()  # defaults
    c2 = DCC.startCalibration(loadImagePeriod=0.1, calibrationPeriod=1.5)
    assert isinstance(c1, DCC)
    assert isinstance(c2, DCC)


def test_static_factory_stop_calibration():
    c = DCC.stopCalibration()
    assert isinstance(c, DCC)


def test_static_factory_load_image():
    c = DCC.loadImage()
    assert isinstance(c, DCC)


def test_static_factory_apply_calibration():
    handler = dai.CalibrationHandler()
    c = DCC.applyCalibration(handler)
    assert isinstance(c, DCC)


def test_static_factory_reset_data():
    c = DCC.resetData()
    assert isinstance(c, DCC)


def test_static_factory_set_performance_mode_all_modes():
    # Default arg
    c_default = DCC.setPerformanceMode()
    assert isinstance(c_default, DCC)

    # Explicit modes
    for mode in [
        DCC.PerformanceMode.DEFAULT,
        DCC.PerformanceMode.STATIC_SCENERY,
        DCC.PerformanceMode.OPTIMIZE_SPEED,
        DCC.PerformanceMode.OPTIMIZE_PERFORMANCE,
        DCC.PerformanceMode.SKIP_CHECKS,
    ]:
        c = DCC.setPerformanceMode(mode)
        assert isinstance(c, DCC)


def test_dynamic_calibration_node_create():
    pipeline = dai.Pipeline()
    node = pipeline.create(dai.node.DynamicCalibration)

    # Basic API
    assert hasattr(node, "qualityOutput")
    assert hasattr(node, "coverageOutput")
    assert hasattr(node, "calibrationOutput")
    assert hasattr(node, "inputControl")
    assert hasattr(node, "left")
    assert hasattr(node, "right")
    assert hasattr(node, "setRunOnHost")

    # Check method execution
    node.setRunOnHost(True)
    assert node.runOnHost() is True

def test_coverage_data_read_write():
    d = dai.CoverageData()

    d.coveragePerCellA = [[1.0], [2.0], [3.0]]
    d.coveragePerCellB = [[4.0], [5.0], [6.0]]
    d.meanCoverage = 0.42
    d.dataAcquired = 1.0
    d.coverageAcquired = 1.0

    assert d.coveragePerCellA == [[1.0], [2.0], [3.0]]
    assert d.coveragePerCellB == [[4.0], [5.0], [6.0]]
    assert d.meanCoverage == pytest.approx(0.42)
    assert d.dataAcquired == pytest.approx(1.0)
    assert d.coverageAcquired == pytest.approx(1.0)

def test_calibration_quality_data_rw():
    q = dai.CalibrationQualityData()

    q.rotationChange = [0.1, 0.2, 0.3]
    q.depthErrorDifference = [1.0, 2.0]
    q.sampsonErrorCurrent = 0.5
    q.sampsonErrorNew = 0.25

    assert list(q.rotationChange) == pytest.approx([0.1, 0.2, 0.3])
    assert q.depthErrorDifference == pytest.approx([1.0, 2.0])
    assert q.sampsonErrorCurrent == pytest.approx(0.5)
    assert q.sampsonErrorNew == pytest.approx(0.25)

def test_calibration_quality_default():
    q = dai.CalibrationQuality()
    assert q.info == ""
    assert q.qualityData is None

def test_calibration_quality_with_data():
    data = dai.CalibrationQualityData()
    data.rotationChange = [0.1, 0.2, 0.3]

    q = dai.CalibrationQuality(data, "ok")
    assert q.info == "ok"
    assert q.qualityData.rotationChange == pytest.approx([0.1, 0.2, 0.3])

def test_calibration_quality_info_only():
    q = dai.CalibrationQuality("just-info")
    assert q.info == "just-info"
    assert q.qualityData is None

def test_calibration_quality_optional_modification():
    q = dai.CalibrationQuality()
    assert q.qualityData is None

    # Assign new data
    data = dai.CalibrationQualityData()
    data.sampsonErrorCurrent = 1.0
    q.qualityData = data

    assert q.qualityData.sampsonErrorCurrent == pytest.approx(1.0)

def test_dynamic_calibration_result_data_rw():
    r = dai.DynamicCalibrationResultData()

    new_calib = dai.CalibrationHandler()
    curr_calib = dai.CalibrationHandler()

    r.newCalibration = new_calib
    r.currentCalibration = curr_calib

    diff = dai.CalibrationQualityData()
    diff.sampsonErrorNew = 1.1
    r.calibrationDifference = diff

    assert isinstance(r.newCalibration, dai.CalibrationHandler)
    assert isinstance(r.currentCalibration, dai.CalibrationHandler)
    assert r.calibrationDifference.sampsonErrorNew == pytest.approx(1.1)

def test_dynamic_calibration_result_default():
    r = dai.DynamicCalibrationResult()
    assert r.info == ""
    assert r.calibrationData is None

def test_dynamic_calibration_result_info_only():
    r = dai.DynamicCalibrationResult("status")
    assert r.info == "status"
    assert r.calibrationData is None

def test_dynamic_calibration_result_with_data():
    data = dai.DynamicCalibrationResultData()
    data.newCalibration = dai.CalibrationHandler()

    r = dai.DynamicCalibrationResult(data, "ok")
    assert r.info == "ok"
    assert isinstance(r.calibrationData.newCalibration, dai.CalibrationHandler)

def test_dynamic_calibration_result_optional_assignment():
    r = dai.DynamicCalibrationResult()

    data = dai.DynamicCalibrationResultData()
    data.currentCalibration = dai.CalibrationHandler()

    r.calibrationData = data

    assert isinstance(r.calibrationData.currentCalibration, dai.CalibrationHandler)
