import depthai as dai
from math import isclose


def test_auto_calibration_config_bindings():
    cfg = dai.AutoCalibrationConfig()

    assert hasattr(cfg, "mode")
    assert hasattr(cfg, "sleepingTime")
    assert hasattr(cfg, "calibrationConfidenceThreshold")
    assert hasattr(cfg, "dataConfidenceThreshold")
    assert hasattr(cfg, "maxIterations")
    assert hasattr(cfg, "maxImagesPerRecalibration")
    assert hasattr(cfg, "flashCalibration")
    assert hasattr(cfg, "validationSetSize")

    assert cfg.mode == dai.AutoCalibrationConfig.Mode.ON_START
    assert cfg.sleepingTime == 30
    assert isclose(cfg.calibrationConfidenceThreshold, 0.9)
    assert isclose(cfg.dataConfidenceThreshold, 0.7)
    assert cfg.maxIterations == 5
    assert cfg.maxImagesPerRecalibration == 10
    assert cfg.flashCalibration is True
    assert cfg.validationSetSize == 5

    cfg.mode = dai.AutoCalibrationConfig.Mode.CONTINUOUS
    cfg.sleepingTime = 12
    cfg.calibrationConfidenceThreshold = 0.95
    cfg.dataConfidenceThreshold = 0.75
    cfg.maxIterations = 8
    cfg.flashCalibration = False
    cfg.validationSetSize = 6

    assert cfg.mode == dai.AutoCalibrationConfig.Mode.CONTINUOUS
    assert cfg.sleepingTime == 12
    assert isclose(cfg.calibrationConfidenceThreshold, 0.95)
    assert isclose(cfg.dataConfidenceThreshold, 0.75)
    assert cfg.maxIterations == 8
    cfg.maxImagesPerRecalibration = 7
    assert cfg.flashCalibration is False
    assert cfg.validationSetSize == 6
    assert cfg.maxImagesPerRecalibration == 7

    assert not hasattr(cfg, "maxImagesPerReacalibration")
