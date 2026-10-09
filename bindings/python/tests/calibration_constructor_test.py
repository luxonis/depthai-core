import json
import struct

import depthai as dai
import pytest


@pytest.mark.parametrize("from_file", [False, True])
def test_optional_validation_preserves_calibration(tmp_path, from_file):
    camera = dai.CameraInfo()
    camera.extrinsics.toCameraSocket = dai.CameraBoardSocket.CAM_B
    eeprom = dai.EepromData()
    eeprom.productName = "binding regression"
    eeprom.cameraData = {dai.CameraBoardSocket.CAM_A: camera}
    source = eeprom
    if from_file:
        source = tmp_path / "calibration.json"
        dai.CalibrationHandler(eeprom, False).eepromToJsonFile(source)

    for kwargs in ({}, {"validateExtrinsics": None}, {"validateExtrinsics": False}):
        calibration = dai.CalibrationHandler(source, **kwargs)
        assert calibration.getEepromData().productName == "binding regression"
        with pytest.raises(RuntimeError, match="Dangling extrinsic reference"):
            calibration.validateCalibrationHandler()

    with pytest.raises(RuntimeError, match="Dangling extrinsic reference"):
        dai.CalibrationHandler(source, validateExtrinsics=True)


def test_legacy_constructor_accepts_optional_validation(tmp_path):
    calibration_path = tmp_path / "calibration.bin"
    calibration_path.write_bytes(struct.pack("=111f", *([0.0] * 111)))
    board_path = tmp_path / "board.json"
    board_path.write_text(json.dumps({"board_config": {
        "name": "binding regression",
        "revision": "R1",
        "swap_left_and_right_cameras": False,
        "left_fov_deg": 70,
        "rgb_fov_deg": 80,
        "left_to_right_distance_cm": 7.5,
        "left_to_rgb_distance_cm": 3.75,
    }}))

    for kwargs in ({}, {"validateExtrinsics": None}, {"validateExtrinsics": False}):
        calibration = dai.CalibrationHandler(calibration_path, board_path, **kwargs)
        assert calibration.getEepromData().boardName == "binding regression"
        assert calibration.getSourceWidth(dai.CameraBoardSocket.CAM_A) == 1920
