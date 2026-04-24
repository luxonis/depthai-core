from pathlib import Path

import depthai as dai


def test_board_config_ffc_fsync_gpio_roundtrip():
    cfg = dai.BoardConfig()

    assert cfg.ffcFsyncGeneratorGpio is None

    cfg.ffcFsyncGeneratorGpio = 40
    assert cfg.ffcFsyncGeneratorGpio == 40

    cfg.ffcFsyncGeneratorGpio = None
    assert cfg.ffcFsyncGeneratorGpio is None


def test_device_config_board_ffc_fsync_gpio_roundtrip():
    cfg = dai.Device.Config()

    assert cfg.board.ffcFsyncGeneratorGpio is None

    cfg.board.ffcFsyncGeneratorGpio = 40
    assert cfg.board.ffcFsyncGeneratorGpio == 40


def test_pipeline_device_config_preserves_ffc_fsync_gpio():
    pipeline = dai.Pipeline(False)
    board_cfg = dai.BoardConfig()
    board_cfg.ffcFsyncGeneratorGpio = 40

    pipeline.setBoardConfig(board_cfg)

    device_cfg = pipeline.getDeviceConfig()
    assert device_cfg.board.ffcFsyncGeneratorGpio == 40


def test_tof_ffc4p_example_uses_explicit_device_boot():
    example = (
        Path(__file__).resolve().parents[3]
        / "examples"
        / "python"
        / "RVC2"
        / "ToF"
        / "tof_ffc4p_r5_internal_trigger.py"
    ).read_text()

    assert "device_config = dai.Device.Config()" in example
    assert "device_config.board.ffcFsyncGeneratorGpio = BOARD_FSYNC_GPIO" in example
    assert "with dai.Device(device_config) as device:" in example
    assert "pipeline = dai.Pipeline(device)" in example
