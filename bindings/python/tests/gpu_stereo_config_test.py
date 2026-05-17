import depthai as dai


def test_gpu_stereo_config_is_confidence_only():
    cfg = dai.GPUStereoConfig()

    assert hasattr(cfg, "confidenceThreshold")
    cfg.confidenceThreshold = 25
    assert cfg.confidenceThreshold == 25


def test_gpu_stereo_initial_config_exposes_only_confidence():
    pipeline = dai.Pipeline()
    gpu = pipeline.create(dai.node.GPUStereo)

    assert hasattr(gpu, "initialConfig")
    assert hasattr(gpu.initialConfig, "confidenceThreshold")

    # Default propagated from config type
    assert gpu.initialConfig.confidenceThreshold == 10

    gpu.initialConfig.confidenceThreshold = 42
    assert gpu.initialConfig.confidenceThreshold == 42
