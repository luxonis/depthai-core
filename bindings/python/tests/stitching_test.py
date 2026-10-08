import depthai as dai


def test_stitching_api_and_input_calibration_configuration():
    assert dai.node.Stitching.Properties is dai.StitchingProperties
    if hasattr(dai, "beta"):
        assert not hasattr(dai.beta.node, "Stitching")

    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        stitching = pipeline.create(dai.node.Stitching).build(2)

        assert isinstance(stitching, dai.node.Stitching)
        assert stitching.getNumInputs() == 2
        assert stitching.getMode() == dai.node.Stitching.Mode.PANORAMA
        assert stitching.getUseInputCalibration()

        stitching.setUseInputCalibration(False)
        assert not stitching.getUseInputCalibration()
