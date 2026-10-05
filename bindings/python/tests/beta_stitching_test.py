import depthai as dai


def test_stitching_beta_api_and_input_calibration_configuration():
    assert not hasattr(dai.node, "Stitching")
    assert dai.beta.node.Stitching.Properties is dai.beta.StitchingProperties

    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        stitching = pipeline.create(dai.beta.node.Stitching).build(2)

        assert isinstance(stitching, dai.beta.node.Stitching)
        assert stitching.getNumInputs() == 2
        assert stitching.getMode() == dai.beta.node.Stitching.Mode.PANORAMA
        assert not stitching.getUseInputCalibration()

        stitching.setUseInputCalibration(True)
        assert stitching.getUseInputCalibration()
