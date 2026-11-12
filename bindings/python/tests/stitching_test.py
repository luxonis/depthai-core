import depthai as dai
import pytest


def test_stitching_api_and_input_calibration_configuration():
    assert dai.node.Stitching.Properties is dai.StitchingProperties

    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        stitching = pipeline.create(dai.node.Stitching).build(2)

        assert isinstance(stitching, dai.node.Stitching)
        assert stitching.getNumInputs() == 2
        assert stitching.getMode() == dai.node.Stitching.Mode.PANORAMA
        assert stitching.getUseInputCalibration()
        assert hasattr(stitching, "outSourceMasks")

        stitching.setUseInputCalibration(False)
        assert not stitching.getUseInputCalibration()


def test_stitching_beta_names_are_deprecated_aliases():
    if not hasattr(dai, "beta"):
        pytest.skip("depthai is built without the beta API")

    with pytest.warns(DeprecationWarning, match=r"depthai\.beta\.node\.Stitching is deprecated, use depthai\.node\.Stitching"):
        assert dai.beta.node.Stitching is dai.node.Stitching
    with pytest.warns(DeprecationWarning, match=r"use depthai\.StitchingProperties"):
        assert dai.beta.StitchingProperties is dai.StitchingProperties

    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        with pytest.warns(DeprecationWarning):
            stitching = pipeline.create(dai.beta.node.Stitching).build(2)
        assert isinstance(stitching, dai.node.Stitching)

    # Names that never existed still raise AttributeError
    assert not hasattr(dai.beta.node, "NoSuchNode")
    assert not hasattr(dai.beta, "NoSuchProperties")
