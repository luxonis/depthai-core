import depthai as dai
import pytest


def temporal_config(bits=3):
    config = dai.StereoDepthConfig()
    config.setSubpixel(True).setSubpixelFractionalBits(bits)
    config.setMedianFilter(dai.StereoDepthConfig.MedianFilter.MEDIAN_OFF)
    config.postProcessing.temporalFilter.enable = True
    config.postProcessing.spatialFilter.enable = False
    config.postProcessing.speckleFilter.enable = False
    config.postProcessing.decimationFilter.decimationFactor = 1
    return config


@pytest.mark.parametrize("bits,scale,maximum", [(3, 80, 7600), (4, 80, 7600), (5, 64, 6080)])
def test_disparity_encoding(bits, scale, maximum):
    config = temporal_config(bits)
    assert config.getDisparityScale() == scale
    assert config.getMaxDisparity() == maximum


def test_terminal_median_restores_subpixel_units():
    config = temporal_config()
    config.setMedianFilter(dai.StereoDepthConfig.MedianFilter.KERNEL_7x7)
    filter_type = dai.StereoDepthConfig.PostProcessing.Filter
    config.postProcessing.filteringOrder = [filter_type.TEMPORAL, filter_type.MEDIAN] + [filter_type.NONE] * 3
    assert config.getDisparityScale() == 8
    assert config.getMaxDisparity() == 760


def test_companding_has_no_linear_scale():
    config = temporal_config()
    config.costMatching.enableCompanding = True
    with pytest.raises(ValueError, match="Companded disparity"):
        config.getDisparityScale()
