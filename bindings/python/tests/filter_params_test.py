import depthai as dai
import pytest


@pytest.mark.parametrize("name", ["MEDIAN_OFF", "KERNEL_3x3", "KERNEL_5x5", "KERNEL_7x7"])
def test_median_filter_str(name):
    # A custom __str__ that formats itself recursed without end with pybind11 v3.
    value = getattr(dai.filters.params.MedianFilter, name)
    assert str(value) == f"MedianFilter.{name}"
    assert f"{value}" == f"MedianFilter.{name}"


def test_image_filters_median_filter_params_str():
    assert str(dai.node.ImageFilters.MedianFilterParams.KERNEL_3x3) == "MedianFilter.KERNEL_3x3"
