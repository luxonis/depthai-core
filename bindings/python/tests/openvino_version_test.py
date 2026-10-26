import depthai as dai
import pytest


@pytest.mark.parametrize("method_name", ["getBlobSupportedVersions", "getBlobLatestSupportedVersion"])
@pytest.mark.parametrize("version", [(2021, 4), (0, 0)])
def test_blob_version_keyword_arguments(method_name, version):
    method = getattr(dai.OpenVINO, method_name)
    major, minor = version
    expected = method(major, minor)
    assert method(majorVersion=major, minorVersion=minor) == expected
    assert method(major, minorVersion=minor) == expected


def test_blob_version_lookup():
    assert dai.OpenVINO.getBlobVersion(majorVersion=2021, minorVersion=4) == dai.OpenVINO.Version.VERSION_2021_4
    with pytest.raises(IndexError):
        dai.OpenVINO.getBlobVersion(0, 0)
