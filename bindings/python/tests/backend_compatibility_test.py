"""Keep the existing ImageManip enum registration when PointCloud reuses it."""

import pickle

import depthai as dai
import pytest


def test_image_manip_backend_public_identity_is_unchanged():
    backend = dai.ImageManipProperties.Backend
    assert backend.__module__ == "depthai"
    assert backend.__qualname__ == "ImageManipProperties.Backend"
    assert list(backend.__members__) == ["HW", "CPU", "GPU", "AUTO"]
    assert [int(value) for value in backend.__members__.values()] == [1, 0, 2, 3]
    assert dai.node.ImageManip.Backend is backend


def test_point_cloud_aliases_reuse_the_original_image_manip_enum():
    backend = dai.ImageManipProperties.Backend
    assert dai.Backend is backend
    assert dai.PointCloudProperties.Backend is backend
    assert dai.node.PointCloud.Backend is backend


@pytest.mark.parametrize("name", ["HW", "CPU", "GPU", "AUTO"])
def test_backend_pickle_preserves_the_original_import_path(name):
    value = dai.ImageManipProperties.Backend.__members__[name]
    data = pickle.dumps(value, protocol=4)
    assert b"ImageManipProperties.Backend" in data
    assert pickle.loads(data) == value


def test_original_image_manip_enum_works_with_both_nodes():
    with dai.Pipeline(False) as pipeline:
        image_manip = pipeline.create(dai.node.ImageManip)
        point_cloud = pipeline.create(dai.node.PointCloud)
        for backend in dai.ImageManipProperties.Backend.__members__.values():
            image_manip.setBackend(backend)
            if backend == dai.ImageManipProperties.Backend.HW:
                with pytest.raises(ValueError):
                    point_cloud.setBackend(backend)
            else:
                assert point_cloud.setBackend(backend) is point_cloud
                expected = dai.ImageManipProperties.Backend.GPU if backend == dai.ImageManipProperties.Backend.AUTO else backend
                assert point_cloud.getBackend() == expected
