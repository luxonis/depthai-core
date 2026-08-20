import depthai as dai


def test_stitching_is_exposed_only_through_beta_api():
    assert not hasattr(dai.node, "Stitching")
    assert dai.beta.node.Stitching.Properties is dai.beta.StitchingProperties

    pipeline = dai.Pipeline()
    stitching = pipeline.create(dai.beta.node.Stitching).build(2)

    assert isinstance(stitching, dai.beta.node.Stitching)
    assert stitching.getNumInputs() == 2
    assert stitching.getMode() == dai.beta.node.Stitching.Mode.PANORAMA
