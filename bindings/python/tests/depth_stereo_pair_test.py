"""Device test for Depth stereo-pair selection through the Python API."""

import depthai as dai
import pytest


def test_depth_stereo_pair_selection_and_reset():
    with dai.Device() as device:
        pairs = device.getStereoPairs()
        if not pairs:
            pytest.skip("Requires a device with a stereo pair")
        selected = pairs[-1]
        pipeline = dai.Pipeline(device)
        depth = pipeline.create(dai.node.Depth)
        assert depth.getRequestedStereoPair() is None
        assert depth.getResolvedStereoPair() is None
        assert depth.setStereoPair(selected) is depth
        requested = depth.getRequestedStereoPair()
        assert (requested.left, requested.right) == (selected.left, selected.right)
        assert depth.getResolvedStereoPair() is None

        depth.setStereoPair(None)
        assert depth.getRequestedStereoPair() is None
        depth.setStereoPair(selected)
        invalid = dai.StereoPair()
        invalid.left = selected.left
        invalid.right = selected.left
        with pytest.raises(RuntimeError, match="two distinct, explicit camera sockets"):
            depth.setStereoPair(invalid)

        depth.build(
            dai.node.Depth.Algorithm.STEREO,
            dai.node.StereoDepth.PresetMode.DEFAULT,
            fps=15,
            size=(640, 400),
        )
        assert depth.depth is not None
        resolved = depth.getResolvedStereoPair()
        assert (resolved.left, resolved.right) == (selected.left, selected.right)
        assert resolved.baseline == selected.baseline
        assert resolved.isVertical == selected.isVertical
        for pair in (selected, None):
            with pytest.raises(RuntimeError, match="before the graph is wired"):
                depth.setStereoPair(pair)
