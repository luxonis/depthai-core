"""Device test for Depth stereo-pair selection through the Python API."""

import re
import sys

import depthai as dai

SKIP_RETURN_CODE = 77


class _Raises:
    def __init__(self, match):
        self.match = match

    def __enter__(self):
        return self

    def __exit__(self, excType, exc, tb):
        assert excType is not None and issubclass(excType, RuntimeError), f"expected RuntimeError matching {self.match!r}"
        assert re.search(self.match, str(exc)), f"{str(exc)!r} does not match {self.match!r}"
        return True


def test_depth_stereo_pair_selection_and_reset():
    with dai.Device() as device:
        pairs = device.getStereoPairs()
        if not pairs:
            print("SKIP: requires a device with a stereo pair")
            sys.exit(SKIP_RETURN_CODE)
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
        with _Raises("two distinct, explicit camera sockets"):
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
            with _Raises("before the graph is wired"):
                depth.setStereoPair(pair)


if __name__ == "__main__":
    test_depth_stereo_pair_selection_and_reset()
    print("PASS")
