import depthai as dai

def build_fn(p: dai.Pipeline):
    # Build a whole complicated pipeline here, for the example just build a camera node
    cam = p.create(dai.node.Camera).build()
    return cam
