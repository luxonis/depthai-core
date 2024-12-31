#!/usr/bin/env python3
import depthai as dai
import time

# Create pipeline
with dai.Pipeline() as pipeline:
    # Create the nodes
    cam = pipeline.create(dai.node.Camera).build()
    benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
    # benchmarkIn.setRunOnHost(True) # The node can also run on host and include the transfer limitation, default is False
    output = cam.requestFullResolutionOutput()
    output.link(benchmarkIn.input)

    pipeline.start()
    while pipeline.isRunning():
        time.sleep(1) # Let the logger print out the FPS
