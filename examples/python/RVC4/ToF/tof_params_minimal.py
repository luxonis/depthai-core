#!/usr/bin/env python3
"""Minimal RVC4 ToF example with explicit parameter setup."""

import depthai as dai

# --- build parameters ---
BOARD_SOCKET = dai.CameraBoardSocket.AUTO
SENSOR_MODE = dai.ToFSensorMode.F1_FULL   # raw superframe: 1344 x 2420
FPS = 10.0
PRESET = dai.ToFPreset.MID_RANGE         # IPP phase-unwrap profile

with dai.Pipeline() as pipeline:
    tof = pipeline.create(dai.node.ToF)

    # build resolves socket; preset= applies IPP profile when explicitly passed
    tof.build(
        BOARD_SOCKET,
        sensorMode=SENSOR_MODE,
        fps=FPS,
        preset=PRESET,
    )

    # tune initialConfig after build() so values are not overwritten
    cfg = tof.initialConfig
    cfg.enableBilateralFilter = True
    cfg.enableTemporalNoiseReduction = True
    cfg.enableFlyingPixelCorrection = True
    cfg.enableRadialToPerp = False
    cfg.phaseUnwrapErrorThreshold = 75

    depth_q = tof.depth.createOutputQueue()
    cfg_q = tof.inputConfig.createInputQueue()

    pipeline.start()
    print(
        f"socket={tof.getBoardSocket()} output={tof.getOutputResolution()} raw={tof.getRawResolution()} "
        f"auto_cam={tof.getCamera() is not None}"
    )

    depth = depth_q.get()
    print(f"depth frame: {depth.getWidth()}x{depth.getHeight()}")

    # optional: push updated IPP params at runtime
    cfg.enableRadialToPerp = True
    cfg_q.send(cfg)

    depth2 = depth_q.get()
    print(f"depth frame after runtime config: {depth2.getWidth()}x{depth2.getHeight()}")
