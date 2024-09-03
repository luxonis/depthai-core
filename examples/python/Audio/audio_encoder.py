#!/usr/bin/env python3

import depthai as dai

# Create pipeline
device = dai.Device()
with dai.Pipeline(device) as pipeline:
    in = pipeline.create(dai.node.AudioIn);
    in.setRunOnHost(True);
    in.setDeviceName("microphone");
    in.setDevicePath("default");
    in.setBitrate(48000);
    in.setFps(16);
    in.setChannels(2);
    in.setFormat(dai.AudioFrame.AUDIO_FORMAT_PCM_32);

    out = pipeline.create(dai.node.AudioOut);
    out.setRunOnHost(True);
    out.setDeviceName("speaker");
    out.setDevicePath("default");
    out.setBitrate(44100);
    out.setFps(16);
    out.setChannels(2);
    out.setFormat(dai.AudioFrame.AUDIO_FORMAT_PCM_16);

    encoder = pipeline.create(dai.node.AudioOut);
    out.setRunOnHost(False);
    out.setBitrate(44100);
    out.setChannels(2);
    out.setFormat(dai.AudioFrame.AUDIO_FORMAT_PCM_16);

    in.out.link(encoder.input);
    encoder.out.link(out.input);

    pipeline.start()
    while pipeline.isRunning():
        continue;
