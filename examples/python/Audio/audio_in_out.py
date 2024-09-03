#!/usr/bin/env python3

import depthai as dai

# Create pipeline
device = dai.Device()
with dai.Pipeline(device) as pipeline:
    ain = pipeline.create(dai.node.AudioIn);
    ain.setRunOnHost(True);
    ain.setDeviceName("microphone");
    ain.setDevicePath("default");
    ain.setBitrate(48000);
    ain.setFps(16);
    ain.setChannels(2);
    ain.setFormat(dai.AudioFrame.AudioFormat.AUDIO_FORMAT_PCM_32);

    aout = pipeline.create(dai.node.AudioOut);
    aout.setRunOnHost(True);
    aout.setDeviceName("speaker");
    aout.setDevicePath("default");
    aout.setBitrate(48000);
    aout.setFps(16);
    aout.setChannels(2);
    aout.setFormat(dai.AudioFrame.AudioFormat.AUDIO_FORMAT_PCM_32);

    ain.out.link(aout.input);

    pipeline.start()
    while pipeline.isRunning():
        continue;
