#!/usr/bin/env python3

import depthai as dai

# Create pipeline
device = dai.Device()
with dai.Pipeline(device) as pipeline:
    aout = pipeline.create(dai.node.AudioOut);
    aout.setRunOnHost(True);
    aout.setDeviceName("speaker");
    aout.setDevicePath("default");
    aout.setBitrate(48000);
    aout.setFps(16);
    aout.setChannels(2);
    aout.setFormat(dai.AudioFrame.AudioFormat.AUDIO_FORMAT_PCM_32);

    areplay = pipeline.create(dai.node.AudioReplay);
    areplay.setSourceFile("/tmp/test.wav");
    areplay.setLoop(True);
    areplay.setFps(16);

    aencoder = pipeline.create(dai.node.AudioEncoder);
    aencoder.setRunOnHost(False);
    aencoder.setFormat(dai.AudioFrame.AudioFormat.AUDIO_FORMAT_PCM_32);
    aencoder.setBitrate(48000);
    aencoder.setChannels(2);

    areplay.out.link(aencoder.input);
    aencoder.out.link(aout.input);

    pipeline.start()
    while pipeline.isRunning():
        continue;
