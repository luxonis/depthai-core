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

    areplay = pipeline.create(dai.node.AudioReplay);
    areplay.setSourceFile("/tmp/test.wav");
    areplay.setLoop(True);
    areplay.setFps(16);

    aencoder = pipeline.create(dai.node.AudioEncoder);
    aencoder.setRunOnHost(False);
    aencoder.setFormat(dai.AudioFrame.AudioFormat.AUDIO_FORMAT_PCM_32);
    aencoder.setBitrate(48000);
    aencoder.setChannels(2);

    amixer = pipeline.create(dai::node::AudioMixer);
    amixer.setRunOnHost(False);
    amixer.registerSource("source1", 0.2);
    amixer.registerSource("source2", 0.8);
    amixer.registerSink("sink1", 48000, 2, dai.AudioFrame.AudioFormat.AUDIO_FORMAT_PCM_32);
    amixer.linkSourceToSink("source1", "sink1");
    amixer.linkSourceToSink("source2", "sink1");

    areplay.out.link(aencoder.input);
    aencoder.out.link(amixer.inputs["source1"]);
    ain.out.link(amixer.inputs["source2"]);
    amixer.outputs["sink1"].link(aout.input);

    pipeline.start()
    while pipeline.isRunning():
        continue;
