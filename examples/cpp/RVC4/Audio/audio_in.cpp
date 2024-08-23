// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/utility/AudioHelpers.hpp"
#include "depthai/pipeline/InputQueue.hpp"

#include <iostream>
#include <chrono>

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    auto replay = pipeline.create<dai::node::AudioReplay>();
    replay->setSourceFile("/tmp/test.wav");
    replay->setLoop(true);
    replay->setFps(10); 

    auto inHost = pipeline.create<dai::node::AudioIn>();
    inHost->setRunOnHost(true);
    inHost->setDeviceName("microphone");
    inHost->setDevicePath("default");
    inHost->setBitrate(48000);
    inHost->setFps(10);
    inHost->setChannels(2);
    inHost->setFormat(SF_FORMAT_PCM_32);

    auto outHost = pipeline.create<dai::node::AudioOut>();
    outHost->setRunOnHost(true);
    outHost->setDeviceName("speaker");
    outHost->setDevicePath("default");
    outHost->setBitrate(48000);
    outHost->setFps(10);
    outHost->setChannels(2);
    outHost->setFormat(SF_FORMAT_PCM_32);
    
    auto encoder = pipeline.create<dai::node::AudioEncoder>();
    encoder->setRunOnHost(true);
    encoder->setFormat(SF_FORMAT_PCM_32);
    encoder->setBitrate(48000);
    encoder->setChannels(2);

    auto mixer = pipeline.create<dai::node::AudioMixer>();
    mixer->setRunOnHost(true);

    mixer->registerSource("source1", 0.8);
    mixer->registerSink("sink1", SF_FORMAT_PCM_32);

    mixer->linkSourceToSink("source1", "sink1");

    replay->out.link(encoder->input);
    encoder->out.link(mixer->inputs["source1"]);

    mixer->outputs["sink1"].link(outHost->input);

    pipeline.start();
    while(pipeline.isRunning()) {
    }

    return 0;
}
