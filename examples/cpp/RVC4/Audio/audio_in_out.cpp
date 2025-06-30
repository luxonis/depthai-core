// Includes common necessary includes for development using depthai library
#include <chrono>
#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/utility/AudioHelpers.hpp"

int main(int argc, char **argv) {
    // Create pipeline
    dai::Pipeline pipeline;

    auto inHost = pipeline.create<dai::node::AudioIn>();
    inHost->setRunOnHost(false);
    inHost->setDeviceName("microphone");
    //inHost->setDevicePath("default");
    inHost->setDevicePath(argc == 2 ? argv[1] : "default");
    inHost->setBitrate(48000);
    inHost->setFps(90);
    inHost->setChannels(2);
    inHost->setFormat(SF_FORMAT_PCM_32);
    
    auto mixer = pipeline.create<dai::node::AudioMixer>();
    mixer->setRunOnHost(true);
    mixer->registerSource("source1", 4);
    mixer->registerSink("sink1", 48000, 2, SF_FORMAT_PCM_32);
    mixer->linkSourceToSink("source1", "sink1");

    auto outHost = pipeline.create<dai::node::AudioOut>();
    outHost->setRunOnHost(true);
    outHost->setDeviceName("speaker");
    //outHost->setDevicePath(argc == 2 ? argv[1] : "default");
    outHost->setDevicePath("default");
    outHost->setBitrate(48000);
    outHost->setFps(90);
    outHost->setChannels(2);
    outHost->setFormat(SF_FORMAT_PCM_32);

    inHost->out.link(mixer->inputs["source1"]);
    mixer->outputs["sink1"].link(outHost->input);
    
    //inHost->out.link(outHost->input);

    pipeline.start();
    while(pipeline.isRunning()) {
    }

    return 0;
}
