// Includes common necessary includes for development using depthai library
#include <chrono>
#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/utility/AudioHelpers.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    auto inHost = pipeline.create<dai::node::AudioIn>();
    inHost->setRunOnHost(true);
    inHost->setDeviceName("microphone");
    inHost->setDevicePath("default");
    inHost->setBitrate(44100);
    inHost->setFps(16);
    inHost->setChannels(2);
    inHost->setFormat(SF_FORMAT_PCM_32);

    auto encoder = pipeline.create<dai::node::AudioEncoder>();
    encoder->setRunOnHost(false);
    encoder->setFormat(SF_FORMAT_PCM_16);
    encoder->setBitrate(44100);
    encoder->setChannels(2);

    auto outHost = pipeline.create<dai::node::AudioOut>();
    outHost->setRunOnHost(true);
    outHost->setDeviceName("speaker");
    outHost->setDevicePath("default");
    outHost->setBitrate(44100);
    outHost->setFps(16);
    outHost->setChannels(2);
    outHost->setFormat(SF_FORMAT_PCM_16);

    inHost->out.link(encoder->input);
    encoder->out.link(outHost->input);

    pipeline.start();
    while(pipeline.isRunning()) {
    }

    return 0;
}
