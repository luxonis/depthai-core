// Includes common necessary includes for development using depthai library
#include <chrono>
#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/utility/AudioHelpers.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    auto replay = pipeline.create<dai::node::AudioReplay>();
    replay->setSourceFile("/tmp/test.wav");
    replay->setLoop(true);
    replay->setFps(16);

    auto outHost = pipeline.create<dai::node::AudioOut>();
    outHost->setRunOnHost(true);
    outHost->setDeviceName("speaker");
    outHost->setDevicePath("default");
    outHost->setBitrate(48000);
    outHost->setFps(16);
    outHost->setChannels(2);
    outHost->setFormat(SF_FORMAT_PCM_32);

    auto encoder = pipeline.create<dai::node::AudioEncoder>();
    encoder->setRunOnHost(false);
    encoder->setFormat(SF_FORMAT_PCM_32);
    encoder->setBitrate(48000);
    encoder->setChannels(2);

    replay->out.link(encoder->input);
    encoder->out.link(outHost->input);

    pipeline.start();
    while(pipeline.isRunning()) {
    }

    return 0;
}
