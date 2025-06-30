// Includes common necessary includes for development using depthai library
#include <chrono>
#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/utility/AudioHelpers.hpp"

int main(int argc, char **argv) {
    // Create pipeline
    dai::Pipeline pipeline;

    auto replay = pipeline.create<dai::node::AudioReplay>();
    replay->setSourceFile("/tmp/test.wav");
    replay->setLoop(true);
    replay->setFps(100);

    auto outHost = pipeline.create<dai::node::AudioOut>();
    outHost->setRunOnHost(false);
    outHost->setDeviceName("speaker");
    //outHost->setDevicePath("default");
    outHost->setDevicePath(argc == 2 ? argv[1] : "default");
    outHost->setBitrate(24000);
    outHost->setFps(100);
    outHost->setChannels(2);
    outHost->setFormat(SF_FORMAT_PCM_16);

    auto encoder = pipeline.create<dai::node::AudioEncoder>();
    encoder->setRunOnHost(false);
    encoder->setFormat(SF_FORMAT_PCM_16);
    encoder->setBitrate(24000);
    encoder->setChannels(2);

    replay->out.link(encoder->input);
    encoder->out.link(outHost->input);

    pipeline.start();
    while(pipeline.isRunning()) {
    }

    return 0;
}
