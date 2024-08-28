// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/utility/AudioHelpers.hpp"
#include "depthai/pipeline/InputQueue.hpp"

#include <iostream>
#include <chrono>

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    auto inHost = pipeline.create<dai::node::AudioIn>();
    inHost->setRunOnHost(true);
    inHost->setDeviceName("microphone");
    inHost->setDevicePath("default");
    inHost->setBitrate(48000);
    inHost->setFps(16);
    inHost->setChannels(2);
    inHost->setFormat(SF_FORMAT_PCM_32);

    auto outHost = pipeline.create<dai::node::AudioOut>();
    outHost->setRunOnHost(true);
    outHost->setDeviceName("speaker");
    outHost->setDevicePath("default");
    outHost->setBitrate(48000);
    outHost->setFps(16);
    outHost->setChannels(2);
    outHost->setFormat(SF_FORMAT_PCM_32);

    inHost->out.link(outHost->input);

    pipeline.start();
    while(pipeline.isRunning()) {
    }

    return 0;
}
