// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/utility/SharedMemory.hpp"
#include "depthai/pipeline/InputQueue.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;
    
    auto aReplay = pipeline.create<dai::node::AudioReplay>();
    auto aIn = pipeline.create<dai::node::AudioIn>();
    auto aOut = pipeline.create<dai::node::AudioOut>();
    auto aEncode = pipeline.create<dai::node::AudioEncoder>();
    auto aMix = pipeline.create<dai::node::AudioMixer>();

    pipeline.start();
    while(pipeline.isRunning()) {
    }
    return 0;
}
