#include "depthai/pipeline/node/AudioProc.hpp"

namespace dai {
namespace node {

AudioProc::AudioProc(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : AudioProc(par, nodeId, std::make_unique<AudioProc::Properties>()) {}
AudioProc::AudioProc(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, AudioProc, AudioProcProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&input, &reference});
    setOutputRefs({&out});
}

// node properties
void AudioProc::setNumFramesPool(int frames) {
    // 16 is maximum allowed value
    if(frames > 16) {
        throw std::invalid_argument("Maximum number of frames in pool for AudioProc is 16");
    }

    properties.numFramesPool = frames;
    // Set default input queue size as well
    input.defaultQueueSize = frames;
    reference.defaultQueueSize = frames;
}

int AudioProc::getNumFramesPool() const {
    return properties.numFramesPool;
}

}  // namespace node
}  // namespace dai
