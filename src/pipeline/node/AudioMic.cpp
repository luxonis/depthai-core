#include "depthai/pipeline/node/AudioMic.hpp"

namespace dai {
namespace node {

AudioMic::AudioMic(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : AudioMic(par, nodeId, std::make_unique<AudioMic::Properties>()) {}
AudioMic::AudioMic(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, AudioMic, AudioMicProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawAudioInConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig});
    setOutputRefs({&out, &outBack});
}

AudioMic::Properties& AudioMic::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

void AudioMic::setStreamBackMic(bool enable) {
    properties.streamBackMic = enable;
}

void AudioMic::setMicAutoGain(bool enable) {
    properties.enableAgc = enable;
}

void AudioMic::setXlinkApplyMicGain(bool enable) {
    properties.xLinkApplyMicGain = enable;
}

void AudioMic::setXlinkSampleSizeBytes(int size) {
    if (size < 2 || size > 4) {
        throw std::runtime_error("AudioMic | sample size must be: 2, 3 or 4");
    }
    properties.xlinkSampleSizeBytes = size;
}

}  // namespace node
}  // namespace dai
