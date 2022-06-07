#include "depthai/pipeline/datatype/AudioInConfig.hpp"

#include <cmath>

namespace dai {

std::shared_ptr<RawBuffer> AudioInConfig::serialize() const {
    return raw;
}

AudioInConfig::AudioInConfig() : Buffer(std::make_shared<RawAudioInConfig>()), cfg(*dynamic_cast<RawAudioInConfig*>(raw.get())) {}
AudioInConfig::AudioInConfig(std::shared_ptr<RawAudioInConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawAudioInConfig*>(raw.get())) {}

void AudioInConfig::setMicGainTimes(float times) {
    cfg.config.micGain = times;
}

void AudioInConfig::setMicGainDecibels(float dB) {
    float times = pow(10, dB / 20);
    setMicGainTimes(times);
}

void AudioInConfig::setPassthrough(bool passThrough) {
    cfg.config.passThrough = passThrough;
}

void AudioInConfig::setDisableOutput(bool disable) {
    cfg.config.disableOutput = disable;
}

AudioInConfigData AudioInConfig::getConfigData() const {
    return cfg.config;
}

}  // namespace dai
