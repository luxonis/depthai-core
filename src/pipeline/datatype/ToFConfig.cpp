#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {
ToFConfig& ToFConfig::setDepthParams(dai::ToFConfig::DepthParams config) {
    depthParams = config;
    return *this;
}

ToFConfig& ToFConfig::setFreqModUsed(dai::ToFConfig::DepthParams::TypeFMod fmod) {
    depthParams.freqModUsed = fmod;
    return *this;
}

ToFConfig& ToFConfig::setAvgPhaseShuffle(bool enable) {
    depthParams.avgPhaseShuffle = enable;
    return *this;
}

ToFConfig& ToFConfig::setMinAmplitude(float minamp) {
    depthParams.minimumAmplitude = minamp;
    return *this;
}

ToFConfig& ToFConfig::setMedianFilter(MedianFilter median) {
    depthParams.median = median;
    return *this;
}

}  // namespace dai
