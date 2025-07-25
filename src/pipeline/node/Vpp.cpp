#include "depthai/pipeline/node/Vpp.hpp"

#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

Vpp::Vpp(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, Vpp, VppProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

Vpp::Properties& Vpp::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void Vpp::setMethod(VppConfig::Method method) {
    initialConfig->setMethod(method);
    properties.initialConfig = *initialConfig;
}

void Vpp::setPatchSize(int size) {
    if (size % 2 == 0) {
        throw std::runtime_error("VPP patch size must be odd number");
    }
    if (size < 1) {
        throw std::runtime_error("VPP patch size must be positive");
    }
    initialConfig->setPatchSize(size);
    properties.initialConfig = *initialConfig;
}

void Vpp::setAggregationWindowSize(int sizeX, int sizeY) {
    if (sizeX < 1 || sizeY < 1) {
        throw std::runtime_error("VPP aggregation window sizes must be positive");
    }
    initialConfig->setAggregationWindowSize(sizeX, sizeY);
    properties.initialConfig = *initialConfig;
}

void Vpp::setProjectionDirection(bool left2right) {
    initialConfig->setProjectionDirection(left2right);
    properties.initialConfig = *initialConfig;
}

void Vpp::setBlending(float blending, float blendingOcclusion) {
    if (blending < 0.0f || blending > 1.0f) {
        throw std::runtime_error("VPP blending factor must be between 0.0 and 1.0");
    }
    if (blendingOcclusion < 0.0f || blendingOcclusion > 1.0f) {
        throw std::runtime_error("VPP occlusion blending factor must be between 0.0 and 1.0");
    }
    initialConfig->setBlending(blending, blendingOcclusion);
    properties.initialConfig = *initialConfig;
}

void Vpp::setDistancePatch(bool enable, float gamma) {
    if (gamma == 0){
        throw std::runtime_error("VPP distance gamma must be positive");
    }
    if (gamma <= 0.0f) {
        throw std::runtime_error("VPP distance gamma must be positive");
    }
    initialConfig->setDistancePatch(enable, gamma);
    properties.initialConfig = *initialConfig;
}

void Vpp::setBilateralPatch(bool enable, float spatialSigma, float intensitySigma, float threshold) {
    if (spatialSigma <= 0.0f || intensitySigma <= 0.0f) {
        throw std::runtime_error("VPP bilateral sigma values must be positive");
    }
    if (threshold < 0.0f) {
        throw std::runtime_error("VPP bilateral threshold must be non-negative");
    }
    initialConfig->setBilateralPatch(enable, spatialSigma, intensitySigma, threshold);
    properties.initialConfig = *initialConfig;
}

void Vpp::setDisparityRange(float minThreshold, float maxThreshold) {
    if (minThreshold < 0.0f || maxThreshold < 0.0f) {
        throw std::runtime_error("VPP disparity thresholds must be non-negative");
    }
    if (minThreshold >= maxThreshold) {
        throw std::runtime_error("VPP minimum disparity threshold must be less than maximum");
    }
    initialConfig->setDisparityRange(minThreshold, maxThreshold);
    properties.initialConfig = *initialConfig;
}

void Vpp::setUniformColor(bool uniform) {
    initialConfig->uniformColor = uniform;
    properties.initialConfig = *initialConfig;
}

void Vpp::setDiscardOcclusion(bool discard) {
    initialConfig->discardOcclusion = discard;
    properties.initialConfig = *initialConfig;
}

void Vpp::setInterpolate(bool interpolate) {
    initialConfig->interpolate = interpolate;
    properties.initialConfig = *initialConfig;
}

void Vpp::setNumFramesPool(int numFramesPool) {
    if (numFramesPool < 1) {
        throw std::runtime_error("VPP number of frames in pool must be positive");
    }
    properties.numFramesPool = numFramesPool;
}

}  // namespace node
}  // namespace dai