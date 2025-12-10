#pragma once

#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for NeuralAssistedStereo
 */
struct NeuralAssistedStereoProperties : PropertiesSerializable<Properties, NeuralAssistedStereoProperties> {
    VppConfig vppConfig;
    StereoDepthConfig stereoConfig;
    NeuralDepthConfig neuralConfig;
    ~NeuralAssistedStereoProperties() override;
};

DEPTHAI_SERIALIZE_EXT(NeuralAssistedStereoProperties, vppConfig, stereoConfig, neuralConfig);

}  // namespace dai
