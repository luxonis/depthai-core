#pragma once

#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for NeuralDepth
 */
struct NeuralDepthProperties : PropertiesSerializable<Properties, NeuralDepthProperties> {
    NeuralDepthConfig initialConfig;
    ~NeuralDepthProperties() override;
};

DEPTHAI_SERIALIZE_EXT(NeuralDepthProperties, initialConfig);

}  // namespace dai
