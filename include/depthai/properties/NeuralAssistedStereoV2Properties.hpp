#pragma once

#include "depthai/pipeline/datatype/NeuralAssistedStereoV2Config.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct NeuralAssistedStereoV2Properties : PropertiesSerializable<Properties, NeuralAssistedStereoV2Properties> {
    NeuralAssistedStereoV2Config initialConfig;
    int numFramesPool = 4;
    ~NeuralAssistedStereoV2Properties() override;
};

DEPTHAI_SERIALIZE_EXT(NeuralAssistedStereoV2Properties, initialConfig, numFramesPool);

}  // namespace dai
