#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for NeuralDepth
 */
struct NeuralDepthProperties : PropertiesSerializable<Properties, NeuralDepthProperties> {

    enum class ModelType : uint32_t {
        LARGE,
        MEDIUM,
        SMALL,
        NANO    
    };

    ModelType modelType = ModelType::LARGE;

    ~NeuralDepthProperties() override;
};

DEPTHAI_SERIALIZE_EXT(NeuralDepthProperties, modelType);

}  // namespace dai