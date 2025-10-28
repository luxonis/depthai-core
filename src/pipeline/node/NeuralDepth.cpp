#include "depthai/pipeline/node/NeuralDepth.hpp"

#include <mutex>

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

NeuralDepth::Properties& NeuralDepth::getProperties() {
    return properties;
}

NeuralDepth& NeuralDepth::setModelType(NeuralDepthProperties::ModelType modelType) {
    properties.modelType = modelType;
    return *this;
}

NeuralDepthProperties::ModelType NeuralDepth::getModelType() const {
    return properties.modelType;
}

}  // namespace node
}  // namespace dai
