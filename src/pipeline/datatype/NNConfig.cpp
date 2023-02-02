#include "depthai/pipeline/datatype/NNConfig.hpp"

#include <cassert>
#include <limits>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawNNConfig.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"

namespace dai {

std::shared_ptr<RawBuffer> NNConfig::serialize() const {
    return raw;
}

NNConfig::NNConfig() : Buffer(std::make_shared<RawNNConfig>()), cfg(*dynamic_cast<RawNNConfig*>(raw.get())) {}
NNConfig::NNConfig(std::shared_ptr<RawNNConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawNNConfig*>(raw.get())) {}

NNConfig& NNConfig::setProcessInputs(bool processInputs) {
    cfg.processInputs = processInputs;
    return *this;
}

bool NNConfig::getProcessInputs() const {
    return cfg.processInputs;
}

}  // namespace dai
