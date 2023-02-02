#pragma once

#include <chrono>
#include <limits>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawNNConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * NNConfig message. Carries tensors and their metadata
 */
class NNConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawNNConfig& cfg;

   public:
    /**
     * Construct NNConfig message.
     */
    NNConfig();
    explicit NNConfig(std::shared_ptr<RawNNConfig> ptr);
    virtual ~NNConfig() = default;

    NNConfig& setProcessInputs(bool processInputs);

    bool getProcessInputs() const;
};

}  // namespace dai
