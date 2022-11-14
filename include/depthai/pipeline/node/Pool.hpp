#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/PoolProperties.hpp>

namespace dai {
namespace node {

class Pool : public NodeCRTP<DeviceNode, Pool, PoolProperties> {
   public:
    constexpr static const char* NAME = "Pool";

    /**
     *  Inputs to Script node. Can be accessed using subscript operator (Eg: inputs['in1'])
     *  By default inputs are set to blocking with queue size 8
     */
    Output out{*this, "out", Output::Type::SSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Sets number of messages in pool
     * @param num number of messages in pool
     */
    void setNumMessages(int num);

    /// Retrieves pool size
    int getNumMessages() const;

    /**
     * Sets reserved data size of each message in the pool
     * @param size data size allocated for each message
     */
    void setMaxMessageSize(std::int64_t size);

    /// Retrieves pool size
    std::int64_t getMaxMessageSize() const;
};

}  // namespace node
}  // namespace dai
