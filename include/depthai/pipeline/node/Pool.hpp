#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai/properties/PoolProperties.hpp>

namespace dai {
namespace node {

// TODO(before mainline) - API not supported on RVC2
class Pool : public DeviceNodeCRTP<DeviceNode, Pool, PoolProperties> {
   public:
    constexpr static const char* NAME = "Pool";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     *  Pool output in pull orientation
     */
    Output out{true, *this, "out", Output::Type::SSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Sets number of messages in pool
     * @param num number of messages in pool
     */
    void setNumMessages(std::optional<int> num);

    /// Retrieves pool size
    std::optional<int> getNumMessages() const;

    /**
     * Sets reserved data size of each message in the pool
     * @param size data size allocated for each message
     */
    void setMaxMessageSize(std::optional<std::int64_t> size);

    /// Retrieves pool size
    std::optional<std::int64_t> getMaxMessageSize() const;

    /**
     * Sets optional type of the pool. TODO(themarpe) - might be required
     * @param datatype Type of the pool messages
     */
    void setDatatype(std::optional<DatatypeEnum> type);

    /// Retrieves optional pool type
    std::optional<DatatypeEnum> getDatatype() const;

    /**
     * Set on which processor the pool should be placed
     * @param type Processor type - Leon CSS or Leon MSS
     */
    void setProcessor(std::optional<ProcessorType> type);

    /**
     * Get on which processor the pool should be placed
     * @returns Processor type - Leon CSS or Leon MSS
     */
    std::optional<ProcessorType> getProcessor() const;
};

}  // namespace node
}  // namespace dai
