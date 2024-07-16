#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/BenchmarkPropertiesIn.hpp>

namespace dai {
namespace node {

// TODO(before mainline) - API not supported on RVC2
class BenchmarkIn : public DeviceNodeCRTP<DeviceNode, BenchmarkIn, BenchmarkPropertiesIn> {
   public:
    constexpr static const char* NAME = "BenchmarkIn";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    std::shared_ptr<BenchmarkIn> build() {
        return std::static_pointer_cast<BenchmarkIn>(shared_from_this());
    }
    /**
     * Receive messages as fast as possible
     */
    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Passthrough for input messages (so the node can be placed between other nodes)
     */
    Output passthrough{*this, {"passthrough", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Send a benchmark report when the set number of messages are received
     */
    Output report{*this, {"report", DEFAULT_GROUP, {{{DatatypeEnum::BenchmarkReport, false}}}}};

    /**
     * Set number of messages that the nodes retrieves before sending the report
     * The passthrough keeps getting forwarded after the report is sent
     * @param num of messages to get for report
     */
    void setNumMessagesToGet(int num);
};

}  // namespace node
}  // namespace dai
