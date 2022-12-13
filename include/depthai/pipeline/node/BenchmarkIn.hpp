#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai-shared/properties/BenchmarkProperties.hpp>

namespace dai {
namespace node {

class BenchmarkIn : public NodeCRTP<DeviceNode, BenchmarkIn, BenchmarkProperties> {
   public:
    constexpr static const char* NAME = "BenchmarkIn";
    using NodeCRTP::NodeCRTP;

    /**
     * Receive messages as fast as possible
     */
    Input input{true, *this, "input", Input::Type::SReceiver, true, 4, {{DatatypeEnum::Buffer, true}}};

    /**
     * Passthrough for input messages (so the node can be placed between other nodes)
     */
    Output passthrough{true, *this, "passthrough", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Send a benchmark report at the end
     */
    Output report{true, *this, "report", Output::Type::MSender, {{DatatypeEnum::BenchmarkReport, false}}};

    /**
     * Set number of messages that the nodes retrieves before sending the report
     * The passthrough keeps getting forwarded after the report is sent
     * @param num of messages to get for report
    */
    void setNumMessagesToGet(int num);
};

}  // namespace node
}  // namespace dai
