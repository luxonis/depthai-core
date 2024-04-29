#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>

namespace dai {
namespace node {
namespace test {

/**
 * @brief XLinkOut node. Sends messages over XLink.
 */
class MyProducer : public NodeCRTP<ThreadedHostNode, MyProducer> {
   public:
    constexpr static const char* NAME = "MyProducer";

    /**
     * Outputs message of same type as sent from host.
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::Buffer, true}}}};

    void run() override;
};

}  // namespace test
}  // namespace node
}  // namespace dai
