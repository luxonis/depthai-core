#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>

#include "depthai/utility/export.hpp"

namespace dai {
namespace node {
namespace test {

/**
 * @brief XLinkOut node. Sends messages over XLink.
 */
class DEPTHAI_API MyProducer : public NodeCRTP<ThreadedHostNode, MyProducer> {
   public:
    constexpr static const char* NAME = "MyProducer";

    MyProducer() = default;
    MyProducer(std::unique_ptr<Properties>) {}

    /**
     * Outputs message of same type as sent from host.
     */
    Output out{this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    void run() override;
};

}  // namespace test
}  // namespace node
}  // namespace dai
