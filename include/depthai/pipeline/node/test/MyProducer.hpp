#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// shared
#include <depthai-shared/properties/XLinkOutProperties.hpp>

namespace dai {
namespace node {
namespace test {

/**
 * @brief XLinkOut node. Sends messages over XLink.
 */
class MyProducer : public NodeCRTP<ThreadedNode, MyProducer, XLinkOutProperties> {
   public:
    constexpr static const char* NAME = "MyProducer";
    void build();

   public:
    /**
     * Outputs message of same type as send from host.
     */
    Output out{true, *this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    void run() override;
};

}  // namespace test
}  // namespace node
}  // namespace dai
