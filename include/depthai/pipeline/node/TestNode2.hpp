#pragma once

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/BatchItem.hpp"

namespace dai {
namespace node {

class TestNode2 : public CustomThreadedNode<TestNode2> {
   public:
    static constexpr const char* NAME = "TestNode2";

    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::BatchItem, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::BatchItem, false}}}}};

    void run() override;
};

}  // namespace node
}  // namespace dai
