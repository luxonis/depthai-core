#pragma once

// project
#include "depthai/pipeline/Node.hpp"
#include "depthai/utility/export.hpp"

namespace dai {

class DEPTHAI_API NodeGroup : public Node {
   public:
    NodeGroup() = default;
    virtual ~NodeGroup() = default;

    const char* getName() const override {
        return "NodeGroup";
    };
    // std::unique_ptr<Node> clone() const override {
    //     return std::make_unique<NodeGroup>(static_cast<const NodeGroup&>(*this));
    // };
    friend class PipelineImpl;
};

}  // namespace dai
