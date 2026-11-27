#pragma once

#include "depthai/pipeline/Node.hpp"

namespace dai {

class NodeGroup : public Node {
   public:
    NodeGroup() = default;
    virtual ~NodeGroup() = default;

    const char* getName() const override {
        return "NodeGroup";
    };
    bool isBuiltInNode() const override {
        return true;
    }
    friend class PipelineImpl;
};

}  // namespace dai
