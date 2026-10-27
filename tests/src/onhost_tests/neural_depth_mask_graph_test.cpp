#include <memory>
#include <stdexcept>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"

namespace {
void require(bool condition) {
    if(!condition) throw std::runtime_error("NeuralDepth mask connection regression check failed");
}
}  // namespace

int main() {
    {
        auto node = std::make_shared<dai::node::NeuralDepth>();
        node->buildStage1();
        require(node->inputValidMask.isConnected());
        require(node->rectification->validMask1.getConnections().size() == 1);
        require(node->rectification->validMask1.getConnections().front().in == &node->inputValidMask);
        // Repeated preparation must not add the default link twice.
        node->buildStage1();
        require(node->rectification->validMask1.getConnections().size() == 1);
    }
    {
        auto node = std::make_shared<dai::node::NeuralDepth>();
        node->setRectification(false);
        node->buildStage1();
        require(!node->inputValidMask.isConnected());
        require(node->rectification->validMask1.getConnections().empty());
    }
    for(bool rectify : {false, true}) {
        auto node = std::make_shared<dai::node::NeuralDepth>();
        auto external = std::make_shared<dai::node::Rectification>();
        node->setRectification(rectify);
        external->validMask1.link(node->inputValidMask);
        node->buildStage1();
        require(node->inputValidMask.isConnected());
        require(node->rectification->validMask1.getConnections().empty());
        require(external->validMask1.getConnections().front().in == &node->inputValidMask);
    }
    {
        // Only prepare the graph here: no device or inference is needed to check queue precedence.
        dai::Pipeline pipeline(false);
        auto node = std::make_shared<dai::node::NeuralDepth>();
        pipeline.add(node);
        auto queue = node->inputValidMask.createInputQueue();
        node->buildStage1();
        require(node->inputValidMask.isConnected());
        require(node->rectification->validMask1.getConnections().empty());
    }
    return 0;
}
