#include "depthai/pipeline/DeviceNode.hpp"

// std
#include "spdlog/fmt/fmt.h"

// project
#include "depthai/pipeline/Pipeline.hpp"

namespace dai {

void DeviceNode::run() {
    auto pipeline = getParentPipeline();
    if(pipeline.getDevice() != nullptr) {
        auto dev = pipeline.getDevice();

        auto outputQueueNames = dev->getOutputQueueNames();
        auto inputQueueNames = dev->getInputQueueNames();

        for(auto output : getOutputRefs()) {
            auto xlinkName = fmt::format("__x_{}_{}", output->getParent().id, output->toString());
            // TODO - validate
            // printf("checking output: %s\n", xlinkName.c_str());

            if(std::find(outputQueueNames.begin(), outputQueueNames.end(), xlinkName) != outputQueueNames.end()) {
                auto thisNode = pipeline.getNode(id);

                dev->getOutputQueue(xlinkName)->addCallback([thisNode, output](std::string, std::shared_ptr<ADatatype> msg) { output->send(msg); });
            }
        }
        for(auto input : getInputRefs()) {
            auto xlinkName = fmt::format("__x_{}_{}", input->getParent().id, input->toString());
            // TODO - validate
            // printf("checking input: %s\n", xlinkName.c_str());
            if(std::find(inputQueueNames.begin(), inputQueueNames.end(), xlinkName) != inputQueueNames.end()) {
                input->queue.addCallback([dev, xlinkName](std::string, std::shared_ptr<ADatatype> msg) { dev->getInputQueue(xlinkName)->send(msg); });
            }
        }
    }

}

}  // namespace dai
