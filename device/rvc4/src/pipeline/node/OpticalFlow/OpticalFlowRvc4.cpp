#include "OpticalFlow.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/FeatureTracker/FeatureTrackerAccel.hpp"
#include "utilities/OpticalFlowInput.hpp"
#include "utilities/OpticalFlowOutput.hpp"
namespace dai {
namespace gate {
void OpticalFlow::run() {
    const auto& logger = ThreadedNode::pimpl->logger;
    const bool connected = !outputFlow.getConnections().empty() || !outputFlow.getQueueConnections().empty();
    std::unique_ptr<OpticalFlowAccel> accelerator;
    std::shared_ptr<ImgFrame> previous;
    while(mainLoop()) {
        std::shared_ptr<ImgFrame> current;
        {
            auto event = inputBlockEvent();
            current = inputImage.get<ImgFrame>();
        }
        if(!current) {
            logger->error("OpticalFlow inputImage requires ImgFrame messages");
            stop();
            return;
        }
        try {
            validateOpticalFlowInput(*current, previous.get());
        } catch(const std::invalid_argument& error) {
            // Invalid caller data must stop this node, not escape its worker
            // thread and terminate the entire device runtime.
            logger->error("{}", error.what());
            stop();
            return;
        }
        std::shared_ptr<MessageGroup> result;
        if(connected) {
            if(!accelerator) {
                accelerator = std::make_unique<OpticalFlowAccel>();
                accelerator->Open();
                accelerator->Init(current, false);
                accelerator->Config();
            }
            if(previous) {
                accelerator->Run(previous, current);
                result = makeOpticalFlowOutput(
                    accelerator->forwardMotionVectors(), accelerator->backwardMotionVectors(), accelerator->confidenceMap(), *previous, *current);
            }
        }
        {
            auto event = outputBlockEvent();
            passthroughInputImage.send(current);
            if(result) outputFlow.send(result);
        }
        previous = std::move(current);
    }
}
}  // namespace gate
}  // namespace dai
