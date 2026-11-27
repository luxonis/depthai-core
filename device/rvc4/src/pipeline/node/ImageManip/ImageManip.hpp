#pragma once

#include <stdexcept>

#include "depthai/pipeline/node/ImageManip.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class ImageManip : public ::dai::node::ImageManip {
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

   public:
    constexpr static const char* NAME = node::ImageManip::NAME;
    ImageManip() : ImageManip(std::make_unique<dai::node::ImageManip::Properties>()) {}
    ImageManip(std::unique_ptr<Properties> props) : dai::node::ImageManip(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
    void run() override;
};

class ManipResizer {
   private:
    Node::Output& outQ;
    std::unique_ptr<std::thread> resizerThread;
    bool isRunning = false;

    ImageManipConfig cfg;
    std::optional<float> alphaScaling;

   public:
    MessageQueue inQ;

    ManipResizer() = delete;
    ManipResizer(ImageManipConfig cfg, Node::Output& output) : cfg(cfg), outQ(output) {}
    ManipResizer(ImageManipConfig cfg, std::optional<float> alphaScaling, Node::Output& output) : cfg(cfg), alphaScaling(alphaScaling), outQ(output) {}

    void runThreadedResizer(int numFramesPool,
                            std::function<std::shared_ptr<ImgFrame>(std::shared_ptr<ImgFrame>)> prepareSend,
                            bool useCpu,
                            std::shared_ptr<spdlog::async_logger> logger);

    void stop();
    void join();
};

}  // namespace gate
}  // namespace dai
