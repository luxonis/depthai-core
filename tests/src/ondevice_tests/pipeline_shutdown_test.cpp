#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"

namespace {
class StopOnFrame : public dai::node::CustomThreadedNode<StopOnFrame> {
   public:
    Input input{*this, {"input", DEFAULT_GROUP, true, 4, {{dai::DatatypeEnum::ImgFrame, false}}}};
    bool requestOnPipeline = false;

    void run() override {
        {
            auto blockEvent = inputBlockEvent();
            input.get<dai::ImgFrame>();
        }
        if(requestOnPipeline) {
            getParentPipeline().addTask([self = std::static_pointer_cast<StopOnFrame>(shared_from_this())] { self->stopPipeline(); });
        } else {
            stopPipeline();
        }
    }
};
}  // namespace

TEST_CASE("Host-requested shutdown closes the camera before run returns") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto stopper = pipeline.create<StopOnFrame>();
    stopper->requestOnPipeline = GENERATE(false, true);
    CAPTURE(stopper->requestOnPipeline);
    camera->requestOutput({320, 240})->link(stopper->input);

    pipeline.run();

    CHECK_FALSE(pipeline.isRunning());
    CHECK(device->isClosed());
}
