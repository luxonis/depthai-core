#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/test/MyConsumer.hpp"
#include "depthai/pipeline/node/test/MyProducer.hpp"

// shared
#include "depthai-shared/properties/XLinkOutProperties.hpp"

// project
#include "depthai/pipeline/ThreadedNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

class Display : public dai::NodeCRTP<dai::ThreadedNode, Display, dai::XLinkOutProperties> {
   public:
    constexpr static const char* NAME = "Display";

   public:
    Display(const std::shared_ptr<dai::PipelineImpl>& par, int64_t nodeId) : Display(par, nodeId, std::make_unique<Display::Properties>()) {}
    Display(const std::shared_ptr<dai::PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
        : dai::NodeCRTP<dai::ThreadedNode, Display, dai::XLinkOutProperties>(par, nodeId, std::move(std::move(props))) {
        setInputRefs(&input);
        hostNode = true;
    }

    /**
     * Input for any ImgFrame messages to be displayed
     * Default queue is blocking with size 8
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 8, true, {{dai::DatatypeEnum::Buffer, true}}};

    void run() override {
        while(isRunning()) {
            std::shared_ptr<dai::ImgFrame> imgFrame = input.queue.get<dai::ImgFrame>();
            if(imgFrame != nullptr) {
                cv::imshow("MyConsumer", imgFrame->getCvFrame());
                cv::waitKey(1);
            }
        }
    }
};

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto display = pipeline.create<Display>();
    camRgb->preview.link(display->input);
    pipeline.start();

    return 0;
}
