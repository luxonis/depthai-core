#include <iostream>


// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// project
#include "depthai/pipeline/ThreadedNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

class Display : public dai::NodeCRTP<dai::ThreadedNode, Display> {
   public:
    constexpr static const char* NAME = "Display";

   public:
    void build() {
        hostNode = true;
    }

    /**
     * Input for any ImgFrame messages to be displayed
     * Default queue is blocking with size 8
     */
    Input input{true, *this, "in", Input::Type::SReceiver, true, 8, true, {{dai::DatatypeEnum::Buffer, true}}};

    void run() override {
        while(isRunning()) {
            std::shared_ptr<dai::ImgFrame> imgFrame = input.queue.get<dai::ImgFrame>();
            if(imgFrame != nullptr) {
                cv::imshow("MyConsumer", imgFrame->getCvFrame());
                auto key = cv::waitKey(1);
                if(key == 'q') {
                    stop();
                }            }
        }
        fmt::print("Display node stopped\n");
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
    pipeline.wait();

    return 0;
}
