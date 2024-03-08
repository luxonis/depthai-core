#include <iostream>

// project
#include "depthai/pipeline/HostNode.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/Pipeline.hpp"

class Display : public dai::NodeCRTP<dai::HostNode, Display> {
   public:
    /**
     * Input for any ImgFrame messages to be displayed
     * Default queue is blocking with size 8
     */
    Input input{*this};

    void run() override {
        while(isRunning()) {
            std::shared_ptr<dai::ImgFrame> imgFrame = input.queue->get<dai::ImgFrame>();
            if(imgFrame != nullptr) {
                cv::imshow("Preview frame", imgFrame->getCvFrame());
                auto key = cv::waitKey(1);
                if(key == 'q') {
                    stop();
                }
            }
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
