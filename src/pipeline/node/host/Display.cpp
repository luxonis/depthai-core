#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include <opencv2/opencv.hpp>
namespace dai {
namespace node {
void Display::run() {
    while(isRunning()) {
        std::shared_ptr<dai::ImgFrame> imgFrame = input.queue->get<dai::ImgFrame>();
        if(imgFrame != nullptr) {
            cv::imshow("Preview frame", imgFrame->getCvFrame());
            auto key = cv::waitKey(1);
            if(key == 'q') {
                // Get the parent pipeline and stop it
                // TODO(Morato) - add a convience stop method directly to the pipeline
                auto parentPipeline = getParentPipeline();
                parentPipeline.stop();
            }
        }
    }
    fmt::print("Display node stopped\n");
}
}  // namespace node
}  // namespace dai