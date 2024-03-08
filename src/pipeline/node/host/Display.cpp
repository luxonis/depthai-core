#include "depthai/pipeline/node/host/Display.hpp"

#include <opencv2/opencv.hpp>
namespace dai {
void Display::run() {
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
}  // namespace dai