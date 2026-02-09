#include <cstddef>
#include <iostream>
#include <memory>

// project
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"

class SyncedDisplay : public dai::NodeCRTP<dai::node::HostNode, SyncedDisplay> {
   public:
    Input& inputRgb = inputs["rgb"];
    Input& inputMono = inputs["mono"];

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto mono = in->get<dai::ImgFrame>("mono");
        auto rgb = in->get<dai::ImgFrame>("rgb");
        // Show the frames side by side
        auto monoFrame = mono->getCvFrame();
        auto rgbFrame = rgb->getCvFrame();
        if(monoFrame.channels() == 1) {
            cv::Mat tempFrame;
            cv::cvtColor(monoFrame, tempFrame, cv::COLOR_GRAY2BGR);
            monoFrame = tempFrame;
        }
        // Resize both to 640x640
        cv::resize(monoFrame, monoFrame, cv::Size(640, 640));
        cv::resize(rgbFrame, rgbFrame, cv::Size(640, 640));
        cv::Mat displayFrame(640, 640 * 2, CV_8UC3);
        monoFrame.copyTo(displayFrame(cv::Rect(0, 0, 640, 640)));
        rgbFrame.copyTo(displayFrame(cv::Rect(640, 0, 640, 640)));
        cv::imshow("Synced frames", displayFrame);
        auto q = cv::waitKey(1);
        if(q == 'q') {
            stopPipeline();
        }
        return nullptr;
    }
};

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);

    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto camMono = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);

    auto syncedDisplay = pipeline.create<SyncedDisplay>();
    syncedDisplay->sendProcessingToPipeline(true);
    camRgb->requestOutput(std::make_pair(640, 480))->link(syncedDisplay->inputRgb);
    camMono->requestOutput(std::make_pair(640, 480))->link(syncedDisplay->inputMono);

    pipeline.run();
    return 0;
}
