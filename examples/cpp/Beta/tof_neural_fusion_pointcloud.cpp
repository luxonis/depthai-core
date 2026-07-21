#include <csignal>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <optional>

#include "depthai/depthai.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"

namespace {
volatile std::sig_atomic_t isRunning{1};
void signalHandler(int) {
    isRunning = 0;
}

class DepthColorizer : public dai::NodeCRTP<dai::node::HostNode, DepthColorizer> {
   public:
    Input& input = inputs["depth"];

    std::shared_ptr<DepthColorizer> build(Output& depth) {
        depth.link(input);
        return std::static_pointer_cast<DepthColorizer>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> messages) override {
        auto colorized = dai::utility::colorizeDepthFrame(*messages->get<dai::ImgFrame>("depth"));
        auto color = colorized.getCvFrame();
        cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
        colorized.setCvFrame(color, dai::ImgFrame::Type::RGB888i);
        return std::make_shared<dai::ImgFrame>(std::move(colorized));
    }
};
}  // namespace

int main() {
    std::signal(SIGINT, signalHandler);

    dai::RemoteConnection remote;
    dai::Pipeline pipeline;
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<dai::beta::node::ToFStereoFusion>()->build(left, right);

    auto colorizer = pipeline.create<DepthColorizer>()->build(fusion->depth);
    auto pointcloud = pipeline.create<dai::node::PointCloud>();
    pointcloud->setRunOnHost(true);
    fusion->depth.link(pointcloud->inputDepth);
    colorizer->out.link(pointcloud->getColorInput());
    remote.addTopic("pcl", pointcloud->outputPointCloud);

    pipeline.start();
    remote.registerPipeline(pipeline);
    std::cout << "Visualizer running at http://localhost:8082\n";

    while(isRunning != 0 && pipeline.isRunning() && remote.waitKey(1) != 'q') {
    }
}
