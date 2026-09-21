#include <algorithm>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"
#include "rerun.hpp"
#include "rerun/archetypes/depth_image.hpp"

class RerunNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, RerunNode> {
   public:
    constexpr static const char* NAME = "RerunNode";

   public:
    Input inputPCL{*this, {.name = "inPCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
    Input inputRGBD{*this, {.name = "inRGBD", .types = {{dai::DatatypeEnum::RGBDData, true}}}};

    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_static("world", rerun::ViewCoordinates::RDF);
        while(mainLoop()) {
            auto pclIn = inputPCL.get<dai::PointCloudData>();
            auto rgbdIn = inputRGBD.get<dai::RGBDData>();
            if(pclIn != nullptr) {
                std::vector<rerun::Position3D> points;
                std::vector<rerun::Color> colors;
                const auto& size = pclIn->getWidth() * pclIn->getHeight();
                points.reserve(size);
                colors.reserve(size);
                const auto& pclData = pclIn->getPointsRGB();
                for(size_t i = 0; i < size; ++i) {
                    points.emplace_back(pclData[i].x, pclData[i].y, pclData[i].z);
                    colors.emplace_back(pclData[i].r, pclData[i].g, pclData[i].b);
                }
                rec.log("world/obstacle_pcl", rerun::Points3D(points).with_colors(colors).with_radii({0.01f}));
                auto rgbFrame = rgbdIn->getRGBFrame();
                if(!rgbFrame.has_value()) continue;
                auto colorFrame = std::get<std::shared_ptr<dai::ImgFrame>>(rgbFrame.value())->getCvFrame();
                cv::cvtColor(colorFrame, colorFrame, cv::COLOR_BGR2RGB);
                rec.log("rgb",
                        rerun::Image(reinterpret_cast<const uint8_t*>(colorFrame.data),
                                     {static_cast<uint32_t>(colorFrame.cols), static_cast<uint32_t>(colorFrame.rows)},
                                     rerun::datatypes::ColorModel::RGB));
            }
        }
    }
};
int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;
    // Define sources and outputs
    auto colorSockets = pipeline.getDefaultDevice()->getConnectedCameras(dai::CameraSensorType::COLOR);
    auto colorSocket = colorSockets.empty() ? dai::CameraBoardSocket::CAM_A : colorSockets.front();
    auto color = pipeline.create<dai::node::Camera>();
    color->build(colorSocket);
    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO, 30.0f, std::make_pair(640u, 400u));
    auto rerun = pipeline.create<RerunNode>();

    auto rgbd = pipeline.create<dai::node::RGBD>()->build(color, depth, std::make_pair(640, 400), 30.0f);
    rgbd->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);

    // Linking
    rgbd->pcl.link(rerun->inputPCL);
    rgbd->rgbd.link(rerun->inputRGBD);
    pipeline.start();
    auto device = pipeline.getDefaultDevice();
    device->setIrLaserDotProjectorIntensity(0.7);
    pipeline.wait();
}
