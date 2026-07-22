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
    // Pick a COLOR-capable socket for the color camera so it does not take the
    // socket the Depth node needs (e.g. the ToF sensor on ToF-only devices, where
    // the default AUTO socket would otherwise grab the only ToF-capable sensor).
    auto colorSocket = dai::CameraBoardSocket::CAM_A;
    for(const auto& features : pipeline.getDefaultDevice()->getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end()) {
            colorSocket = features.socket;
            break;
        }
    }
    auto color = pipeline.create<dai::node::Camera>();
    color->build(colorSocket);
    // The Depth node manages its own stereo cameras and backend internally, so no
    // explicit left/right cameras or StereoDepth node are needed.
    auto depth = pipeline.create<dai::node::Depth>();
    auto rerun = pipeline.create<RerunNode>();

    // RGBD wires the color camera and aligns the Depth node's depth to it
    // internally (using the Depth node's own alignment), so no ImageAlign node is
    // needed.
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
