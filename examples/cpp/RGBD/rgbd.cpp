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
                auto colorFrame = rgbdIn->getRGBFrame()->getCvFrame();
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
    auto left = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto color = pipeline.create<dai::node::Camera>();
    std::shared_ptr<dai::node::ImageAlign> align;
    auto rerun = pipeline.create<RerunNode>();
    color->build();

    left->build(dai::CameraBoardSocket::CAM_B);
    right->build(dai::CameraBoardSocket::CAM_C);
    stereo->setSubpixel(true);
    stereo->setExtendedDisparity(false);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig->setLeftRightCheckThreshold(10);
    stereo->initialConfig->postProcessing.thresholdFilter.maxRange = 10000;
    rgbd->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);

    left->requestOutput(std::pair<int, int>(640, 400))->link(stereo->left);
    right->requestOutput(std::pair<int, int>(640, 400))->link(stereo->right);

    auto platform = pipeline.getDefaultDevice()->getPlatform();
    if(platform == dai::Platform::RVC4) {
        auto* out = color->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i);
        out->link(rgbd->inColor);
        align = pipeline.create<dai::node::ImageAlign>();
        stereo->depth.link(align->input);
        out->link(align->inputAlignTo);
        align->outputAligned.link(rgbd->inDepth);
    } else {
        auto* out = color->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, 30, true);
        out->link(rgbd->inColor);
        out->link(stereo->inputAlignTo);
        stereo->depth.link(rgbd->inDepth);
    }

    // Linking
    rgbd->pcl.link(rerun->inputPCL);
    rgbd->rgbd.link(rerun->inputRGBD);
    pipeline.start();
    auto device = pipeline.getDefaultDevice();
    device->setIrLaserDotProjectorIntensity(0.7);
    pipeline.wait();
}
