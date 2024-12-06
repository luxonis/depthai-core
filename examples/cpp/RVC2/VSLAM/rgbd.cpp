#include "depthai/depthai.hpp"
#include "rerun.hpp"

class RerunNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, RerunNode> {
   public:
    constexpr static const char* NAME = "RerunNode";

   public:
    void build() {}

    Input inputPCL{*this, {.name = "inPCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};

    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_static("world", rerun::ViewCoordinates::FLU);
        rec.log("world/ground", rerun::Boxes3D::from_half_sizes({{3.f, 3.f, 0.00001f}}));
        while(isRunning()) {
            auto pclIn = inputPCL.tryGet<dai::PointCloudData>();
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
            }
        }
    }
};
int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;
    // Define sources and outputs
    int fps = 30;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto color = pipeline.create<dai::node::Camera>();
    auto rerun = pipeline.create<RerunNode>();
    stereo->setExtendedDisparity(false);
    color->build();

    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    right->setCamera("right");
    right->setFps(fps);
    stereo->setSubpixel(true);
    stereo->setExtendedDisparity(false);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::CENTER);

    auto *out = color->requestOutput(std::pair<int, int>(1280, 720));
    left->out.link(stereo->left);
    right->out.link(stereo->right);

    stereo->depth.link(rgbd->inDepth);
    out->link(rgbd->inColor);

    // Linking
    rgbd->pointCloud.link(rerun->inputPCL);
    pipeline.start();
    auto device = pipeline.getDefaultDevice();
    device->setIrLaserDotProjectorIntensity(0.7);
    pipeline.wait();
}
