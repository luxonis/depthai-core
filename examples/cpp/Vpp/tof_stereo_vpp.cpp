#include <cmath>
#include <cstddef>
#include <cstdint>
#include <depthai/depthai.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
constexpr float FPS = 30.0f;

struct Options {
    bool showAll = false;
    std::pair<uint32_t, uint32_t> size{1280, 800};
    double maxDepth = 5000.0;
};

Options parseOptions(int argc, char** argv) {
    Options options;
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--show-all") {
            options.showAll = true;
        } else if(arg == "--size" && i + 1 < argc) {
            const std::string value = argv[++i];
            if(value == "640x400") {
                options.size = {640, 400};
            } else if(value == "1280x800") {
                options.size = {1280, 800};
            } else {
                throw std::invalid_argument("--size must be 1280x800 or 640x400");
            }
        } else if(arg == "--max-depth" && i + 1 < argc) {
            const std::string value = argv[++i];
            std::size_t parsed = 0;
            options.maxDepth = std::stod(value, &parsed);
            if(parsed != value.size() || !std::isfinite(options.maxDepth) || options.maxDepth <= 0.0) {
                throw std::invalid_argument("--max-depth must be finite and positive");
            }
        } else {
            std::cout << "Usage: " << argv[0] << " [--show-all] [--size 1280x800|640x400] [--max-depth millimeters]\n";
            throw std::invalid_argument("Unknown or incomplete argument: " + arg);
        }
    }

    return options;
}

void configureVpp(dai::node::Vpp& vpp) {
    auto& config = *vpp.initialConfig;
    config.blending = 0.6f;
    config.distanceGamma = 0.3f;
    config.maxPatchSize = 3;
    config.patchColoringType = dai::VppConfig::PatchColoringType::RANDOM;
    config.uniformPatch = false;
    config.maxFPS = FPS;
    config.maxNumThreads = 1;
    config.injectionParameters.useInjection = false;
    config.injectionParameters.textureThreshold = 10.0f;
}

std::map<std::string, dai::Node::Output*> buildPipeline(dai::Pipeline& pipeline, const Options& options) {
    const auto& size = options.size;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) {
        throw std::runtime_error("This example requires an RVC4 device");
    }
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto rect = pipeline.create<dai::node::Rectification>();
    rect->setRunOnHost(false);
    rect->setOutputSize(size);
    left->requestOutput(size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, FPS, false)->link(rect->input1);
    right->requestOutput(size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, FPS, false)->link(rect->input2);

    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, dai::ToFConfig::Profile::MID_RANGE, FPS);
    tof->setOutputUndistortion(false);
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(false);
    tof->tofBase->depth.link(align->input);
    rect->output1.link(align->inputAlignTo);

    // Linking depth instead of disparity selects depth mode; confidence is optional.
    auto vpp = pipeline.create<dai::node::Vpp>();
    rect->output1.link(vpp->left);
    rect->output2.link(vpp->right);
    align->outputAligned.link(vpp->depth);
    configureVpp(*vpp);

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::FAST_ACCURACY);
    stereo->setRectification(false);
    vpp->leftOut.link(stereo->left);
    vpp->rightOut.link(stereo->right);

    std::map<std::string, dai::Node::Output*> outputs{{"fused_depth", &stereo->depth}};
    if(options.showAll) {
        outputs.insert(
            {{"tof_depth", &tof->tofBase->depth}, {"aligned_depth", &align->outputAligned}, {"vpp_left", &vpp->leftOut}, {"vpp_right", &vpp->rightOut}});
    }
    return outputs;
}
}  // namespace

// Run ToF depth through VPP and StereoDepth on an RVC4 device. Press 'q' to quit.
int main(int argc, char** argv) {
    if(argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [--show-all] [--size 1280x800|640x400] [--max-depth millimeters]\n";
        return 0;
    }
    const auto options = parseOptions(argc, argv);
    dai::Pipeline pipeline;
    const auto outputs = buildPipeline(pipeline, options);
    std::map<std::string, std::shared_ptr<dai::MessageQueue>> queues;
    for(const auto& output : outputs) {
        queues.emplace(output.first, output.second->createOutputQueue(2, false));
    }
    pipeline.start();
    while(pipeline.isRunning()) {
        for(const auto& entry : queues) {
            if(auto frame = entry.second->tryGet<dai::ImgFrame>()) {
                auto img = frame->getCvFrame();
                if(entry.first == "tof_depth" || entry.first == "aligned_depth" || entry.first == "fused_depth") {
                    cv::convertScaleAbs(img, img, 255.0 / options.maxDepth);
                }
                cv::imshow(entry.first, img);
            }
        }
        if(cv::waitKey(1) == 'q') break;
    }
    cv::destroyAllWindows();
    return 0;
}
