/**
 * Unified Depth node demo.
 *
 * The Depth node picks an algorithm and backend config automatically from the connected device,
 * target FPS, and/or stereo resolution (Algorithm::AUTO). You can also pin the algorithm yourself
 * via CLI or Depth::build().
 *
 * Supported algorithms: AUTO, STEREO, NEURAL, NEURAL_ASSISTED_STEREO, TOF, GPU_STEREO.
 *
 * Configure the node via CLI to exercise Depth::build() overloads (fps-only, algorithm + fps/res,
 * algorithm + config) or pre-built user stereo cameras.
 */
#include <algorithm>
#include <argparse/argparse.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"

namespace {

cv::Mat colorizeConfidence(const cv::Mat& frame) {
    if(frame.empty() || frame.channels() != 1) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }

    cv::Mat normalized;
    if(frame.depth() == CV_16U) {
        double maxValue = 0.0;
        cv::minMaxLoc(frame, nullptr, &maxValue);
        if(maxValue <= 0.0) {
            return cv::Mat::zeros(frame.size(), CV_8UC3);
        }
        cv::Mat scaled;
        frame.convertTo(scaled, CV_32F, 255.0 / maxValue);
        scaled.convertTo(normalized, CV_8U);
    } else {
        frame.convertTo(normalized, CV_8U);
    }

    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
    return colorized;
}

const std::map<std::string, dai::node::Depth::Algorithm>& algorithmMap() {
    static const std::map<std::string, dai::node::Depth::Algorithm> map = {
        {"auto", dai::node::Depth::Algorithm::AUTO},
        {"stereo", dai::node::Depth::Algorithm::STEREO},
        {"neural", dai::node::Depth::Algorithm::NEURAL},
        {"neural_assisted_stereo", dai::node::Depth::Algorithm::NEURAL_ASSISTED_STEREO},
        {"tof", dai::node::Depth::Algorithm::TOF},
        {"gpu_stereo", dai::node::Depth::Algorithm::GPU_STEREO},
    };
    return map;
}

const char* algorithmName(dai::node::Depth::Algorithm algorithm) {
    for(const auto& [name, value] : algorithmMap()) {
        if(value == algorithm) {
            return name.c_str();
        }
    }
    return "unknown";
}

std::optional<dai::node::Depth::Algorithm> parseAlgorithm(const std::string& name) {
    const auto it = algorithmMap().find(name);
    if(it == algorithmMap().end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<dai::node::Depth::Config> parseConfig(const std::string& name) {
    static const std::map<std::string, dai::DeviceModelZoo> modelMap = {
        {"NEURAL_DEPTH_1248X780", dai::DeviceModelZoo::NEURAL_DEPTH_1248X780},
        {"NEURAL_DEPTH_768X480", dai::DeviceModelZoo::NEURAL_DEPTH_768X480},
        {"NEURAL_DEPTH_576X360", dai::DeviceModelZoo::NEURAL_DEPTH_576X360},
        {"NEURAL_DEPTH_480X300", dai::DeviceModelZoo::NEURAL_DEPTH_480X300},
        {"NEURAL_DEPTH_384X240", dai::DeviceModelZoo::NEURAL_DEPTH_384X240},
        {"NEURAL_DEPTH_1056X660", dai::DeviceModelZoo::NEURAL_DEPTH_1056X660},
        {"NEURAL_DEPTH_960X600", dai::DeviceModelZoo::NEURAL_DEPTH_960X600},
        {"NEURAL_DEPTH_864X540", dai::DeviceModelZoo::NEURAL_DEPTH_864X540},
        {"NEURAL_DEPTH_288X180", dai::DeviceModelZoo::NEURAL_DEPTH_288X180},
        {"NEURAL_DEPTH_192X120", dai::DeviceModelZoo::NEURAL_DEPTH_192X120},
        {"NEURAL_DEPTH_EXTRA_LARGE", dai::DeviceModelZoo::NEURAL_DEPTH_EXTRA_LARGE},
        {"NEURAL_DEPTH_LARGE", dai::DeviceModelZoo::NEURAL_DEPTH_LARGE},
        {"NEURAL_DEPTH_MEDIUM", dai::DeviceModelZoo::NEURAL_DEPTH_MEDIUM},
        {"NEURAL_DEPTH_SMALL", dai::DeviceModelZoo::NEURAL_DEPTH_SMALL},
        {"NEURAL_DEPTH_NANO", dai::DeviceModelZoo::NEURAL_DEPTH_NANO},
    };
    static const std::map<std::string, dai::node::StereoDepth::PresetMode> presetMap = {
        {"FAST_ACCURACY", dai::node::StereoDepth::PresetMode::FAST_ACCURACY},
        {"FAST_DENSITY", dai::node::StereoDepth::PresetMode::FAST_DENSITY},
        {"DEFAULT", dai::node::StereoDepth::PresetMode::DEFAULT},
        {"FACE", dai::node::StereoDepth::PresetMode::FACE},
        {"HIGH_DETAIL", dai::node::StereoDepth::PresetMode::HIGH_DETAIL},
        {"ROBOTICS", dai::node::StereoDepth::PresetMode::ROBOTICS},
        {"DENSITY", dai::node::StereoDepth::PresetMode::DENSITY},
        {"ACCURACY", dai::node::StereoDepth::PresetMode::ACCURACY},
    };

    if(const auto modelIt = modelMap.find(name); modelIt != modelMap.end()) {
        return dai::node::Depth::Config{modelIt->second};
    }
    if(const auto presetIt = presetMap.find(name); presetIt != presetMap.end()) {
        return dai::node::Depth::Config{presetIt->second};
    }
    return std::nullopt;
}

struct CliOptions {
    std::string ip;
    std::optional<float> fps;
    std::optional<uint32_t> width;
    std::optional<uint32_t> height;
    std::optional<std::string> algorithm;
    std::optional<std::string> config;
    bool userCameras{false};
};

std::optional<std::pair<uint32_t, uint32_t>> sizeFromOptions(const CliOptions& options) {
    if(!options.width && !options.height) {
        return std::nullopt;
    }
    if(!options.width || !options.height) {
        throw std::runtime_error("--width and --height must be specified together");
    }
    return std::make_pair(*options.width, *options.height);
}

void buildUserStereoCameras(dai::Pipeline& pipeline, const CliOptions& options) {
    const auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        throw std::runtime_error("Connect a device (host-only pipeline cannot use Depth).");
    }

    const auto stereoPairs = device->getStereoPairs();
    if(stereoPairs.empty()) {
        throw std::runtime_error("This device has no stereo pair; Depth cannot run.");
    }

    const auto stereoPair = stereoPairs[0];
    const auto size = sizeFromOptions(options);
    const std::optional<float> fps = options.fps;

    pipeline.create<dai::node::Camera>()->build(stereoPair.left, size, fps);
    pipeline.create<dai::node::Camera>()->build(stereoPair.right, size, fps);
}

void configureDepth(const std::shared_ptr<dai::node::Depth>& depth, const CliOptions& options) {
    const auto fps = options.userCameras ? std::nullopt : options.fps;
    const auto size = options.userCameras ? std::nullopt : sizeFromOptions(options);
    const auto algorithm = options.algorithm ? parseAlgorithm(*options.algorithm) : std::nullopt;
    const auto config = options.config ? parseConfig(*options.config) : std::nullopt;

    if(options.algorithm && !algorithm) {
        throw std::runtime_error("Invalid --algorithm value");
    }
    if(options.config && !config) {
        throw std::runtime_error("Unknown --config value. Use a DeviceModelZoo or StereoDepth.PresetMode name.");
    }

    if(algorithm && config) {
        depth->build(*algorithm, *config, fps, size);
    } else if(algorithm) {
        depth->build(*algorithm, fps, size);
    } else if(fps && size) {
        depth->build(dai::node::Depth::Algorithm::AUTO, *fps, size);
    } else if(fps) {
        depth->build(*fps);
    } else if(size) {
        depth->build(dai::node::Depth::Algorithm::AUTO, std::nullopt, size);
    }
}

void printResolvedConfig(const dai::node::Depth::Config& config) {
    std::visit(
        [](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::is_same_v<T, std::monostate>) {
                std::cout << "none";
            } else if constexpr(std::is_same_v<T, dai::DeviceModelZoo>) {
                std::cout << static_cast<int>(value);
            } else {
                std::cout << static_cast<int>(value);
            }
        },
        config);
    std::cout << std::endl;
}

dai::Pipeline makePipeline(const CliOptions& options) {
    if(options.ip.empty()) {
        return dai::Pipeline();
    }
    auto device = std::make_shared<dai::Device>(dai::DeviceInfo(options.ip));
    return dai::Pipeline(device);
}

}  // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("unified_depth", "1.0.0");
    program.add_description(
        "Unified Depth node demo.\n\n"
        "The Depth node picks an algorithm and backend config automatically from the connected device, "
        "target FPS, and/or stereo resolution (Algorithm::AUTO). You can also pin the algorithm yourself "
        "via CLI or Depth::build().\n\n"
        "Supported algorithms: AUTO, STEREO, NEURAL, NEURAL_ASSISTED_STEREO, TOF, GPU_STEREO.\n\n"
        "Configure the node via CLI to exercise Depth::build() overloads (fps-only, algorithm + fps/res, "
        "algorithm + config) or pre-built user stereo cameras.");
    program.add_argument("--ip").default_value(std::string("")).help("Device IP for TCP/IP (e.g. PoE)");
    program.add_argument("--fps").scan<'g', float>().help("Stereo camera FPS for Depth.build() or user cameras");
    program.add_argument("--width").scan<'u', uint32_t>().help("Stereo frame width (requires --height)");
    program.add_argument("--height").scan<'u', uint32_t>().help("Stereo frame height (requires --width)");
    program.add_argument("--algorithm").help("Depth backend: auto, stereo, neural, neural_assisted_stereo, tof, gpu_stereo");
    program.add_argument("--config").help("DeviceModelZoo (NEURAL_*) or StereoDepth.PresetMode (DEFAULT, ...)");
    program.add_argument("--user-cameras").flag().help("Pre-build stereo Camera nodes instead of using Depth.build() for fps/res");

    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& err) {
        std::cerr << err.what() << '\n';
        std::cerr << program;
        return EXIT_FAILURE;
    }

    CliOptions options{};
    options.ip = program.get<std::string>("--ip");
    if(program.is_used("--fps")) {
        options.fps = program.get<float>("--fps");
    }
    if(program.is_used("--width")) {
        options.width = program.get<uint32_t>("--width");
    }
    if(program.is_used("--height")) {
        options.height = program.get<uint32_t>("--height");
    }
    if(program.is_used("--algorithm")) {
        options.algorithm = program.get<std::string>("--algorithm");
    }
    if(program.is_used("--config")) {
        options.config = program.get<std::string>("--config");
    }
    options.userCameras = program.get<bool>("--user-cameras");

    if(options.config && !options.algorithm) {
        std::cerr << "--config requires --algorithm\n";
        return EXIT_FAILURE;
    }

    try {
        sizeFromOptions(options);
    } catch(const std::runtime_error& err) {
        std::cerr << err.what() << '\n';
        return EXIT_FAILURE;
    }

    try {
        dai::Pipeline pipeline = makePipeline(options);

        if(options.userCameras) {
            buildUserStereoCameras(pipeline, options);
        }

        auto depthNode = pipeline.create<dai::node::Depth>();
        configureDepth(depthNode, options);

        auto depthQueue = depthNode->depth().createOutputQueue();
        auto confidenceQueue = depthNode->confidence().createOutputQueue();

        pipeline.build();

        std::cout << "Resolved algorithm: " << algorithmName(depthNode->getResolvedAlgorithm()) << std::endl;
        std::cout << "Resolved config: ";
        printResolvedConfig(depthNode->getResolvedConfig());

        pipeline.start();

        while(pipeline.isRunning()) {
            auto depthFrame = depthQueue->get<dai::ImgFrame>();
            auto confidenceFrame = confidenceQueue->get<dai::ImgFrame>();

            if(depthFrame != nullptr) {
                cv::imshow("depth", dai::utility::colorizeDepthFrame(*depthFrame, 500.0f, 12000.0f).getCvFrame());
            }
            if(confidenceFrame != nullptr) {
                cv::imshow("confidence", colorizeConfidence(confidenceFrame->getFrame()));
            }

            if(cv::waitKey(1) == 'q') {
                pipeline.stop();
                break;
            }
        }

        cv::destroyAllWindows();
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
