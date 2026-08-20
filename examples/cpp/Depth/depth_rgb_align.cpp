#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

const char* algorithmName(dai::node::Depth::Algorithm algorithm) {
    switch(algorithm) {
        case dai::node::Depth::Algorithm::AUTO:
            return "auto";
        case dai::node::Depth::Algorithm::STEREO:
            return "stereo";
        case dai::node::Depth::Algorithm::NEURAL:
            return "neural";
        case dai::node::Depth::Algorithm::NEURAL_ASSISTED_STEREO:
            return "neural_assisted_stereo";
        case dai::node::Depth::Algorithm::TOF:
            return "tof";
        case dai::node::Depth::Algorithm::GPU_STEREO:
            return "gpu_stereo";
    }
    return "unknown";
}

void printResolvedConfig(const dai::node::Depth::Config& config) {
    std::visit(
        [](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::is_same_v<T, std::monostate>) {
                std::cout << "none";
            } else {
                std::cout << static_cast<int>(value);
            }
        },
        config);
    std::cout << std::endl;
}

class FPSCounter {
   public:
    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push_back(now);
        while(frameTimes.size() > 10) {
            frameTimes.pop_front();
        }
    }

    float getFps() const {
        if(frameTimes.size() <= 1) return 0.0f;
        auto duration = std::chrono::duration_cast<std::chrono::duration<float>>(frameTimes.back() - frameTimes.front()).count();
        return static_cast<float>(frameTimes.size() - 1) / duration;
    }

   private:
    std::deque<std::chrono::steady_clock::time_point> frameTimes;
};

float rgbWeight = 0.4f;
float depthWeight = 0.6f;

void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<float>(percentRgb) / 100.0f;
    depthWeight = 1.0f - rgbWeight;
}

}  // namespace

int main() {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        throw std::runtime_error("No default device available.");
    }

    std::optional<dai::CameraBoardSocket> colorSocket;
    for(const auto& features : device->getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end()) {
            colorSocket = features.socket;
            break;
        }
    }
    if(!colorSocket.has_value()) {
        throw std::runtime_error("No color camera found on this device to align the depth to.");
    }
    std::cout << "Aligning depth to color camera on socket: " << static_cast<int>(*colorSocket) << std::endl;

    auto depthNode = pipeline.create<dai::node::Depth>();
    auto camRgb = pipeline.create<dai::node::Camera>();
    auto sync = pipeline.create<dai::node::Sync>();

    camRgb->build(*colorSocket);
    auto* rgbOut = camRgb->requestOutput(std::make_pair(640u, 400u), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);

    depthNode->setAlignTo(*rgbOut);

    rgbOut->link(sync->inputs["rgb"]);
    depthNode->depth().link(sync->inputs["depth_aligned"]);

    auto queue = sync->out.createOutputQueue();

    pipeline.build();

    std::cout << "Resolved algorithm: " << algorithmName(depthNode->getResolvedAlgorithm()) << std::endl;
    std::cout << "Resolved config:    ";
    printResolvedConfig(depthNode->getResolvedConfig());

    const std::string windowName = "rgb-depth";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 800, 600);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);
    cv::setTrackbarPos("RGB Weight %", windowName, static_cast<int>(rgbWeight * 100));

    FPSCounter fpsCounter;

    pipeline.start();

    while(pipeline.isRunning()) {
        auto messageGroup = queue->get<dai::MessageGroup>();
        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("depth_aligned");

        if(frameDepth != nullptr) {
            cv::Mat cvFrame = frameRgb->getCvFrame();
            cv::Mat alignedDepthColorized = dai::utility::colorizeDepthFrame(*frameDepth).getCvFrame();
            cv::imshow("Depth aligned", alignedDepthColorized);

            if(cvFrame.channels() == 1) {
                cv::cvtColor(cvFrame, cvFrame, cv::COLOR_GRAY2BGR);
            }

            cv::Mat blended;
            cv::addWeighted(cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0, blended);
            cv::putText(blended, "FPS: " + std::to_string(fpsCounter.getFps()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::imshow(windowName, blended);
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    pipeline.stop();
    return 0;
}
