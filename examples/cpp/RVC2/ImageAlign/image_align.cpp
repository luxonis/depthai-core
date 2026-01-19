#include <chrono>
#include <opencv2/opencv.hpp>
#include <queue>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

constexpr auto FPS = 30.0;

class FPSCounter {
   public:
    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push(now);
        if(frameTimes.size() > 100) {
            frameTimes.pop();
        }
    }

    double getFps() {
        if(frameTimes.size() <= 1) return 0;
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(frameTimes.back() - frameTimes.front()).count();
        return (frameTimes.size() - 1) * 1000.0 / duration;
    }

   private:
    std::queue<std::chrono::steady_clock::time_point> frameTimes;
};

double rgbWeight = 0.4;
double leftWeight = 0.6;
int staticDepthPlane = 0;

void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<double>(percentRgb) / 100.0;
    leftWeight = 1.0 - rgbWeight;
}

void updateDepthPlane(int depth, void*) {
    staticDepthPlane = depth;
}

int main() {
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::nullopt, FPS);
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, FPS);
    auto sync = pipeline.create<dai::node::Sync>();
    auto align = pipeline.create<dai::node::ImageAlign>();

    sync->setSyncThreshold(std::chrono::milliseconds(static_cast<int>((1 / FPS) * 1000.0 * 0.5)));

    align->outputAligned.link(sync->inputs["aligned"]);
    auto* RGBOutput = camRgb->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, std::nullopt, true);
    RGBOutput->link(align->inputAlignTo);
    RGBOutput->link(sync->inputs["rgb"]);
    left->requestOutput(std::make_pair(640, 400))->link(align->input);
    auto queue = sync->out.createOutputQueue();

    auto alignQ = align->inputConfig.createInputQueue();

    FPSCounter fpsCounter;

    std::string windowName = "rgb-left";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1280, 720);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);
    cv::createTrackbar("Static Depth Plane [mm]", windowName, nullptr, 2000, updateDepthPlane);
    pipeline.start();
    while(true) {
        auto messageGroup = queue->get<dai::MessageGroup>();
        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto leftAligned = messageGroup->get<dai::ImgFrame>("aligned");

        if(frameRgb && leftAligned) {
            auto frameRgbCv = frameRgb->getCvFrame();
            auto leftCv = leftAligned->getCvFrame();

            if(leftCv.channels() == 1) {
                cv::cvtColor(leftCv, leftCv, cv::COLOR_GRAY2BGR);
            }
            if(leftCv.size() != frameRgbCv.size()) {
                cv::resize(leftCv, leftCv, frameRgbCv.size());
            }

            cv::Mat blended;
            cv::addWeighted(frameRgbCv, rgbWeight, leftCv, leftWeight, 0, blended);
            cv::imshow(windowName, blended);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }

        auto cfg = align->initialConfig;
        auto alignConfig = std::make_shared<dai::ImageAlignConfig>();
        alignConfig = cfg;
        alignConfig->staticDepthPlane = staticDepthPlane;
        alignQ->send(alignConfig);
    }

    return 0;
}
