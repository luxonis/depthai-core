#include <chrono>
#include <opencv2/opencv.hpp>
#include <queue>

#include "depthai/depthai.hpp"

constexpr auto FPS = 30.0;

constexpr auto RGB_SOCKET = dai::CameraBoardSocket::CAM_A;
constexpr auto LEFT_SOCKET = dai::CameraBoardSocket::CAM_B;
constexpr auto ALIGN_SOCKET = LEFT_SOCKET;

constexpr auto COLOR_RESOLUTION = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
constexpr auto LEFT_RIGHT_RESOLUTION = dai::MonoCameraProperties::SensorResolution::THE_720_P;

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

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto out = pipeline.create<dai::node::XLinkOut>();
    auto align = pipeline.create<dai::node::ImageAlign>();
    auto cfgIn = pipeline.create<dai::node::XLinkIn>();

    left->setResolution(LEFT_RIGHT_RESOLUTION);
    left->setBoardSocket(LEFT_SOCKET);
    left->setFps(FPS);

    camRgb->setBoardSocket(RGB_SOCKET);
    camRgb->setResolution(COLOR_RESOLUTION);
    camRgb->setFps(FPS);
    camRgb->setIspScale(1, 3);

    out->setStreamName("out");

    sync->setSyncThreshold(std::chrono::milliseconds(static_cast<int>((1 / FPS) * 1000.0 * 0.5)));

    cfgIn->setStreamName("config");

    align->outputAligned.link(sync->inputs["aligned"]);
    camRgb->isp.link(sync->inputs["rgb"]);
    camRgb->isp.link(align->inputAlignTo);
    left->out.link(align->input);
    sync->out.link(out->input);
    cfgIn->out.link(align->inputConfig);

    dai::Device device(pipeline);
    auto queue = device.getOutputQueue("out", 8, false);
    auto cfgQ = device.getInputQueue("config");

    FPSCounter fpsCounter;

    std::string windowName = "rgb-left";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1280, 720);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);
    cv::createTrackbar("Static Depth Plane [mm]", windowName, nullptr, 2000, updateDepthPlane);

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

        auto cfg = align->initialConfig.get();
        cfg.staticDepthPlane = staticDepthPlane;
        auto alignConfig = std::make_shared<dai::ImageAlignConfig>();
        alignConfig->set(cfg);
        cfgQ->send(alignConfig);
    }

    return 0;
}
