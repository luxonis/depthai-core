#include <chrono>
#include <opencv2/opencv.hpp>
#include <queue>

#include "depthai/depthai.hpp"

constexpr auto FPS = 30.0;
constexpr auto RGB_SOCKET = dai::CameraBoardSocket::CAM_A;
constexpr auto LEFT_SOCKET = dai::CameraBoardSocket::CAM_B;
constexpr auto RIGHT_SOCKET = dai::CameraBoardSocket::CAM_C;
constexpr auto ALIGN_SOCKET = LEFT_SOCKET;

constexpr auto COLOR_RESOLUTION = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
constexpr auto LEFT_RIGHT_RESOLUTION = dai::MonoCameraProperties::SensorResolution::THE_400_P;
constexpr auto ISP_SCALE = 3;

class FPSCounter {
   public:
    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push(now);
        if(frameTimes.size() > 10) {
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
double depthWeight = 0.6;

void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<double>(percentRgb) / 100.0;
    depthWeight = 1.0 - rgbWeight;
}

cv::Mat colorizeDepth(const cv::Mat& frameDepth) {
    cv::Mat invalidMask = (frameDepth == 0);
    cv::Mat depthFrameColor;
    try {
        double minDepth = 0.0;
        double maxDepth = 0.0;
        cv::minMaxIdx(frameDepth, &minDepth, &maxDepth, nullptr, nullptr, ~invalidMask);
        if(minDepth == maxDepth) {
            depthFrameColor = cv::Mat::zeros(frameDepth.size(), CV_8UC3);
            return depthFrameColor;
        }
        cv::Mat logDepth;
        frameDepth.convertTo(logDepth, CV_32F);
        cv::log(logDepth, logDepth);
        logDepth.setTo(log(minDepth), invalidMask);
        cv::normalize(logDepth, logDepth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(logDepth, depthFrameColor, cv::COLORMAP_JET);
        depthFrameColor.setTo(cv::Scalar(0, 0, 0), invalidMask);
    } catch(const std::exception& e) {
        depthFrameColor = cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }
    return depthFrameColor;
}

int main() {
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto out = pipeline.create<dai::node::XLinkOut>();
    auto align = pipeline.create<dai::node::ImageAlign>();

    left->setResolution(LEFT_RIGHT_RESOLUTION);
    left->setBoardSocket(LEFT_SOCKET);
    left->setFps(FPS);

    right->setResolution(LEFT_RIGHT_RESOLUTION);
    right->setBoardSocket(RIGHT_SOCKET);
    right->setFps(FPS);

    camRgb->setBoardSocket(RGB_SOCKET);
    camRgb->setResolution(COLOR_RESOLUTION);
    camRgb->setFps(FPS);
    camRgb->setIspScale(1, ISP_SCALE);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setDepthAlign(LEFT_SOCKET);

    out->setStreamName("out");

    sync->setSyncThreshold(std::chrono::milliseconds(static_cast<int>((1 / FPS) * 1000.0 * 0.5)));

    // Linking
    camRgb->isp.link(sync->inputs["rgb"]);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->depth.link(align->input);
    align->outputAligned.link(sync->inputs["depth_aligned"]);
    camRgb->isp.link(align->inputAlignTo);
    sync->out.link(out->input);

    dai::Device device(pipeline);
    auto queue = device.getOutputQueue("out", 8, false);

    FPSCounter fpsCounter;

    std::string windowName = "rgb-depth";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1280, 720);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);

    while(true) {
        auto messageGroup = queue->get<dai::MessageGroup>();
        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("depth_aligned");

        if(frameRgb && frameDepth) {
            auto cvFrame = frameRgb->getCvFrame();

            // Colorize the aligned depth
            auto alignedDepthColorized = colorizeDepth(frameDepth->getCvFrame());

            cv::imshow("Depth aligned", alignedDepthColorized);

            cv::Mat blended;
            cv::addWeighted(cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0, blended);
            cv::putText(blended, "FPS: " + std::to_string(fpsCounter.getFps()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::imshow(windowName, blended);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }

    return 0;
}
