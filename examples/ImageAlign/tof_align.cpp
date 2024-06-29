#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

constexpr auto FPS = 30.0;

constexpr auto RGB_SOCKET = dai::CameraBoardSocket::CAM_B;
constexpr auto TOF_SOCKET = dai::CameraBoardSocket::CAM_A;
constexpr auto ALIGN_SOCKET = RGB_SOCKET;

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

double rgbWeight = 0.4;
double depthWeight = 0.6;

void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<double>(percentRgb) / 100.0;
    depthWeight = 1.0 - rgbWeight;
}

int main() {
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto camTof = pipeline.create<dai::node::Camera>();
    auto tof = pipeline.create<dai::node::ToF>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto align = pipeline.create<dai::node::ImageAlign>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    camTof->setFps(FPS);
    camTof->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camTof->setBoardSocket(TOF_SOCKET);

    camRgb->setBoardSocket(RGB_SOCKET);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
    camRgb->setFps(FPS);
    camRgb->setIspScale(1, 2);

    xout->setStreamName("out");

    sync->setSyncThreshold(std::chrono::milliseconds(static_cast<int>(1000 / FPS)));

    camRgb->isp.link(sync->inputs["rgb"]);
    camTof->raw.link(tof->input);
    tof->depth.link(align->input);
    align->outputAligned.link(sync->inputs["depth_aligned"]);
    camRgb->isp.link(align->inputAlignTo);
    sync->out.link(xout->input);

    dai::Device device(pipeline);
    auto queue = device.getOutputQueue("out", 8, false);

    FPSCounter fpsCounter;

    std::string rgbDepthWindowName = "rgb-depth";
    cv::namedWindow(rgbDepthWindowName);
    cv::createTrackbar("RGB Weight %", rgbDepthWindowName, nullptr, 100, updateBlendWeights);

    while(true) {
        auto messageGroup = queue->get<dai::MessageGroup>();
        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("depth_aligned");

        if(frameRgb && frameDepth) {
            auto cvFrame = frameRgb->getCvFrame();
            auto alignedDepthColorized = colorizeDepth(frameDepth->getFrame());

            cv::putText(alignedDepthColorized,
                        "FPS: " + std::to_string(fpsCounter.getFps()),
                        cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1,
                        cv::Scalar(255, 255, 255),
                        2);

            cv::imshow("depth", alignedDepthColorized);

            cv::Mat blended;
            cv::addWeighted(cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0, blended);
            cv::imshow(rgbDepthWindowName, blended);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }

    return 0;
}
