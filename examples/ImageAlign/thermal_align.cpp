#include <chrono>
#include <opencv2/opencv.hpp>
#include <queue>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

constexpr auto FPS = 25.0;
constexpr auto RGB_SOCKET = dai::CameraBoardSocket::CAM_A;
constexpr auto COLOR_RESOLUTION = dai::ColorCameraProperties::SensorResolution::THE_1080_P;

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
double thermalWeight = 0.6;
int staticDepthPlane = 0;

void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<double>(percentRgb) / 100.0;
    thermalWeight = 1.0 - rgbWeight;
}

void updateDepthPlane(int depth, void*) {
    staticDepthPlane = depth;
}

cv::Mat createNaNMask(const cv::Mat& frame) {
    cv::Mat nanMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    for(int r = 0; r < frame.rows; ++r) {
        for(int c = 0; c < frame.cols; ++c) {
            if(std::isnan(frame.at<float>(r, c))) {
                nanMask.at<uchar>(r, c) = 255;
            }
        }
    }
    return nanMask;
}

int main() {
    dai::Device device;
    int thermalWidth = -1, thermalHeight = -1;
    bool thermalFound = false;
    dai::CameraBoardSocket thermalSocket;

    for(const auto& features : device.getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::THERMAL) != features.supportedTypes.end()) {
            thermalFound = true;
            thermalSocket = features.socket;
            thermalWidth = features.width;
            thermalHeight = features.height;
            break;
        }
    }

    if(!thermalFound) {
        throw std::runtime_error("No thermal camera found!");
    }

    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto thermalCam = pipeline.create<dai::node::Camera>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto out = pipeline.create<dai::node::XLinkOut>();
    auto align = pipeline.create<dai::node::ImageAlign>();
    auto cfgIn = pipeline.create<dai::node::XLinkIn>();

    thermalCam->setBoardSocket(thermalSocket);
    thermalCam->setFps(FPS);

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
    thermalCam->raw.link(align->input);
    sync->out.link(out->input);
    cfgIn->out.link(align->inputConfig);

    device.startPipeline(pipeline);
    auto queue = device.getOutputQueue("out", 8, false);
    auto cfgQ = device.getInputQueue("config");

    FPSCounter fpsCounter;

    std::string windowName = "rgb-thermal";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1280, 720);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);
    cv::createTrackbar("Static Depth Plane [mm]", windowName, nullptr, 2000, updateDepthPlane);

    while(true) {
        auto messageGroup = queue->get<dai::MessageGroup>();
        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto thermalAligned = messageGroup->get<dai::ImgFrame>("aligned");

        if(frameRgb && thermalAligned) {
            auto frameRgbCv = frameRgb->getCvFrame();
            auto thermalFrame = thermalAligned->getFrame(true);

            // Colorize the aligned depth
            cv::Mat mask;
            cv::Mat colormappedFrame;
            cv::Mat thermalFrameFloat;
            thermalFrame.convertTo(thermalFrameFloat, CV_32F);
            // Get a mask for the nan values
            mask = createNaNMask(thermalFrameFloat);
            auto meanValue = cv::mean(thermalFrameFloat, ~mask);
            thermalFrameFloat.setTo(meanValue, mask);
            cv::normalize(thermalFrameFloat, thermalFrameFloat, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::applyColorMap(thermalFrameFloat, colormappedFrame, cv::COLORMAP_MAGMA);

            colormappedFrame.setTo(cv::Scalar(0, 0, 0), mask);

            cv::Mat blended;
            cv::addWeighted(frameRgbCv, rgbWeight, colormappedFrame, thermalWeight, 0, blended);
            cv::putText(blended, "FPS: " + std::to_string(fpsCounter.getFps()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
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