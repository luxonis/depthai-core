#include <chrono>
#include <depthai/depthai.hpp>
#include <deque>
#include <opencv2/opencv.hpp>

#include "depthai/capabilities/ImgFrameCapability.hpp"

// Simple FPS counter
class FPSCounter {
    static constexpr size_t MAX_SAMPLES = 100;
    std::array<double, MAX_SAMPLES> frameTimes{};
    size_t head = 0;   // index of next write
    size_t count = 0;  // how many valid samples stored

   public:
    void tick() {
        auto now = std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        frameTimes[head] = now;
        head = (head + 1) % MAX_SAMPLES;
        if(count < MAX_SAMPLES) {
            ++count;
        }
    }

    double getFps() const {
        if(count <= 1) return 0.0;

        size_t newestIndex = (head + MAX_SAMPLES - 1) % MAX_SAMPLES;
        size_t oldestIndex = (newestIndex + MAX_SAMPLES - (count - 1)) % MAX_SAMPLES;

        double newest = frameTimes[newestIndex];
        double oldest = frameTimes[oldestIndex];

        return (count - 1) / (newest - oldest);
    }
};

static float rgbWeight = 0.4f;
static float thermalWeight = 0.6f;
static int staticDepthPlane = 0;

void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = percentRgb / 100.0f;
    thermalWeight = 1.0f - rgbWeight;
}

void updateDepthPlane(int depth, void*) {
    staticDepthPlane = depth;
}

int main() {
    constexpr float FPS = 25.0f;
    const dai::CameraBoardSocket RGB_SOCKET = dai::CameraBoardSocket::CAM_A;
    const std::pair COLOR_RESOLUTION{960u, 540u};

    dai::Pipeline pipeline;

    // Create device and query connected cameras
    auto device = pipeline.getDefaultDevice();
    int thermalWidth = -1, thermalHeight = -1;
    bool thermalFound = false;
    dai::CameraBoardSocket thermalSocket;

    for(const auto& features : device->getConnectedCameraFeatures()) {
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

    auto calibData = device->readCalibration();
    auto rgbDistortion = calibData.getDistortionCoefficients(RGB_SOCKET);
    auto distortionModel = calibData.getDistortionModel(RGB_SOCKET);
    if(distortionModel != dai::CameraModel::Perspective) {
        throw std::runtime_error("Unsupported distortion model for RGB camera. This example supports only Perspective model.");
    }

    // Nodes
    auto camRgb = pipeline.create<dai::node::Camera>()->build(RGB_SOCKET);

    auto thermalCam = pipeline.create<dai::node::Thermal>()->build(thermalSocket, FPS);

    auto sync = pipeline.create<dai::node::Sync>();
    auto align = pipeline.create<dai::node::ImageAlign>();

    auto camRgbOut = camRgb->requestOutput(COLOR_RESOLUTION, std::nullopt, dai::ImgResizeMode::CROP, FPS);

    sync->setSyncThreshold(std::chrono::milliseconds((unsigned int)(3000.f / FPS)));

    // Linking
    align->outputAligned.link(sync->inputs["aligned"]);
    camRgbOut->link(sync->inputs["rgb"]);
    camRgbOut->link(align->inputAlignTo);
    thermalCam->temperature.link(align->input);
    auto out = sync->out.createOutputQueue();
    auto cfgIn = align->inputConfig.createInputQueue();

    // Start pipeline
    pipeline.start();

    // OpenCV windows
    std::string windowName = "rgb-thermal";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1280, 720);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);
    cv::setTrackbarPos("RGB Weight %", windowName, static_cast<int>(rgbWeight * 100));
    cv::createTrackbar("Static Depth Plane [mm]", windowName, nullptr, 2000, updateDepthPlane);

    FPSCounter fpsCounter;

    while(true) {
        auto msgGroup = out->get<dai::MessageGroup>();
        auto frameRgb = std::dynamic_pointer_cast<dai::ImgFrame>(msgGroup->get<dai::ImgFrame>("rgb"));
        auto thermalAligned = std::dynamic_pointer_cast<dai::ImgFrame>(msgGroup->get<dai::ImgFrame>("aligned"));

        cv::Mat frameRgbCv = frameRgb->getCvFrame();
        fpsCounter.tick();

        auto rgbIntrinsics = calibData.getCameraIntrinsics(RGB_SOCKET, frameRgb->getWidth(), frameRgb->getHeight());
        auto rgbIntrinsicsFlat = std::vector<float>();
        for(const auto& row : rgbIntrinsics) {
            rgbIntrinsicsFlat.insert(rgbIntrinsicsFlat.end(), row.begin(), row.end());
        }
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, rgbIntrinsicsFlat.data());
        cv::Mat distCoeffs(rgbDistortion);

        cv::Mat cvFrameUndistorted;
        cv::undistort(frameRgbCv, cvFrameUndistorted, cameraMatrix, distCoeffs);

        // Convert thermal frame
        cv::Mat thermalFrame(thermalAligned->getHeight(), thermalAligned->getWidth(), CV_16FC1, (void*)thermalAligned->getData().data());
        thermalFrame.convertTo(thermalFrame, CV_32F);

        cv::patchNaNs(thermalFrame, cv::mean(thermalFrame)[0]);  // replace NaN with mean

        cv::Mat thermalNorm, colormappedFrame;
        cv::normalize(thermalFrame, thermalNorm, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(thermalNorm, colormappedFrame, cv::COLORMAP_MAGMA);

        // Blend
        cv::Mat blended;
        cv::addWeighted(cvFrameUndistorted, rgbWeight, colormappedFrame, thermalWeight, 0, blended);

        cv::putText(blended, "FPS: " + std::to_string(fpsCounter.getFps()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

        cv::imshow(windowName, blended);
        cv::imshow("aligned thermal", colormappedFrame);
        cv::imshow("undistorted rgb", cvFrameUndistorted);

        int key = cv::waitKey(1);
        if(key == 'q') break;

        auto cfg = align->initialConfig;
        cfg->staticDepthPlane = staticDepthPlane;
        cfgIn->send(cfg);
    }
    return 0;
}
