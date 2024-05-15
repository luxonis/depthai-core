#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{true};
static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_720_P;

static float rgbWeight = 0.4f;
static float depthWeight = 0.6f;

static void updateBlendWeights(int percentRgb, void* ctx) {
    rgbWeight = float(percentRgb) / 100.f;
    depthWeight = 1.f - rgbWeight;
}

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    dai::Device device;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto depthAlign = pipeline.create<dai::node::DepthAlign>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();
    auto alignedDepthOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");
    alignedDepthOut->setStreamName("depth_aligned");
    queueNames.push_back("depth_aligned");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    camRgb->setIspScale(2, 3);
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    try {
        auto calibData = device.readCalibration2();
        auto lensPosition = calibData.getLensPosition(dai::CameraBoardSocket::CAM_A);
        if(lensPosition) {
            camRgb->initialControl.setManualFocus(lensPosition);
        }
    } catch(const std::exception& ex) {
        std::cout << ex.what() << std::endl;
        return 1;
    }

    left->setResolution(monoRes);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setCamera("right");
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->depth.link(depthAlign->inputDepth);

    depthAlign->outputDepthAlign.link(alignedDepthOut->input);
    depthAlign->properties.alignTo = dai::CameraBoardSocket::CAM_A;
    stereo->depth.link(depthOut->input);

    // Connect to device and start pipeline
    device.startPipeline(pipeline);

    // Sets queues size and behavior
    for(const auto& name : queueNames) {
        device.getOutputQueue(name, 4, false);
    }

    std::unordered_map<std::string, cv::Mat> frame;

    auto rgbWindowName = "rgb";
    auto depthWindowName = "depth";
    auto blendedWindowName = "rgb-depth";
    auto alignedDepth = "depth_aligned";
    cv::namedWindow(rgbWindowName);
    cv::namedWindow(depthWindowName);
    cv::namedWindow(blendedWindowName);
    cv::namedWindow(alignedDepth);
    int defaultValue = (int)(rgbWeight * 100);
    cv::createTrackbar("RGB Weight %", blendedWindowName, &defaultValue, 100, updateBlendWeights);

    while(true) {
        std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

        auto queueEvents = device.getQueueEvents(queueNames);
        for(const auto& name : queueEvents) {
            auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
            auto count = packets.size();
            if(count > 0) {
                latestPacket[name] = packets[count - 1];
            }
        }

        for(const auto& name : queueNames) {
            if(latestPacket.find(name) != latestPacket.end()) {
                if(name == depthWindowName) {
                    frame[name] = latestPacket[name]->getFrame();
                } else if (name == rgbWindowName) {
                    frame[name] = latestPacket[name]->getCvFrame();
                }
                else {
                    frame[name] = latestPacket[name]->getFrame();
                }


                cv::imshow(name, frame[name]);
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
