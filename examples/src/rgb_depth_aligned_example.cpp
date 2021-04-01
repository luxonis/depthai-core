#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static constexpr bool downscaleColor = true;

int main() {
    using namespace std;

    dai::Pipeline p;
    std::vector<std::string> queueNames;

    auto cam = p.create<dai::node::ColorCamera>();
    cam->setBoardSocket(dai::CameraBoardSocket::RGB);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    if(downscaleColor) cam->setIspScale(2, 3);
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    cam->initialControl.setManualFocus(130);

    auto rgbOut = p.create<dai::node::XLinkOut>();
    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    cam->isp.link(rgbOut->input);

    auto left = p.create<dai::node::MonoCamera>();
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);

    auto right = p.create<dai::node::MonoCamera>();
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    auto stereo = p.create<dai::node::StereoDepth>();
    stereo->setConfidenceThreshold(200);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    left->out.link(stereo->left);
    right->out.link(stereo->right);

    auto depthOut = p.create<dai::node::XLinkOut>();
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");
    // Currently we use the 'disparity' output. TODO 'depth'
    stereo->disparity.link(depthOut->input);

    // Connect to device
    dai::Device d(p);
    d.startPipeline();

    // Sets queues size and behavior
    for(const auto& name : queueNames) {
        d.getOutputQueue(name, 4, false);
    }

    std::unordered_map<std::string, cv::Mat> frame;

    while(1) {
        std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

        auto queueEvents = d.getQueueEvents(queueNames);
        for(const auto& name : queueEvents) {
            auto packets = d.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
            auto count = packets.size();
            if(count > 0) {
                latestPacket[name] = packets[count - 1];
            }
        }

        for(const auto& name : queueNames) {
            if(latestPacket.find(name) != latestPacket.end()) {
                if(name == "depth") {
                    frame[name] = latestPacket[name]->getFrame();
                    // Optional, extend range 0..95 -> 0..255, for a better visualisation
                    if(1) frame[name].convertTo(frame[name], CV_8UC1, 255. / 95);
                    // Optional, apply false colorization
                    if(1) cv::applyColorMap(frame[name], frame[name], cv::COLORMAP_HOT);
                } else {
                    frame[name] = latestPacket[name]->getCvFrame();
                }

                cv::imshow(name, frame[name]);
            }
        }

        // Blend when both received
        if(frame.find("rgb") != frame.end() && frame.find("depth") != frame.end()) {
            // Need to have both frames in BGR format before blending
            if(frame["depth"].channels() < 3) {
                cv::cvtColor(frame["depth"], frame["depth"], cv::COLOR_GRAY2BGR);
            }
            cv::Mat blended;
            cv::addWeighted(frame["rgb"], 0.6, frame["depth"], 0.4, 0, blended);
            cv::imshow("rgb-depth", blended);
            frame.clear();
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
}
