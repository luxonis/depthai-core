

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#include "depthai/depthai.hpp"

static std::atomic_bool running{true};

class MosaicNode : public dai::node::CustomThreadedNode<MosaicNode> {
   public:
    InputMap inputs{*this, "inputs", dai::Node::InputDescription{"", dai::Node::DEFAULT_GROUP, false, 4, {{{dai::DatatypeEnum::ImgFrame, true}}}, false}};
    Output out = dai::Node::Output{*this, {}};

    void run() override {
        // Which device produces each input - valid after pipeline build
        auto sourceDevices = inputs.getSourceDevices();
        auto pipeline = getParentPipeline();

        struct Latest {
            cv::Mat image;
            std::chrono::steady_clock::time_point timestamp;
        };
        std::map<std::string, Latest> latest;
        while(mainLoop()) {
            bool anyNew = false;
            for(auto& entry : inputs) {
                const auto& name = entry.first.second;
                if(auto frame = entry.second.tryGet<dai::ImgFrame>()) {
                    latest[name] = {frame->getCvFrame(), frame->getTimestamp()};
                    anyNew = true;
                }
            }
            if(!anyNew) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            std::vector<cv::Mat> tiles;
            std::vector<std::chrono::steady_clock::time_point> liveTimestamps;
            for(auto& entry : latest) {
                auto tile = entry.second.image.clone();
                const auto timestamp = entry.second.timestamp;
                auto device = sourceDevices[entry.first];
                std::ostringstream label;
                label << entry.first << "  " << std::fixed << std::setprecision(3) << std::chrono::duration<double>(timestamp.time_since_epoch()).count()
                      << " s";
                if(device != nullptr && pipeline.getDeviceState(device) != dai::DeviceState::RUNNING) {
                    label << " [OFFLINE]";
                    cv::cvtColor(tile, tile, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(tile, tile, cv::COLOR_GRAY2BGR);
                } else {
                    liveTimestamps.push_back(timestamp);
                }
                cv::putText(tile, label.str(), {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 127, 255}, 2, cv::LINE_AA);
                tiles.push_back(tile);
            }
            cv::Mat mosaic;
            cv::hconcat(tiles, mosaic);
            if(liveTimestamps.size() >= 2) {
                const auto [minTs, maxTs] = std::minmax_element(liveTimestamps.begin(), liveTimestamps.end());
                std::ostringstream diff;
                diff << "max diff = " << std::fixed << std::setprecision(2) << std::chrono::duration<double, std::milli>(*maxTs - *minTs).count() << " ms";
                cv::putText(mosaic, diff.str(), {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2, cv::LINE_AA);
            }

            auto outFrame = std::make_shared<dai::ImgFrame>();
            outFrame->setCvFrame(mosaic, dai::ImgFrame::Type::BGR888i);
            out.send(outFrame);
        }
    }
};

int main(int argc, char** argv) {
    signal(SIGINT, [](int) { running = false; });

    // Usage: multi_device_stream [--stop-on-device-loss] [device_1 device_2 ...]
    bool stopOnDeviceLoss = false;
    std::vector<dai::DeviceInfo> deviceInfos;
    for(int i = 1; i < argc; i++) {
        const std::string arg = argv[i];
        if(arg == "--stop-on-device-loss") {
            stopOnDeviceLoss = true;
        } else {
            deviceInfos.emplace_back(arg);
        }
    }
    if(deviceInfos.empty()) {
        deviceInfos = dai::Device::getAllAvailableDevices();
    }
    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        return 0;
    }

    dai::Pipeline pipeline(false);
    // Off by default (partial operation)
    pipeline.setStopOnDeviceLoss(stopOnDeviceLoss);
    auto mosaic = pipeline.create<MosaicNode>();

    for(auto& info : deviceInfos) {
        auto device = pipeline.addDevice(info);
        auto camera = pipeline.create<dai::node::Camera>(device)->build(dai::CameraBoardSocket::CAM_A);
        camera->requestOutput(std::make_pair(640, 400))->link(mosaic->inputs[device->getDeviceId()]);
    }

    auto queue = mosaic->out.createOutputQueue();
    pipeline.start();

    while(running && pipeline.isRunning()) {
        bool hasTimedOut = false;
        std::shared_ptr<dai::ImgFrame> frame;
        try {
            frame = queue->get<dai::ImgFrame>(std::chrono::milliseconds(500), hasTimedOut);
        } catch(const dai::MessageQueue::QueueException&) {
            // The pipeline stopped itself after a device loss
            std::cout << "Pipeline stopped - a device was lost" << std::endl;
            break;
        }
        if(frame == nullptr) continue;
        cv::imshow("multi_device_stream", frame->getCvFrame());
        if(cv::waitKey(1) == 'q') break;
    }

    pipeline.stop();
    return 0;
}
