// A custom host node consuming streams from several devices in ONE dai::Pipeline.
//
// The node discovers which device feeds each of its inputs with
// InputMap::getSourceDevices() (resolved at pipeline build) and composes a mosaic.
// It keeps running when a device disappears: the lost device's tile freezes and is
// labeled OFFLINE while the other tiles keep updating (partial operation).

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>

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

        std::map<std::string, cv::Mat> latest;
        while(mainLoop()) {
            bool anyNew = false;
            for(auto& entry : inputs) {
                const auto& name = entry.first.second;
                if(auto frame = entry.second.tryGet<dai::ImgFrame>()) {
                    latest[name] = frame->getCvFrame();
                    anyNew = true;
                }
            }
            if(!anyNew) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            std::vector<cv::Mat> tiles;
            for(auto& entry : latest) {
                auto tile = entry.second.clone();
                auto device = sourceDevices[entry.first];
                std::string label = entry.first;
                if(device != nullptr && pipeline.getDeviceState(device) != dai::DeviceState::RUNNING) {
                    label += " [OFFLINE]";
                    cv::cvtColor(tile, tile, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(tile, tile, cv::COLOR_GRAY2BGR);
                }
                cv::putText(tile, label, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 127, 255}, 2, cv::LINE_AA);
                tiles.push_back(tile);
            }
            cv::Mat mosaic;
            cv::hconcat(tiles, mosaic);

            auto outFrame = std::make_shared<dai::ImgFrame>();
            outFrame->setCvFrame(mosaic, dai::ImgFrame::Type::BGR888i);
            out.send(outFrame);
        }
    }
};

int main(int argc, char** argv) {
    signal(SIGINT, [](int) { running = false; });

    std::vector<dai::DeviceInfo> deviceInfos;
    if(argc >= 2) {
        for(int i = 1; i < argc; i++) deviceInfos.emplace_back(argv[i]);
    } else {
        deviceInfos = dai::Device::getAllAvailableDevices();
    }
    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        return 0;
    }

    dai::Pipeline pipeline(false);
    auto mosaic = pipeline.create<MosaicNode>();

    for(auto& info : deviceInfos) {
        auto device = pipeline.addDevice(info);
        auto camera = pipeline.create<dai::node::Camera>(device)->build(dai::CameraBoardSocket::CAM_A);
        camera->requestOutput(std::make_pair(640, 400))->link(mosaic->inputs[device->getDeviceId()]);
    }

    auto queue = mosaic->out.createOutputQueue();
    pipeline.start();

    const char* displayEnv = std::getenv("DISPLAY");
    const bool display = displayEnv != nullptr && displayEnv[0] != '\0';
    while(running && pipeline.isRunning()) {
        bool hasTimedOut = false;
        auto frame = queue->get<dai::ImgFrame>(std::chrono::milliseconds(500), hasTimedOut);
        if(frame == nullptr) continue;
        if(display) {
            cv::imshow("multi_device_host_node", frame->getCvFrame());
            if(cv::waitKey(1) == 'q') break;
        } else {
            std::cout << "Mosaic frame " << frame->getWidth() << "x" << frame->getHeight() << std::endl;
        }
    }

    pipeline.stop();
    return 0;
}
