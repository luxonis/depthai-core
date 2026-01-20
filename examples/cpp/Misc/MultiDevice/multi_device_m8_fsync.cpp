#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/common/M8FsyncRoles.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

class FPSCounter {
   public:
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> frameTimes;

    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push_back(now);
        if(frameTimes.size() > 100) {
            frameTimes.erase(frameTimes.begin(), frameTimes.begin() + (frameTimes.size() - 100));
        }
    }

    float getFps() {
        if(frameTimes.size() <= 1) {
            return 0.0f;
        }
        // Calculate the FPS
        return float(frameTimes.size() - 1) * 1e6 / std::chrono::duration_cast<std::chrono::microseconds>(frameTimes.back() - frameTimes.front()).count();
    }
};

dai::Node::Output* createPipeline(dai::Pipeline& pipeline, dai::CameraBoardSocket socket, int sensorFps, dai::M8FsyncRole role) {
    std::shared_ptr<dai::node::Camera> cam;
    if(role == dai::M8FsyncRole::MASTER) {
        cam = pipeline.create<dai::node::Camera>()->build(socket, std::nullopt, sensorFps);
    } else {
        cam = pipeline.create<dai::node::Camera>()->build(socket, std::nullopt);
    }

    auto output = cam->requestOutput(std::make_pair(640, 480), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH);
    return output;
}

namespace {
bool running = true;
}

void interruptHandler(int sig) {
    if(running) {
        std::cout << "Interrupted! Exiting..." << std::endl;
        running = false;
    } else {
        std::cout << "Exiting now!" << std::endl;
        exit(0);
    }
}

int main(int argc, char** argv) {
    if(argc < 5) {
        std::cout << "Usage: " << argv[0]
                  << " <target_fps> <recv-all-timeout-sec> <sync-threshold-sec> <initial-sync-timeout-sec> [<device_ip_1> <device_ip_2> [device_ip_3]] ..."
                  << std::endl;
        std::exit(0);
    }

    signal(SIGINT, interruptHandler);

    float targetFps = std::stof(argv[1]);

    int recvAllTimeoutSec = std::stoi(argv[2]);
    float syncThresholdSec = std::stof(argv[3]);
    int initialSyncTimeoutSec = std::stoi(argv[4]);

    std::vector<dai::DeviceInfo> deviceInfos;

    if(argc > 5) {
        for(int i = 5; i < argc; i++) {
            deviceInfos.emplace_back(std::string(argv[i]));
        }
    } else {
        deviceInfos = dai::Device::getAllAvailableDevices();
    }

    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        std::exit(0);
    }

    std::vector<dai::Node::Output*> masterNodes;
    std::vector<std::shared_ptr<dai::MessageQueue>> slaveQueues;
    std::vector<dai::Pipeline> masterPipelines;
    std::vector<dai::Pipeline> slavePipelines;
    std::vector<std::string> deviceIds;

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    std::vector<std::string> outputNames;

    for(auto deviceInfo : deviceInfos) {
        auto pipeline = dai::Pipeline(std::make_shared<dai::Device>(deviceInfo));
        auto device = pipeline.getDefaultDevice();

        auto role = device->getM8FsyncRole();

        std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
        std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
        std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

        auto socket = device->getConnectedCameras()[0];
        if(device->getProductName().find("OAK4-D") != std::string::npos || device->getProductName().find("OAK-4-D") != std::string::npos) {
            socket = device->getConnectedCameras()[1];
        }

        auto outNode = createPipeline(pipeline, socket, targetFps, role);

        if(role == dai::M8FsyncRole::MASTER) {
            device->setM8StrobeEnable(true);
            masterPipelines.push_back(pipeline);
            masterNodes.push_back(outNode);
            std::cout << device->getDeviceId() << " is master" << std::endl;
        } else if(role == dai::M8FsyncRole::SLAVE) {
            slavePipelines.push_back(pipeline);
            slaveQueues.push_back(outNode->createOutputQueue());
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        } else {
            throw std::runtime_error("Don't know how to handle role " + dai::toString(role));
        }

        deviceIds.push_back(deviceInfo.getXLinkDeviceDesc().name);
    }

    if(masterPipelines.size() > 1) {
        throw std::runtime_error("Multiple masters detected!");
    }
    if(masterPipelines.size() == 0) {
        throw std::runtime_error("No master detected!");
    }
    if(slavePipelines.size() < 1) {
        throw std::runtime_error("No slaves detected!");
    }

    auto sync = masterPipelines[0].create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(std::chrono::nanoseconds(long(round(1e9 * 0.5f / targetFps))));
    masterNodes[0]->link(sync->inputs["master"]);
    outputNames.push_back("master");

    for(unsigned long i = 0; i < slaveQueues.size(); i++) {
        auto name = std::string("slave_") + std::to_string(i);
        outputNames.push_back(name);
        auto input_queue = sync->inputs[name].createInputQueue();
        inputQueues.push_back(input_queue);
    }

    auto queue = sync->out.createOutputQueue();

    for(auto p : masterPipelines) {
        p.start();
    }
    for(auto p : slavePipelines) {
        p.start();
    }

    FPSCounter fpsCounter;

    std::optional<std::shared_ptr<dai::MessageGroup>> latestFrameGroup;
    bool firstReceived = false;
    auto startTime = std::chrono::steady_clock::now();
    auto prevReceived = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initialSyncTime;

    bool waitingForInitialSync = true;

    while(running) {
        for(int i = 0; i < slaveQueues.size(); i++) {
            while(slaveQueues[i]->has()) {
                inputQueues[i]->send(slaveQueues[i]->get());
            }
        }

        while(queue->has()) {
            auto syncData = queue->get();
            latestFrameGroup = std::dynamic_pointer_cast<dai::MessageGroup>(syncData);
            if(!firstReceived) {
                firstReceived = true;
                initialSyncTime = std::chrono::steady_clock::now();
            }
            prevReceived = std::chrono::steady_clock::now();
            fpsCounter.tick();
        }

        if(!firstReceived) {
            auto endTime = std::chrono::steady_clock::now();
            auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
            if(elapsedSec >= recvAllTimeoutSec) {
                std::cout << "Timeout: Didn't receive all frames in time: " << elapsedSec << std::endl;
                running = false;
            }
        }

        if(latestFrameGroup.has_value() && size_t(latestFrameGroup.value()->getNumMessages()) == outputNames.size()) {
            std::vector<std::chrono::time_point<std::chrono::steady_clock>> tsValues;
            for(auto name : outputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(name);
                tsValues.push_back(frame->getTimestamp(dai::CameraExposureOffset::END));
            }

            cv::Mat imgs;
            for(int i = 0; i < outputNames.size(); i++) {
                auto msg = latestFrameGroup.value()->get<dai::ImgFrame>(outputNames[i]);
                auto fps = fpsCounter.getFps();
                auto frame = msg->getCvFrame();

                cv::putText(frame, deviceIds[i] + " (" + outputNames[i] + ")", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 127, 255}, 2, cv::LINE_AA);

                cv::putText(frame,
                            "Timestamp: " + std::to_string(1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(tsValues[i].time_since_epoch()).count())
                                + " | FPS: " + std::to_string(fps).substr(0, 5),
                            {20, 80},
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            {255, 0, 50},
                            2,
                            cv::LINE_AA);

                if(i == 0) {
                    imgs = frame;
                } else {
                    cv::hconcat(frame, imgs, imgs);
                }
            }

            auto delta = *std::max_element(tsValues.begin(), tsValues.end()) - *std::min_element(tsValues.begin(), tsValues.end());

            bool syncStatus = std::chrono::duration_cast<std::chrono::seconds>(delta).count() < syncThresholdSec;
            std::string syncStatusStr = (syncStatus) ? "in sync" : "out of sync";

            cv::Scalar color = (syncStatus) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            if(!syncStatus && waitingForInitialSync) {
                auto endTime = std::chrono::steady_clock::now();
                auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - initialSyncTime.value()).count();
                if(elapsedSec >= initialSyncTimeoutSec) {
                    std::cout << "Timeout: Didn't sync frames in time" << std::endl;
                    running = false;
                }
            }

            if(syncStatus && waitingForInitialSync) {
                std::cout << "Sync status: " << syncStatusStr << std::endl;
                waitingForInitialSync = false;
            }

            if(!syncStatus && !waitingForInitialSync) {
                std::cout << "Sync error: Sync lost, threshold exceeded " << std::chrono::duration_cast<std::chrono::microseconds>(delta).count() << " us"
                          << std::endl;
                running = false;
            }

            cv::putText(imgs,
                        syncStatusStr + " | delta = "
                            + std::to_string(1e-3 * float(std::chrono::duration_cast<std::chrono::microseconds>(delta).count())).substr(0, 5) + " ms",
                        {20, 120},
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.7,
                        color,
                        2,
                        cv::LINE_AA);

            cv::imshow("synced_view", imgs);

            latestFrameGroup.reset();
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
