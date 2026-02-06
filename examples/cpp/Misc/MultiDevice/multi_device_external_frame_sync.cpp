#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
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

dai::Node::Output* createPipeline(std::shared_ptr<dai::Pipeline> pipeline, dai::CameraBoardSocket socket, float sensorFps, dai::ExternalFrameSyncRole role) {
    std::shared_ptr<dai::node::Camera> cam;
    if(role == dai::ExternalFrameSyncRole::MASTER) {
        cam = pipeline->create<dai::node::Camera>()->build(socket, std::nullopt, sensorFps);
    } else {
        cam = pipeline->create<dai::node::Camera>()->build(socket, std::nullopt);
    }

    auto output = cam->requestOutput(std::make_pair(640, 480), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH);
    return output;
}

std::shared_ptr<dai::node::Sync> createSyncNode(std::shared_ptr<dai::Pipeline>& masterPipeline,
                                                std::map<std::string, dai::Node::Output*>& masterNode,
                                                std::chrono::nanoseconds syncThreshold,
                                                std::vector<std::string>& outputNames,
                                                std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                                                std::map<std::string, std::shared_ptr<dai::InputQueue>>& inputQueues) {
    auto sync = masterPipeline->create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(syncThreshold);
    for(auto p : masterNode) {
        auto name = std::string("master_") + p.first;
        p.second->link(sync->inputs[name]);
        outputNames.push_back(name);
    }

    for(auto& p : slaveQueues) {
        for(auto& q : p.second) {
            auto name = std::string("slave_") + p.first + "_" + q.first;
            outputNames.push_back(name);
            auto input_queue = sync->inputs[name].createInputQueue();
            inputQueues.emplace(name, input_queue);
        }
    }

    return sync;
}

void setUpCameraSocket(std::shared_ptr<dai::Pipeline>& pipeline,
                       dai::CameraBoardSocket socket,
                       std::string& name,
                       float targetFps,
                       dai::ExternalFrameSyncRole role,
                       std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                       std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                       std::vector<std::string>& camSockets) {
    auto outNode = createPipeline(pipeline, socket, targetFps, role);

    if(role == dai::ExternalFrameSyncRole::MASTER) {
        if(!masterNode.has_value()) {
            masterNode.emplace();
        }
        masterNode.value().emplace(dai::toString(socket), outNode);

    } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
        if(slaveQueues.find(name) == slaveQueues.end()) {
            slaveQueues.emplace(name, std::map<std::string, std::shared_ptr<dai::MessageQueue>>());
        }
        slaveQueues[name].emplace(dai::toString(socket), outNode->createOutputQueue());

    } else {
        throw std::runtime_error("Don't know how to handle role " + dai::toString(role));
    }

    if(std::find(camSockets.begin(), camSockets.end(), dai::toString(socket)) == camSockets.end()) {
        camSockets.emplace_back(dai::toString(socket));
    }
}

void setupDevice(dai::DeviceInfo& deviceInfo,
                 std::shared_ptr<dai::Pipeline>& masterPipeline,
                 std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                 std::map<std::string, std::shared_ptr<dai::Pipeline>>& slavePipelines,
                 std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                 std::vector<std::string>& camSockets,
                 std::vector<std::string>& sockets,
                 float targetFps) {
    auto pipeline = std::make_shared<dai::Pipeline>(std::make_shared<dai::Device>(deviceInfo));
    auto device = pipeline->getDefaultDevice();

    if(device->getPlatform() != dai::Platform::RVC4) {
        throw std::runtime_error("This example supports only RVC4 platform!");
    }

    std::string name = deviceInfo.getXLinkDeviceDesc().name;
    auto role = device->getExternalFrameSyncRole();

    std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
    std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
    std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

    for(auto socket : device->getConnectedCameras()) {
        if(std::find(sockets.begin(), sockets.end(), dai::toString(socket)) == sockets.end()) {
            continue;
        }
        setUpCameraSocket(pipeline, socket, name, targetFps, role, masterNode, slaveQueues, camSockets);
    }

    if(role == dai::ExternalFrameSyncRole::MASTER) {
        device->setExternalStrobeEnable(true);
        std::cout << device->getDeviceId() << " is master" << std::endl;

        if(masterPipeline != nullptr) {
            throw std::runtime_error("Only one master pipeline is supported");
        }
        masterPipeline = pipeline;
    } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
        slavePipelines[name] = pipeline;
        std::cout << device->getDeviceId() << " is slave" << std::endl;
    } else {
        throw std::runtime_error("Don't know how to handle role " + dai::toString(role));
    }
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
    if(argc < 6) {
        std::cout
            << "Usage: " << argv[0]
            << " <target_fps> <recv-all-timeout-sec> <sync-threshold-sec> <initial-sync-timeout-sec> <sockets> [<device_ip_1> <device_ip_2> [device_ip_3]] ..."
            << std::endl;
        std::exit(0);
    }

    signal(SIGINT, interruptHandler);

    float targetFps = std::stof(argv[1]);

    int recvAllTimeoutSec = std::stoi(argv[2]);
    float syncThresholdSec = std::stof(argv[3]);
    int initialSyncTimeoutSec = std::stoi(argv[4]);

    std::vector<std::string> sockets;
    std::stringstream ss(argv[5]);
    std::string item;
    while(std::getline(ss, item, ',')) {
        sockets.push_back(item);
    }

    std::vector<dai::DeviceInfo> deviceInfos;

    if(argc > 6) {
        for(int i = 6; i < argc; i++) {
            deviceInfos.emplace_back(std::string(argv[i]));
        }
    } else {
        deviceInfos = dai::Device::getAllAvailableDevices();
    }

    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        std::exit(0);
    }

    std::shared_ptr<dai::Pipeline> masterPipeline;
    std::optional<std::map<std::string, dai::Node::Output*>> masterNode;

    std::map<std::string, std::shared_ptr<dai::Pipeline>> slavePipelines;
    std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>> slaveQueues;

    std::map<std::string, std::shared_ptr<dai::InputQueue>> inputQueues;
    std::vector<std::string> outputNames;
    std::vector<std::string> camSockets;

    for(auto deviceInfo : deviceInfos) {
        setupDevice(deviceInfo, masterPipeline, masterNode, slavePipelines, slaveQueues, camSockets, sockets, targetFps);
    }

    if(masterPipeline == nullptr || !masterNode.has_value()) {
        throw std::runtime_error("No master detected!");
    }
    if(slavePipelines.size() < 1) {
        throw std::runtime_error("No slaves detected!");
    }

    auto sync =
        createSyncNode(masterPipeline, *masterNode, std::chrono::nanoseconds(long(round(1e9 * 0.5f / targetFps))), outputNames, slaveQueues, inputQueues);
    auto queue = sync->out.createOutputQueue();

    masterPipeline->start();
    for(auto p : slavePipelines) {
        p.second->start();
    }

    FPSCounter fpsCounter;

    std::optional<std::shared_ptr<dai::MessageGroup>> latestFrameGroup;
    bool firstReceived = false;
    auto startTime = std::chrono::steady_clock::now();
    auto prevReceived = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initialSyncTime;

    bool waitingForInitialSync = true;

    while(running) {
        for(auto p : slaveQueues) {
            for(auto q : p.second) {
                while(q.second->has()) {
                    inputQueues[std::string("slave_") + p.first + "_" + q.first]->send(q.second->get());
                }
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
            using ts_type = std::chrono::time_point<std::chrono::steady_clock>;
            std::map<std::string, ts_type> tsValues;
            for(auto name : outputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(name);
                tsValues.emplace(name, frame->getTimestamp(dai::CameraExposureOffset::END));
            }

            std::vector<cv::Mat> imgs;
            std::vector<bool> firstFrameDone;
            for(auto name : camSockets) {
                imgs.emplace_back();
                firstFrameDone.emplace_back(false);
            }

            int32_t idx = -1;
            for(auto outputName : outputNames) {
                for(uint32_t i = 0; i < camSockets.size(); i++) {
                    if(outputName.find(camSockets[i]) != std::string::npos) {
                        idx = i;
                        break;
                    }
                }
                if(idx == -1) {
                    throw std::runtime_error(std::string("Could not find camera socket for ") + outputName);
                }

                auto msg = latestFrameGroup.value()->get<dai::ImgFrame>(outputName);
                auto fps = fpsCounter.getFps();
                auto frame = msg->getCvFrame();

                cv::putText(frame, outputName, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 127, 255}, 2, cv::LINE_AA);

                cv::putText(frame,
                            "Timestamp: "
                                + std::to_string(1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(tsValues[outputName].time_since_epoch()).count())
                                + " | FPS: " + std::to_string(fps).substr(0, 5),
                            {20, 80},
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            {255, 0, 50},
                            2,
                            cv::LINE_AA);

                if(!firstFrameDone[idx]) {
                    imgs[idx] = frame;
                    firstFrameDone[idx] = true;
                } else {
                    cv::hconcat(frame, imgs[idx], imgs[idx]);
                }
            }

            auto compFunct = [](const std::pair<std::string, ts_type>& p1, const std::pair<std::string, ts_type>& p2) -> bool { return p1.second < p2.second; };

            auto maxElement = std::max_element(tsValues.begin(), tsValues.end(), compFunct);
            auto minElement = std::min_element(tsValues.begin(), tsValues.end(), compFunct);

            auto delta = maxElement->second - minElement->second;

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

            for(auto i = 0; i < imgs.size(); i++) {
                cv::putText(imgs[i],
                            syncStatusStr + " | delta = "
                                + std::to_string(1e-3 * float(std::chrono::duration_cast<std::chrono::microseconds>(delta).count())).substr(0, 5) + " ms",
                            {20, 120},
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            color,
                            2,
                            cv::LINE_AA);
            }

            for(auto i = 0; i < imgs.size(); i++) {
                cv::imshow("synced_view_" + camSockets[i], imgs[i]);
            }

            latestFrameGroup.reset();
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
