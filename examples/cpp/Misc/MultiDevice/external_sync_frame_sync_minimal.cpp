#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>

#include "depthai/depthai.hpp"

namespace {

constexpr float TARGET_FPS = 30.0f;
constexpr std::pair<uint32_t, uint32_t> RESOLUTION{640, 480};
constexpr double SYNC_THRESHOLD_SEC = 1e-3;

std::atomic_bool running{true};

void interruptHandler(int) {
    if(running.exchange(false)) {
        std::cout << "Interrupted! Exiting..." << std::endl;
    } else {
        std::cout << "Exiting now!" << std::endl;
        std::exit(0);
    }
}

std::string getDeviceName(const std::shared_ptr<dai::Device>& device) {
    const auto info = device->getDeviceInfo();
    auto name = info.deviceId;
    if(!info.name.empty()) {
        name += "[" + info.name + "]";
    }
    return name;
}

}  // namespace

int main() {
    signal(SIGINT, interruptHandler);

    // This example only works on devices that are connected with M8 cables with FSYNC Y splitters.
    const auto deviceInfos = dai::Device::getAllAvailableDevices();

    // Variables to keep track of master and slave pipelines and outputs
    std::shared_ptr<dai::Pipeline> masterPipeline;
    std::optional<std::map<std::string, dai::Node::Output*>> masterOutputs;
    std::optional<std::string> masterName;

    std::map<std::string, std::shared_ptr<dai::Pipeline>> slavePipelines;
    std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>> slaveQueues;

    // keep track of sync node inputs for slaves
    std::map<std::string, std::shared_ptr<dai::InputQueue>> inputQueues;
    // keep track of all sync node output names
    std::vector<std::string> outputNames;

    for(const auto& deviceInfo : deviceInfos) {
        // Create pipeline for each device
        auto pipeline = std::make_shared<dai::Pipeline>(std::make_shared<dai::Device>(deviceInfo));
        auto device = pipeline->getDefaultDevice();
        const auto deviceName = getDeviceName(device);
        const auto fsyncRole = device->getExternalFrameSyncRole();

        for(const auto socket : device->getConnectedCameras()) {
            // create a queue for each camera on the device
            std::shared_ptr<dai::node::Camera> cam;
            if(fsyncRole == dai::ExternalFrameSyncRole::MASTER) {
                cam = pipeline->create<dai::node::Camera>()->build(socket, std::nullopt, TARGET_FPS);
            } else {
                // slaves will lock to the master's FPS
                cam = pipeline->create<dai::node::Camera>()->build(socket);
            }

            auto* output = cam->requestOutput(RESOLUTION, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP);
            const auto socketName = dai::toString(socket);

            // Master cameras will be linked to the sync node directly
            if(fsyncRole == dai::ExternalFrameSyncRole::MASTER) {
                if(!masterOutputs.has_value()) {
                    masterOutputs.emplace();
                }
                (*masterOutputs)[socketName] = output;
            // Gather all slave camera outputs
            } else if(fsyncRole == dai::ExternalFrameSyncRole::SLAVE) {
                slaveQueues[deviceName][socketName] = output->createOutputQueue();
            }
        }

        if(fsyncRole == dai::ExternalFrameSyncRole::MASTER) {
            device->setExternalStrobeEnable(true);
            std::cout << device->getDeviceId() << " is master" << std::endl;

            if(masterPipeline != nullptr) {
                throw std::runtime_error("Only one master pipeline is supported");
            }

            masterPipeline = pipeline;
            masterName = deviceName;
        } else if(fsyncRole == dai::ExternalFrameSyncRole::SLAVE) {
            slavePipelines[deviceName] = pipeline;
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        }
    }

    if(masterPipeline == nullptr || !masterOutputs.has_value() || !masterName.has_value()) {
        throw std::runtime_error("No master detected!");
    }

    if(slavePipelines.empty()) {
        throw std::runtime_error("No slaves detected!");
    }

    // Create sync node
    auto sync = masterPipeline->create<dai::node::Sync>();
    // Sync node will run on the host, since it needs to sync multiple devices
    sync->setRunOnHost(true);
    // group frames into pairs that are within 1/2 frame period
    sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / TARGET_FPS)));

    // Link master camera outputs to the sync node
    for(const auto& [socketName, output] : *masterOutputs) {
        const auto name = "master_" + *masterName + "_" + socketName;
        output->link(sync->inputs[name]);
        outputNames.push_back(name);
    }

    // For slaves, we must create an input queue for each output
    // We will then manually forward the frames from each input queue to the output queue
    // This is because slave devices have separate pipelines from the master
    for(const auto& [deviceName, sockets] : slaveQueues) {
        for(const auto& [socketName, queue] : sockets) {
            (void)queue;
            const auto name = "slave_" + deviceName + "_" + socketName;
            outputNames.push_back(name);
            inputQueues[name] = sync->inputs[name].createInputQueue();
        }
    }

    auto syncedGroups = sync->out.createOutputQueue();

    // thread worker for forwarding slave queues to sync node
    auto dataCollector = [&](std::string deviceName, std::string socketName) {
        const auto queueName = "slave_" + deviceName + "_" + socketName;
        auto camOutputQueue = slaveQueues.at(deviceName).at(socketName);
        auto inputQueue = inputQueues.at(queueName);

        // Send frames from slave output queues to sync node input queues
        while(running.load()) {
            if(camOutputQueue->has()) {
                inputQueue->send(camOutputQueue->get());
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    };

    // Start pipelines
    masterPipeline->start();
    for(const auto& [deviceName, pipeline] : slavePipelines) {
        (void)deviceName;
        pipeline->start();
    }

    // Start threads
    std::vector<std::thread> threads;
    for(const auto& [deviceName, sockets] : slaveQueues) {
        for(const auto& [socketName, queue] : sockets) {
            (void)queue;
            threads.emplace_back(dataCollector, deviceName, socketName);
        }
    }

    std::optional<std::shared_ptr<dai::MessageGroup>> latestFrameGroup;

    // main display loop
    while(running.load()) {
        // Get frames from sync node output queue
        while(syncedGroups->has()) {
            latestFrameGroup = syncedGroups->get<dai::MessageGroup>();
        }

        if(latestFrameGroup.has_value() && static_cast<size_t>(latestFrameGroup.value()->getNumMessages()) == outputNames.size()) {
            using ts_type = std::chrono::time_point<std::chrono::steady_clock>;
            std::map<std::string, ts_type> tsValues;
            for(auto name : outputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(name);
                tsValues.emplace(name, frame->getTimestamp(dai::CameraExposureOffset::END));
            }
            auto compFunct = [](const std::pair<std::string, ts_type>& p1, const std::pair<std::string, ts_type>& p2) -> bool { return p1.second < p2.second; };

            auto maxElement = std::max_element(tsValues.begin(), tsValues.end(), compFunct);
            auto minElement = std::min_element(tsValues.begin(), tsValues.end(), compFunct);

            auto delta = maxElement->second - minElement->second;
            auto deltaUs = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();

            if(deltaUs >= SYNC_THRESHOLD_SEC * 1e6) {
                std::cout << "Sync error: Sync lost, threshold exceeded " << deltaUs << " us" << std::endl;
                continue;
            }

            for(const auto& outputName : outputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(outputName);
                cv::imshow("synced_view_" + outputName, frame->getCvFrame());
            }

            latestFrameGroup.reset();  // Wait for next batch
        }

        if((cv::waitKey(1) & 0xFF) == 'q') {
            running.store(false);
            break;
        }
    }

    for(auto& thread : threads) {
        thread.join();
    }

    cv::destroyAllWindows();
    return 0;
}
