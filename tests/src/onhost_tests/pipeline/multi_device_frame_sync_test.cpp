#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

#define REQUIRE_MSG(x, msg)                                         \
    if(!(x)) {                                                      \
        std::cout << "\x1B[1;31m" << msg << "\x1B[0m" << std::endl; \
        REQUIRE((x));                                               \
    }

namespace {
    enum class SyncType {
        EXTERNAL,
        PTP,
    };

    std::string toString(SyncType syncType) {
        if(syncType == SyncType::EXTERNAL) {
            return "external";
        } else if(syncType == SyncType::PTP) {
            return "PTP";
        } else {
            throw std::runtime_error("Unknown sync type");
        }
    }
}

dai::Node::Output* createPipeline(std::shared_ptr<dai::Pipeline> pipeline,
                                  dai::CameraBoardSocket socket,
                                  float sensorFps,
                                  SyncType syncType,
                                  std::optional<dai::ExternalFrameSyncRole> role) {
    std::shared_ptr<dai::node::Camera> cam;
    if(syncType == SyncType::PTP || role == dai::ExternalFrameSyncRole::MASTER) {
        cam = pipeline->create<dai::node::Camera>()->build(socket, std::nullopt, sensorFps);
    } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
        cam = pipeline->create<dai::node::Camera>()->build(socket, std::nullopt);
    } else {
        throw std::runtime_error("Don't know how to handle role");
    }

    if(syncType == SyncType::PTP) {
        cam->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::TIME_PTP);
        std::cout << "Setting PTP for " << dai::toString(socket) << std::endl;
    }

    auto output = cam->requestOutput(std::make_pair(640, 480), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH);
    return output;
}

std::shared_ptr<dai::node::Sync> createSyncNode(std::shared_ptr<dai::Pipeline>& masterPipeline,
                                                std::map<std::string, dai::Node::Output*>& masterNode,
                                                const std::string& masterName,
                                                std::chrono::nanoseconds syncThreshold,
                                                std::vector<std::string>& outputNames,
                                                std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                                                std::map<std::string, std::shared_ptr<dai::InputQueue>>& inputQueues) {
    auto sync = masterPipeline->create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(syncThreshold);
    for(auto p : masterNode) {
        auto name = std::string("master_") + masterName + "_" + p.first;
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
                       SyncType syncType,
                       std::optional<dai::ExternalFrameSyncRole> role,
                       std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                       std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                       std::vector<std::string>& camSockets) {
    auto outNode = createPipeline(pipeline, socket, targetFps, syncType, role);

    if(syncType == SyncType::EXTERNAL) {
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
            throw std::runtime_error("Don't know how to handle role");
        }
    } else if(syncType == SyncType::PTP) {
        // For PTP just put the first camera in master.
        // Actual PTP master might be different, but it doesn't matter for this test.
        if(!masterNode.has_value()) {
            masterNode.emplace();
            masterNode.value().emplace(dai::toString(socket), outNode);
        } else {
            if(slaveQueues.find(name) == slaveQueues.end()) {
                slaveQueues.emplace(name, std::map<std::string, std::shared_ptr<dai::MessageQueue>>());
            }
            slaveQueues[name].emplace(dai::toString(socket), outNode->createOutputQueue());
        }
    }

    if(std::find(camSockets.begin(), camSockets.end(), dai::toString(socket)) == camSockets.end()) {
        camSockets.emplace_back(dai::toString(socket));
    }
}

void setUpIrLeds(std::shared_ptr<dai::Device> device) {
    auto drivers = device->getIrDrivers();
    bool found = false;

    for(auto& driver : drivers) {
        std::string name;
        int bus;
        int addr;
        std::tie(name, bus, addr) = driver;

        if(name == "stm-dot") {
            device->setIrLaserDotProjectorIntensity(0.1);
            found = true;
        } else if(name == "stm-flood") {
            device->setIrFloodLightIntensity(0.1);
            found = true;
        }
    }

    if(!found) {
        std::cout << "No IR drivers found, skipping IR intensity setting" << std::endl;
    } else {
        std::cout << "IR intensity set" << std::endl;
    }
}

void setupDevice(dai::DeviceInfo& deviceInfo,
                 std::shared_ptr<dai::Pipeline>& masterPipeline,
                 std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                 std::optional<std::string>& masterName,
                 std::map<std::string, std::shared_ptr<dai::Pipeline>>& slavePipelines,
                 std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                 std::vector<std::string>& camSockets,
                 float targetFps,
                 SyncType syncType) {
    auto pipeline = std::make_shared<dai::Pipeline>(std::make_shared<dai::Device>(deviceInfo));
    auto device = pipeline->getDefaultDevice();

    if(device->getPlatform() != dai::Platform::RVC4) {
        throw std::runtime_error("This test supports only RVC4 platform!");
    }

    std::string name = deviceInfo.getXLinkDeviceDesc().name;
    std::optional<dai::ExternalFrameSyncRole> role = std::nullopt;
    if(syncType == SyncType::EXTERNAL) {
        role = device->getExternalFrameSyncRole();
    }

    std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
    std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
    std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

    for(auto socket : device->getConnectedCameras()) {
        setUpCameraSocket(pipeline, socket, name, targetFps, syncType, role, masterNode, slaveQueues, camSockets);
    }

    setUpIrLeds(device);

    if(syncType == SyncType::EXTERNAL) {
        if(role == dai::ExternalFrameSyncRole::MASTER) {
            device->setExternalStrobeEnable(true);
            std::cout << device->getDeviceId() << " is master" << std::endl;

            if(masterPipeline != nullptr) {
                throw std::runtime_error("Only one master pipeline is supported");
            }
            masterPipeline = pipeline;
            masterName = name;
        } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
            slavePipelines[name] = pipeline;
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        } else {
            throw std::runtime_error("Don't know how to handle role");
        }
    } else if(syncType == SyncType::PTP) {
        // For PTP just put the first camera in master.
        // Actual PTP master might be different, but it doesn't matter for this test.
        if(masterPipeline == nullptr) {
            masterPipeline = pipeline;
            masterName = name;
        } else {
            slavePipelines[name] = pipeline;
        }
    }
}

int testFsync(
    float targetFps, double syncThresholdSec, uint64_t testDurationSec, int recvAllTimeoutSec, int initialSyncTimeoutSec, SyncType syncType) {
    std::cout << "=================================\x1B[1;32mTest started\x1B[0m================================" << std::endl;
    std::cout << "Sync type: " << toString(syncType) << std::endl;
    std::cout << "FPS: " << targetFps << std::endl;
    std::cout << "SYNC_THRESHOLD_SEC: " << syncThresholdSec << std::endl;
    std::cout << "RECV_ALL_TIMEOUT_SEC: " << recvAllTimeoutSec << std::endl;
    std::cout << "INITIAL_SYNC_TIMEOUT_SEC: " << initialSyncTimeoutSec << std::endl;
    std::vector<dai::DeviceInfo> deviceInfos = dai::Device::getAllAvailableDevices();

    REQUIRE_MSG(deviceInfos.size() >= 2, "At least two devices are required for this test.");

    std::shared_ptr<dai::Pipeline> masterPipeline;
    std::optional<std::map<std::string, dai::Node::Output*>> masterNode;
    std::optional<std::string> masterName;

    std::map<std::string, std::shared_ptr<dai::Pipeline>> slavePipelines;
    std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>> slaveQueues;

    std::map<std::string, std::shared_ptr<dai::InputQueue>> inputQueues;
    std::vector<std::string> outputNames;
    std::vector<std::string> camSockets;

    for(auto deviceInfo : deviceInfos) {
        setupDevice(deviceInfo, masterPipeline, masterNode, masterName, slavePipelines, slaveQueues, camSockets, targetFps, syncType);
    }

    if(masterPipeline == nullptr || !masterNode.has_value() || !masterName.has_value()) {
        throw std::runtime_error("No master detected!");
    }
    if(slavePipelines.size() < 1) {
        throw std::runtime_error("No slaves detected!");
    }

    auto sync =
        createSyncNode(masterPipeline, *masterNode, *masterName, std::chrono::nanoseconds(long(round(1e9 * 0.5f / targetFps))), outputNames, slaveQueues, inputQueues);
    auto queue = sync->out.createOutputQueue();

    masterPipeline->start();
    for(auto p : slavePipelines) {
        p.second->start();
    }

    std::optional<std::shared_ptr<dai::MessageGroup>> latestFrameGroup;
    bool firstReceived = false;
    auto startTime = std::chrono::steady_clock::now();
    auto prevReceived = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initialSyncTime;

    bool waitingForInitialSync = true;
    std::atomic_bool running{true};

    auto dataCollector = [&](const std::string& deviceName, const std::string& socketName) {
        const auto queueName = std::string("slave_") + deviceName + "_" + socketName;
        auto camOutputQueue = slaveQueues.at(deviceName).at(socketName);
        auto inputQueue = inputQueues.at(queueName);
        while(running.load()) {
            if(camOutputQueue->has()) {
                inputQueue->send(camOutputQueue->get());
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    };

    std::vector<std::thread> threads;
    for(const auto& slaveEntry : slaveQueues) {
        for(const auto& socketEntry : slaveEntry.second) {
            threads.emplace_back(dataCollector, slaveEntry.first, socketEntry.first);
        }
    }

    struct ThreadsGuard {
        std::atomic_bool& running;
        std::vector<std::thread>& threads;
        ~ThreadsGuard() {
            running.store(false);
            for(auto& thread : threads) {
                if(thread.joinable()) {
                    thread.join();
                }
            }
        }
    } threadsGuard{running, threads};

    while(true) {
        while(queue->has()) {
            auto syncData = queue->get();
            REQUIRE_MSG(syncData != nullptr, "Sync node failed to receive message");
            latestFrameGroup = std::dynamic_pointer_cast<dai::MessageGroup>(syncData);
            if(!firstReceived) {
                firstReceived = true;
                initialSyncTime = std::chrono::steady_clock::now();
            }
            prevReceived = std::chrono::steady_clock::now();
        }

        if(!firstReceived) {
            auto endTime = std::chrono::steady_clock::now();
            auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
            REQUIRE_MSG(elapsedSec < recvAllTimeoutSec, "Timeout: Didn't receive all frames in time");
        }

        auto totalElapsedSec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();

        if(totalElapsedSec >= testDurationSec) {
            std::cout << "Timeout: Test finished after " << totalElapsedSec << " sec" << std::endl;
            running.store(false);
            break;
        }

        if(latestFrameGroup.has_value()) {
            REQUIRE_MSG(size_t(latestFrameGroup.value()->getNumMessages()) == outputNames.size(),
                        "Number of messages received doesn't match number of outputs");

            using ts_type = std::chrono::time_point<std::chrono::steady_clock>;
            std::map<std::string, ts_type> tsValues;
            for(auto name : outputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(name);
                REQUIRE_MSG(frame != nullptr, "Frame pointer is null");
                tsValues.emplace(name, frame->getTimestamp(dai::CameraExposureOffset::END));
            }

            auto compFunct = [](const std::pair<std::string, ts_type>& p1, const std::pair<std::string, ts_type>& p2) -> bool { return p1.second < p2.second; };

            auto maxElement = std::max_element(tsValues.begin(), tsValues.end(), compFunct);
            auto minElement = std::min_element(tsValues.begin(), tsValues.end(), compFunct);

            auto delta = maxElement->second - minElement->second;
            auto deltaUs = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();

            bool syncStatus = abs(deltaUs) < syncThresholdSec * 1e6;

            if(!syncStatus && waitingForInitialSync) {
                auto endTime = std::chrono::steady_clock::now();
                auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - initialSyncTime.value()).count();
                REQUIRE_MSG(elapsedSec < initialSyncTimeoutSec, "Timeout: Didn't sync frames in time");
            }

            if(syncStatus && waitingForInitialSync) {
                std::cout << "Sync status: in sync" << std::endl;
                waitingForInitialSync = false;
            }

            REQUIRE_MSG(waitingForInitialSync || syncStatus, "Sync error: Sync lost, threshold exceeded: " << deltaUs << " us");

            latestFrameGroup.reset();
        }
    }

    return 0;
}

TEST_CASE("Test Multi-device external frame sync with different FPS values", "[fsync]") {
    // Specify a list of FPS values to test with.
    // auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 60.0f, 120.0f, 240.0f, 300.0f, 600.0f);
    auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 60.0f);
    CAPTURE(fps);
    testFsync(fps, 1e-3, 60, 10, 4, SyncType::EXTERNAL);
}

TEST_CASE("Test Multi-device PTP frame sync with different FPS values", "[fsync]") {
    // Specify a list of FPS values to test with.
    // auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 60.0f, 120.0f, 240.0f, 300.0f, 600.0f);
    auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 60.0f);
    CAPTURE(fps);
    testFsync(fps, 1e-3, 60, 15, 60, SyncType::PTP);
}
