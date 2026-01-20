#include <cstddef>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Sync.hpp"

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#define REQUIRE_MSG(x, msg) if(!(x)) { std::cout << "\x1B[1;31m" << msg << "\x1B[0m" << std::endl; REQUIRE((x)); }

dai::Node::Output *createPipeline(dai::Pipeline& pipeline, dai::CameraBoardSocket socket, int sensorFps, dai::M8FsyncRole role) {
    std::shared_ptr<dai::node::Camera> cam;
    if (role == dai::M8FsyncRole::MASTER) {
        cam = pipeline.create<dai::node::Camera>()
            ->build(socket, std::nullopt, sensorFps);
    } else {
        cam = pipeline.create<dai::node::Camera>()
        ->build(socket, std::nullopt);
    }

    auto output = cam->requestOutput(
        std::make_pair(640, 480), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH);
    return output;
}

int testFsync(float targetFps, double syncThresholdSec, uint64_t testDurationSec, int recvAllTimeoutSec, int initialSyncTimeoutSec) {

    std::cout << "=================================\x1B[1;32mTest started\x1B[0m================================" << std::endl;
    std::cout << "FPS: " << targetFps << std::endl;
    std::cout << "SYNC_THRESHOLD_SEC: " << syncThresholdSec << std::endl;
    std::cout << "RECV_ALL_TIMEOUT_SEC: " << recvAllTimeoutSec << std::endl;
    std::cout << "INITIAL_SYNC_TIMEOUT_SEC: " << initialSyncTimeoutSec << std::endl;

    std::vector<dai::DeviceInfo> deviceInfos = dai::Device::getAllAvailableDevices();

    REQUIRE_MSG(deviceInfos.size() >= 2, "At least two devices are required for this test.");

    std::vector<dai::Node::Output *> masterNodes;
    std::vector<std::shared_ptr<dai::MessageQueue>> slaveQueues;
    std::vector<dai::Pipeline> masterPipelines;
    std::vector<dai::Pipeline> slavePipelines;
    std::vector<std::string> deviceIds;

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    std::vector<std::string> slaveInputNames;
    std::vector<std::string> outputNames;

    for (auto deviceInfo : deviceInfos) 
    {
        auto pipeline = dai::Pipeline(std::make_shared<dai::Device>(deviceInfo));
        auto device = pipeline.getDefaultDevice();

        auto role = device->getM8FsyncRole();

        std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
        std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
        std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

        auto socket = device->getConnectedCameras()[0];
        if (device->getProductName().find("OAK4-D") != std::string::npos ||
            device->getProductName().find("OAK-4-D") != std::string::npos)
        {
            socket = device->getConnectedCameras()[1];
        }

        auto outNode = createPipeline(pipeline, socket, targetFps, role);

        if (device->getProductName().find("OAK4-D-PRO") != std::string::npos ||
            device->getProductName().find("OAK-4-PRO") != std::string::npos) {
            device->setIrFloodLightIntensity(0.1);
            device->setIrLaserDotProjectorIntensity(0.1);
        }

        if (role == dai::M8FsyncRole::MASTER) {
            device->setM8StrobeEnable(true);
            masterPipelines.push_back(pipeline);
            masterNodes.push_back(outNode);
            std::cout << device->getDeviceId() << " is master" << std::endl;
        } else if (role == dai::M8FsyncRole::SLAVE) {
            slavePipelines.push_back(pipeline);
            slaveQueues.push_back(outNode->createOutputQueue());
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        } else {
            throw std::runtime_error("Don't know how to handle role " + dai::toString(role));
        }

        deviceIds.push_back(deviceInfo.getXLinkDeviceDesc().name);
    }

    if (masterPipelines.size() > 1) {
        throw std::runtime_error("Multiple masters detected!");
    }
    if (masterPipelines.size() == 0) {
        throw std::runtime_error("No master detected!");
    }
    if (slavePipelines.size() < 1) {
        throw std::runtime_error("No slaves detected!");
    }

    auto sync = masterPipelines[0].create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(std::chrono::nanoseconds(long(round(1e9 * 0.5f / targetFps))));
    masterNodes[0]->link(sync->inputs["master"]);
    outputNames.push_back("master");

    for (unsigned long i = 0; i < slaveQueues.size(); i++) {
        auto name = std::string("slave_") + std::to_string(i);
        slaveInputNames.push_back(name);
        outputNames.push_back(name);
        auto input_queue = sync->inputs[name].createInputQueue();
        inputQueues.push_back(input_queue);
    }

    auto queue = sync->out.createOutputQueue();

    for (auto p : masterPipelines) {
        p.start();
    }
    for (auto p : slavePipelines) {
        p.start();
    }

    std::optional<std::shared_ptr<dai::MessageGroup>> latestFrameGroup;
    bool firstReceived = false;
    auto startTime = std::chrono::steady_clock::now();
    auto prevReceived = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initialSyncTime;

    bool waitingForInitialSync = true;

    while (true) {
        for (int i = 0; i < slaveQueues.size(); i++) {
            while (slaveQueues[i]->has()) {
                inputQueues[i]->send(slaveQueues[i]->get());
            }
        }

        while (queue->has()) {
            auto syncData = queue->get();
            REQUIRE_MSG(syncData != nullptr, "Sync node failed to receive message");
            latestFrameGroup = std::dynamic_pointer_cast<dai::MessageGroup>(syncData);
            if (!firstReceived) {
                firstReceived = true;
                initialSyncTime = std::chrono::steady_clock::now();
            }
            prevReceived = std::chrono::steady_clock::now();
        }

        if (!firstReceived) {
            auto endTime = std::chrono::steady_clock::now();
            auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
            REQUIRE_MSG(elapsedSec < recvAllTimeoutSec, "Timeout: Didn't receive all frames in time");
        }

        auto totalElapsedSec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();

        if (totalElapsedSec >= testDurationSec) {
            std::cout << "Timeout: Test finished after " << totalElapsedSec << " sec" << std::endl;
            break;
        }

        if (latestFrameGroup.has_value()) {
            REQUIRE_MSG(size_t(latestFrameGroup.value()->getNumMessages()) == outputNames.size(), "Number of messages received doesn't match number of outputs");

            std::vector<std::chrono::time_point<std::chrono::steady_clock>> tsValues;
            // std::vector<std::chrono::time_point<std::chrono::system_clock>> ts_values;
            for (auto name : outputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(name);
                REQUIRE_MSG(frame != nullptr, "Frame pointer is null");
                tsValues.push_back(frame->getTimestamp(dai::CameraExposureOffset::END));
            }

            auto delta = *std::max_element(tsValues.begin(), tsValues.end()) - *std::min_element(tsValues.begin(), tsValues.end());
            auto deltaUs = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();

            bool syncStatus = abs(deltaUs) < syncThresholdSec * 1e6;

            if (!syncStatus && waitingForInitialSync) {
                auto endTime = std::chrono::steady_clock::now();
                auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - initialSyncTime.value()).count();
                REQUIRE_MSG(elapsedSec < initialSyncTimeoutSec, "Timeout: Didn't sync frames in time");
            }

            if (syncStatus && waitingForInitialSync) {
                std::cout << "Sync status: in sync" << std::endl;
                waitingForInitialSync = false;
            }

            REQUIRE_MSG(waitingForInitialSync || syncStatus, "Sync error: Sync lost, threshold exceeded: " << deltaUs << " us");

            latestFrameGroup.reset();
        }
    }

    return 0;
}

TEST_CASE("Test Multi-device Fsync with different FPS values", "[fsync]") {
    // Specify a list of FPS values to test with.
    // auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 60.0f, 120.0f, 240.0f, 300.0f, 600.0f);
    auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 60.0f);
    CAPTURE(fps);
    testFsync(fps, 3e-3, 60, 10, 4);
}
