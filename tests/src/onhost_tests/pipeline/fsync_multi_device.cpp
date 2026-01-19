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

int testFsync(float TARGET_FPS, double SYNC_THRESHOLD_SEC, uint64_t testDuration_sec, int RECV_ALL_TIMEOUT_SEC, int INITIAL_SYNC_TIMEOUT_SEC) {

    std::cout << "=================================\x1B[1;32mTest started\x1B[0m================================" << std::endl;
    std::cout << "FPS: " << TARGET_FPS << std::endl;
    std::cout << "SYNC_THRESHOLD_SEC: " << SYNC_THRESHOLD_SEC << std::endl;
    std::cout << "RECV_ALL_TIMEOUT_SEC: " << RECV_ALL_TIMEOUT_SEC << std::endl;
    std::cout << "INITIAL_SYNC_TIMEOUT_SEC: " << INITIAL_SYNC_TIMEOUT_SEC << std::endl;

    std::vector<dai::DeviceInfo> DEVICE_INFOS = dai::Device::getAllAvailableDevices();

    REQUIRE_MSG(DEVICE_INFOS.size() >= 2, "At least two devices are required for this test.");

    std::vector<dai::Node::Output *> master_nodes;
    std::vector<std::shared_ptr<dai::MessageQueue>> slave_queues;
    std::vector<dai::Pipeline> master_pipelines;
    std::vector<dai::Pipeline> slave_pipelines;
    std::vector<std::string> device_ids;

    std::vector<std::shared_ptr<dai::InputQueue>> input_queues;
    std::vector<std::string> slave_input_names;
    std::vector<std::string> output_names;

    for (auto deviceInfo : DEVICE_INFOS) 
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

        auto out_n = createPipeline(pipeline, socket, TARGET_FPS, role);

        if (device->getProductName().find("OAK4-D-PRO") != std::string::npos ||
            device->getProductName().find("OAK-4-PRO") != std::string::npos) {
            device->setIrFloodLightIntensity(0.1);
            device->setIrLaserDotProjectorIntensity(0.1);
        }

        if (role == dai::M8FsyncRole::MASTER) {
            device->setM8StrobeEnable(true);
            device->setM8StrobeLimits(0.05f, 0.95f);
            master_pipelines.push_back(pipeline);
            master_nodes.push_back(out_n);
            std::cout << device->getDeviceId() << " is master" << std::endl;
        } else if (role == dai::M8FsyncRole::SLAVE) {
            slave_pipelines.push_back(pipeline);
            slave_queues.push_back(out_n->createOutputQueue());
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        } else {
            throw std::runtime_error("Don't know how to handle role " + dai::toString(role));
        }

        device_ids.push_back(deviceInfo.getXLinkDeviceDesc().name);
    }

    if (master_pipelines.size() > 1) {
        throw std::runtime_error("Multiple masters detected!");
    }
    if (master_pipelines.size() == 0) {
        throw std::runtime_error("No master detected!");
    }
    if (slave_pipelines.size() < 1) {
        throw std::runtime_error("No slaves detected!");
    }

    auto sync = master_pipelines[0].create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(std::chrono::nanoseconds(long(round(1e9 * 0.5f / TARGET_FPS))));
    master_nodes[0]->link(sync->inputs["master"]);
    output_names.push_back("master");

    for (unsigned long i = 0; i < slave_queues.size(); i++) {
        auto name = std::string("slave_") + std::to_string(i);
        slave_input_names.push_back(name);
        output_names.push_back(name);
        auto input_queue = sync->inputs[name].createInputQueue();
        input_queues.push_back(input_queue);
    }

    auto queue = sync->out.createOutputQueue();

    for (auto p : master_pipelines) {
        p.start();
    }
    for (auto p : slave_pipelines) {
        p.start();
    }

    std::optional<std::shared_ptr<dai::MessageGroup>> latest_frame_group;
    bool first_received = false;
    auto start_time = std::chrono::steady_clock::now();
    auto prev_received = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initial_sync_time;

    bool waiting_for_initial_sync = true;

    while (true) {
        for (int i = 0; i < slave_queues.size(); i++) {
            while (slave_queues[i]->has()) {
                input_queues[i]->send(slave_queues[i]->get());
            }
        }

        while (queue->has()) {
            auto syncData = queue->get();
            REQUIRE_MSG(syncData != nullptr, "Sync node failed to receive message");
            latest_frame_group = std::dynamic_pointer_cast<dai::MessageGroup>(syncData);
            if (!first_received) {
                first_received = true;
                initial_sync_time = std::chrono::steady_clock::now();
            }
            prev_received = std::chrono::steady_clock::now();
        }

        if (!first_received) {
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
            REQUIRE_MSG(elapsed_sec < RECV_ALL_TIMEOUT_SEC, "Timeout: Didn't receive all frames in time");
        }

        auto total_elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();

        if (total_elapsed_sec >= testDuration_sec) {
            std::cout << "Timeout: Test finished after " << total_elapsed_sec << " sec" << std::endl;
            break;
        }

        if (latest_frame_group.has_value()) {
            REQUIRE_MSG(size_t(latest_frame_group.value()->getNumMessages()) == output_names.size(), "Number of messages received doesn't match number of outputs");

            std::vector<std::chrono::time_point<std::chrono::steady_clock>> ts_values;
            // std::vector<std::chrono::time_point<std::chrono::system_clock>> ts_values;
            for (auto name : output_names) {
                auto frame = latest_frame_group.value()->get<dai::ImgFrame>(name);
                REQUIRE_MSG(frame != nullptr, "Frame pointer is null");
                ts_values.push_back(frame->getTimestamp(dai::CameraExposureOffset::END));
            }

            auto delta = *std::max_element(ts_values.begin(), ts_values.end()) - *std::min_element(ts_values.begin(), ts_values.end());
            auto delta_us = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();

            bool sync_status = abs(delta_us) < SYNC_THRESHOLD_SEC * 1e6;

            if (!sync_status && waiting_for_initial_sync) {
                auto end_time = std::chrono::steady_clock::now();
                auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(end_time - initial_sync_time.value()).count();
                REQUIRE_MSG(elapsed_sec < INITIAL_SYNC_TIMEOUT_SEC, "Timeout: Didn't sync frames in time");
            }

            if (sync_status && waiting_for_initial_sync) {
                std::cout << "Sync status: in sync" << std::endl;
                waiting_for_initial_sync = false;
            }

            REQUIRE_MSG(waiting_for_initial_sync || sync_status, "Sync error: Sync lost, threshold exceeded: " << delta_us << " us");

            latest_frame_group.reset();
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
