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
#include <utility>
#include <vector>
#include <string>

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
        if (frameTimes.size() > 100) {
            frameTimes.erase(frameTimes.begin(), frameTimes.begin() + (frameTimes.size() - 100));
        }
    }

    float getFps() {
        if (frameTimes.size() <= 1) {
            return 0.0f;
        }
        // Calculate the FPS
        return float(frameTimes.size() - 1) * 1e6 / std::chrono::duration_cast<std::chrono::microseconds>(frameTimes.back() - frameTimes.front()).count();
    }
};

dai::Node::Output *createPipeline(dai::Pipeline& pipeline, dai::CameraBoardSocket socket, int sensorFps) {
    auto cam = pipeline.create<dai::node::Camera>()
        ->build(socket, std::nullopt, sensorFps);

    auto output = cam->requestOutput(
        std::make_pair(640, 480), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH);
    return output;
}

namespace {
    bool running = true;
}

void interruptHandler(int sig) {
    if (running) {
        std::cout << "Interrupted! Exiting..." << std::endl;
        running = false;
    } else {
        std::cout << "Exiting now!" << std::endl;
        exit(0);
    }
}

int main(int argc, char** argv) {

    if (argc < 5) {
        std::cout << "Usage: " << argv[0] << " <target_fps> <recv-all-timeout-sec> <sync-threshold-sec> <initial-sync-timeout-sec> [<device_ip_1> <device_ip_2> [device_ip_3]] ..." << std::endl;
        std::exit(0);
    }

    signal(SIGINT, interruptHandler);

    float TARGET_FPS = std::stof(argv[1]);

    int RECV_ALL_TIMEOUT_SEC = std::stoi(argv[2]);
    float SYNC_THRESHOLD_SEC = std::stof(argv[3]);
    float FRAME_LOST_THRESHOLD = 1.0f / TARGET_FPS * 5;
    int INITIAL_SYNC_TIMEOUT_SEC = std::stoi(argv[4]);

    std::vector<dai::DeviceInfo> DEVICE_INFOS;

    if (argc > 5) {
        for (int i = 5; i < argc; i++) {
            DEVICE_INFOS.emplace_back(std::string(argv[i]));
        }
    } else {
        DEVICE_INFOS = dai::Device::getAllAvailableDevices();
    }

    if (DEVICE_INFOS.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        std::exit(0);
    }

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

        std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
        std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
        std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

        auto socket = device->getConnectedCameras()[0];
        if (device->getProductName().find("OAK4-D") != std::string::npos) {
            socket = device->getConnectedCameras()[1];
        }

        auto out_n = createPipeline(pipeline, socket, TARGET_FPS);

        if (device->getProductName().find("OAK4-D-PRO") != std::string::npos) {
            device->setIrFloodLightIntensity(0.1);
            device->setIrLaserDotProjectorIntensity(0.1);
        }

        auto role = device->getM8FsyncRole();

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

    FPSCounter fpsCounter;

    std::optional<std::shared_ptr<dai::MessageGroup>> latest_frame_group;
    bool first_received = false;
    auto start_time = std::chrono::steady_clock::now();
    auto prev_received = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initial_sync_time;

    bool waiting_for_initial_sync = true;

    while (running) {
        for (int i = 0; i < slave_queues.size(); i++) {
            while (slave_queues[i]->has()) {
                input_queues[i]->send(slave_queues[i]->get());
            }
        }

        while (queue->has()) {
            auto syncData = queue->get();
            latest_frame_group = std::dynamic_pointer_cast<dai::MessageGroup>(syncData);
            if (!first_received) {
                first_received = true;
                initial_sync_time = std::chrono::steady_clock::now();
            }
            prev_received = std::chrono::steady_clock::now();
            fpsCounter.tick();
        }

        if (!first_received) {
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
            if (elapsed_sec >= RECV_ALL_TIMEOUT_SEC) {
                std::cout << "Timeout: Didn't receive all frames in time: " << elapsed_sec << std::endl;
                running = false;
            }
        } else {
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(end_time - prev_received).count();
            if (elapsed_sec >= FRAME_LOST_THRESHOLD) {
                std::cout << "Frame lost: Didn't receive all frames in time: " << elapsed_sec << std::endl;
            }
        }

        if (latest_frame_group.has_value() && size_t(latest_frame_group.value()->getNumMessages()) == output_names.size()) {

            std::vector<std::chrono::time_point<std::chrono::steady_clock>> ts_values;
            for (auto name : output_names) {
                auto frame = latest_frame_group.value()->get<dai::ImgFrame>(name);
                ts_values.push_back(frame->getTimestamp(dai::CameraExposureOffset::END));
            }
            
            cv::Mat imgs;
            for (int i = 0; i < output_names.size(); i++) {
                auto msg = latest_frame_group.value()->get<dai::ImgFrame>(output_names[i]);
                auto fps = fpsCounter.getFps();
                auto frame = msg->getCvFrame();
                
                cv::putText(frame,
                    device_ids[i] + " (" + output_names[i] + ")",
                    {20, 40},
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    {0, 127, 255},
                    2,
                    cv::LINE_AA);

                cv::putText(frame,
                    "Timestamp: " + std::to_string(1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(ts_values[i].time_since_epoch()).count()) + " | FPS: " + std::to_string(fps).substr(0, 5),
                    {20, 80},
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    {255, 0, 50},
                    2,
                    cv::LINE_AA);

                if (i == 0) {
                    imgs = frame;
                } else {
                    cv::hconcat(frame, imgs, imgs);
                }
            }

            auto delta = *std::max_element(ts_values.begin(), ts_values.end()) - *std::min_element(ts_values.begin(), ts_values.end());

            bool sync_status = std::chrono::duration_cast<std::chrono::seconds>(delta).count() < SYNC_THRESHOLD_SEC;
            std::string sync_status_str = (sync_status) ? "in sync" : "out of sync";

            cv::Scalar color = (sync_status) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            if (!sync_status && waiting_for_initial_sync) {
                auto end_time = std::chrono::steady_clock::now();
                auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(end_time - initial_sync_time.value()).count();
                if (elapsed_sec >= INITIAL_SYNC_TIMEOUT_SEC) {
                    std::cout << "Timeout: Didn't sync frames in time" << std::endl;
                    running = false;
                }
            }

            if (sync_status && waiting_for_initial_sync) {
                std::cout << "Sync status: " << sync_status_str << std::endl;
                waiting_for_initial_sync = false;
            }

            if (!sync_status && !waiting_for_initial_sync) {
                std::cout << "Sync error: Sync lost, threshold exceeded " << std::chrono::duration_cast<std::chrono::microseconds>(delta).count() << " us" << std::endl;
                running = false;
            }

            cv::putText(imgs,
                sync_status_str + " | delta = " + std::to_string(1e-3 * float(std::chrono::duration_cast<std::chrono::microseconds>(delta).count())).substr(0, 5) + " ms",
                {20, 120},
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv::LINE_AA);

            cv::imshow("synced_view", imgs);

            latest_frame_group.reset();
        }

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}