#include <algorithm>
#include <chrono>
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


int main(int argc, char** argv) {

    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <target_fps> <device_ip_1> <device_ip_2> [device_ip_3] ..." << std::endl;
        std::exit(0);
    }

    float TARGET_FPS = std::stof(argv[1]);

    std::vector<dai::DeviceInfo> DEVICE_INFOS;
    for (int i = 2; i < argc; i++) {
        DEVICE_INFOS.emplace_back(std::string(argv[i]));
    }

    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    std::vector<dai::Pipeline> master_pipelines;
    std::vector<dai::Pipeline> slave_pipelines;
    std::vector<std::string> device_ids;

    for (auto deviceInfo : DEVICE_INFOS) 
    {
        auto pipeline = dai::Pipeline(std::make_shared<dai::Device>(deviceInfo));
        auto device = pipeline.getDefaultDevice();

        std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
        std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
        std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

        // auto socket = device->getConnectedCameras()[0];
        auto socket = device->getConnectedCameras()[1];

        auto cam = pipeline.create<dai::node::Camera>()->build(socket, std::nullopt, TARGET_FPS);
        auto out_q = cam->requestOutput(std::make_pair(640, 480), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH)->createOutputQueue();

        auto role = device->getM8FsyncRole();

        if (role == dai::M8FsyncRole::MASTER) {
            device->setM8StrobeEnable(true);
            device->setM8StrobeLimits(0.05f, 0.95f);
            master_pipelines.push_back(pipeline);
        } else if (role == dai::M8FsyncRole::SLAVE) {
            slave_pipelines.push_back(pipeline);
        } else {
            throw std::runtime_error("Don't know how to handle role " + dai::toString(role));
        }

        queues.push_back(out_q);
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
    for (auto p : master_pipelines) {
        p.start();
    }
    for (auto p : slave_pipelines) {
        p.start();
    }

    std::map<int, std::shared_ptr<dai::ImgFrame>> latest_frames;
    std::vector<bool> receivedFrames;
    std::vector<FPSCounter> fpsCounters(queues.size());

    for(auto q : queues) {
        receivedFrames.push_back(false);
    }

    while (true) {
        for (int i = 0; i < queues.size(); i++) {
            auto q = queues[i];
            while (q->has()) {
                latest_frames.emplace(std::make_pair(i,std::dynamic_pointer_cast<dai::ImgFrame>(q->get())));
                if (!receivedFrames[i]) {
                    std::cout << "=== Received frame from " << device_ids[i] << std::endl;
                    receivedFrames[i] = true;
                }
                fpsCounters[i].tick();
            }
        }

        if (latest_frames.size() == queues.size()) {
            std::vector<std::chrono::time_point<std::chrono::steady_clock>> ts_values;
            for (auto pair : latest_frames) {
                auto f = pair.second;
                ts_values.push_back(f->getTimestamp(dai::CameraExposureOffset::END));
            }
            
            cv::Mat imgs;
            for (int i = 0; i < queues.size(); i++) {
                auto msg = latest_frames[i];
                auto fps = fpsCounters[i].getFps();
                auto frame = msg->getCvFrame();
                
                cv::putText(frame,
                    device_ids[i] + " | Timestamp: " + std::to_string(ts_values[i].time_since_epoch().count()) + " | FPS: " + std::to_string(fps).substr(0, 5),
                    {20, 40},
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

            std::string sync_status = "out of sync";
            if (abs(std::chrono::duration_cast<std::chrono::microseconds>(
                    *std::max_element(ts_values.begin(), ts_values.end()) -
                    *std::min_element(ts_values.begin(), ts_values.end())
                ).count()) < 1e3) {
                sync_status = "in sync";
            }
            auto delta = *std::max_element(ts_values.begin(), ts_values.end()) - *std::min_element(ts_values.begin(), ts_values.end());
            cv::Scalar color = (sync_status == "in sync") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            cv::putText(imgs,
                sync_status + " | delta = " + std::to_string(1e-3 * float(std::chrono::duration_cast<std::chrono::microseconds>(delta).count())).substr(0, 5) + " ms",
                {20, 80},
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv::LINE_AA);

            cv::imshow("synced_view", imgs);

            latest_frames.clear();
        }

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}