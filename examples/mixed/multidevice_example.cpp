#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

std::shared_ptr<dai::Pipeline> getPipeline(std::string& deviceType) {
    // Start defining a pipeline
    auto pipeline = std::make_shared<dai::Pipeline>();
    // Define a source - color camera
    auto cam_rgb = pipeline->create<dai::node::ColorCamera>();
    // For the demo, just set a larger RGB preview size for OAK-D
    if(deviceType.find("OAK-D") != std::string::npos) {
        cam_rgb->setPreviewSize(600, 300);
    } else {
        cam_rgb->setPreviewSize(300, 300);
    }
    cam_rgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    cam_rgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam_rgb->setInterleaved(false);

    // Create output
    auto xout_rgb = pipeline->create<dai::node::XLinkOut>();
    xout_rgb->setStreamName("rgb");
    cam_rgb->preview.link(xout_rgb->input);

    return pipeline;
}

int main(int argc, char** argv) {
    auto deviceInfoVec = dai::Device::getAllAvailableDevices();
    auto usb2mode = false;
    auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;

    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> q_rgb_map;
    std::vector<std::shared_ptr<dai::Device>> devices;

    for(auto& deviceInfo : deviceInfoVec) {
        auto device = std::make_shared<dai::Device>(openVinoVersion, deviceInfo, usb2mode);
        devices.push_back(device);
        std::cout << "===Connected to " << deviceInfo.getMxId() << std::endl;
        auto mxid = device->getMxId();
        auto cameras = device->getConnectedCameras();
        auto usb_speed = device->getUsbSpeed();
        std::cout << "   >>> MXID:" << mxid << std::endl;
        std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
        std::cout << "   >>> USB speed:";
        switch(usb_speed) {
            case dai::UsbSpeed::UNKNOWN:
                std::cout << "UNKNOWN";
                break;
            case dai::UsbSpeed::LOW:
                std::cout << "LOW";
                break;
            case dai::UsbSpeed::FULL:
                std::cout << "FULL";
                break;
            case dai::UsbSpeed::HIGH:
                std::cout << "HIGH";
                break;
            case dai::UsbSpeed::SUPER:
                std::cout << "SUPER";
                break;
            case dai::UsbSpeed::SUPER_PLUS:
                std::cout << "SUPER PLUS";
                break;
        }
        std::cout << std::endl;

        std::string deviceType{"unknown"};
        if(cameras.size() == 1) {
            deviceType = "OAK-1";
        } else if(cameras.size() == 3) {
            deviceType = "OAK-D";
        }
        // If USB speed is UNKNOWN, assume it's a POE device
        if(usb_speed == dai::UsbSpeed::UNKNOWN) {
            deviceType += "-POE";
        }

        auto pipeline = getPipeline(deviceType);
        std::cout << "   >>> Loading pipeline for:" << deviceType << std::endl;
        device->startPipeline(*pipeline);

        auto q_rgb = device->getOutputQueue("rgb", 4, false);
        std::string streamName = "rgb-" + mxid + "-" + deviceType;
        q_rgb_map.insert({streamName, q_rgb});
    }
    while(true) {
        for(auto& element : q_rgb_map) {
            auto q_rgb = element.second;
            auto streamName = element.first;
            auto in_rgb = q_rgb->tryGet<dai::ImgFrame>();
            if(in_rgb != nullptr) {
                cv::imshow(streamName, in_rgb->getCvFrame());
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
