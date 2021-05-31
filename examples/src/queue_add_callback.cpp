#include <iostream>
#include <queue>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

struct callbackType {
    std::string name;
    cv::Mat frame;
};

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();

    auto xout = pipeline.create<dai::node::XLinkOut>();

    xout->setStreamName("frames");

    // Properties
    camRgb->setPreviewSize(300, 300);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);

    // Linking
    camRgb->preview.link(xout->input);
    left->out.link(xout->input);
    right->out.link(xout->input);

    auto queue = std::queue<callbackType>();
    std::mutex queueMtx;

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto newFrame = [&queueMtx, &queue](std::shared_ptr<dai::ADatatype> callback) {
        if(dynamic_cast<dai::ImgFrame*>(callback.get()) != nullptr) {
            std::unique_lock<std::mutex> lock(queueMtx);
            callbackType cb;
            dai::ImgFrame* imgFrame = static_cast<dai::ImgFrame*>(callback.get());
            auto num = imgFrame->getInstanceNum();
            cb.name = num == 0 ? "color" : (num == 1 ? "left" : "right");
            cb.frame = imgFrame->getCvFrame();
            queue.push(cb);
        }
    };

    // Output queues will be used to get the grayscale frames from the outputs defined above
    device.getOutputQueue("frames", 4, false)->addCallback(newFrame);

    while(true) {
        callbackType data;
        {
            std::unique_lock<std::mutex> lock(queueMtx);
            if(!queue.empty()) {
                data = queue.front();
                queue.pop();
            }
        }

        if(!data.frame.empty()) {
            cv::imshow(data.name.c_str(), data.frame);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
