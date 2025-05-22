#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto camQIn = cam->inputControl.createInputQueue();

    // In some cases (IMX586), this requires an 8k screen to be able to see the full resolution at once
    auto streamHighestRes = cam->requestFullResolutionOutput();

    // Create script node for still capture
    auto script = pipeline.create<dai::node::Script>();
    streamHighestRes->link(script->inputs["in"]);

    // Current workaround for OAK4 cameras, as Camera node doesn't yet support "still" frame capture
    script->setScript(R"(
        while True:
            message = node.inputs["in"].get()
            trigger = node.inputs["trigger"].tryGet()
            if trigger is not None:
                node.warn("Trigger received!")
                node.io["highest_res"].send(message)
    )");

    // If 8k, we can only have 1 output stream, so we need to use ImageManip to downscale
    auto imgManip = pipeline.create<dai::node::ImageManip>();
    streamHighestRes->link(imgManip->inputImage);
    imgManip->initialConfig->setOutputSize(1333, 1000);
    imgManip->setMaxOutputFrameSize(1333 * 1000 * 3);
    auto downscaledResQ = imgManip->out.createOutputQueue();

    auto highestResQ = script->outputs["highest_res"].createOutputQueue();
    auto qTrigger = script->inputs["trigger"].createInputQueue();

    // Start pipeline
    pipeline.start();
    std::cout << "To capture an image, press 'c'" << std::endl;

    while(true) {
        auto imgHd = downscaledResQ->get<dai::ImgFrame>();
        if(imgHd == nullptr) continue;

        cv::Mat frame = imgHd->getCvFrame();
        cv::imshow("video", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
        if(key == 'c') {
            // Send a trigger message to the Script node
            qTrigger->send(std::make_shared<dai::Buffer>());
        }

        if(highestResQ->has()) {
            auto highresImg = highestResQ->get<dai::ImgFrame>();
            if(highresImg != nullptr) {
                cv::Mat frame = highresImg->getCvFrame();
                // Save the full image
                cv::imwrite("full_image.png", frame);
            }
        }
    }

    return 0;
}