#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create device and pipeline
        std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
        dai::Pipeline pipeline(device);

        // Get connected cameras and create camera nodes
        std::vector<std::shared_ptr<dai::node::Camera>> cameras;
        std::vector<std::string> inputKeys;

        for(const auto& socket : device->getConnectedCameras()) {
            auto camera = pipeline.create<dai::node::Camera>();
            camera->build(socket);
            cameras.push_back(camera);
            inputKeys.push_back(std::to_string(cameras.size() - 1));
        }

        // Create script node
        auto script = pipeline.create<dai::node::Script>();

        // Link cameras to script inputs
        for(size_t i = 0; i < cameras.size(); i++) {
            std::string inputName = std::to_string(i);
            cameras[i]->requestFullResolutionOutput()->link(script->inputs[inputName]);
            script->inputs[inputName].setBlocking(false);
            script->inputs[inputName].setMaxSize(1);
        }

        // Create control and preview queues
        auto controlQueue = script->inputs["control"].createInputQueue();
        auto preview = script->outputs["out"].createOutputQueue();

        // Set the script content
        std::string scriptContent = R"(
            inputToStream = 0
            maxID = )" + std::to_string(inputKeys.size() - 1)
                                    + R"(
            while True:
                controlMessage = node.inputs["control"].tryGet()
                if controlMessage is not None:
                    if(inputToStream < maxID):
                        inputToStream += 1
                    else:
                        inputToStream = 0
                frame = node.inputs[str(inputToStream)].get()
                node.outputs["out"].send(frame)
        )";
        script->setScript(scriptContent);

        // Start pipeline
        pipeline.start();
        std::cout << "To switch between streams, press 's'" << std::endl;

        // Main loop
        while(pipeline.isRunning() && !quitEvent) {
            auto previewMessage = preview->get<dai::ImgFrame>();
            cv::Mat frame = previewMessage->getCvFrame();
            cv::imshow("preview", frame);

            int key = cv::waitKey(1);
            if(key == 's') {
                auto controlMessage = std::make_shared<dai::Buffer>();
                controlQueue->send(controlMessage);
            } else if(key == 'q') {
                break;
            }
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}