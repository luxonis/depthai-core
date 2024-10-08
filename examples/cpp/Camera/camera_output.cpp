// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto* output = camRgb->requestOutput(std::make_pair(640, 480));
    if(output == nullptr) {
        std::cout << "Error creating output, exiting\n";
        return 1;
    }
    auto outputQueue = output->createOutputQueue();
    pipeline.start();

    while(pipeline.isRunning()) {
        auto imgFrame = outputQueue->get<dai::ImgFrame>();
        cv::imshow("rgb", imgFrame->getCvFrame());
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            pipeline.stop();
            return 0;
        }
    }
    return 0;
}
