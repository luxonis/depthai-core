// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Record.hpp"

int main(int argc, char** argv) {
    dai::Pipeline pipeline(argc > 1 ? std::make_shared<dai::Device>(argv[1]) : std::make_shared<dai::Device>());
    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto* output = camRgb->requestOutput(std::make_pair(640, 480));
    if(output == nullptr) {
        std::cout << "Error creating output, exiting\n";
        return 1;
    }
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRecordVideoFile("camera_recorded.mp4");
    record->setRecordMetadataFile("camera_recorded.mcap");
    output->link(record->input);
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
