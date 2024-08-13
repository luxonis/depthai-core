// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/utility/SharedMemory.hpp"
#include "depthai/pipeline/InputQueue.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;
    
    auto cam = pipeline.create<dai::node::Camera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);  // TODO(Morato) - change to semantic name
    dai::ImgFrameCapability cap;
    cap.type = dai::ImgFrame::Type::NV12;  // Fastest
    cap.size.fixed(std::pair<int, int>(3840, 2160));
    auto outputQueue = cam->requestOutput(cap, true)->createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
	auto t_start = std::chrono::high_resolution_clock::now();

        auto videoIn = outputQueue->get<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
	auto frame = videoIn->getCvFrame();

	auto t_end = std::chrono::high_resolution_clock::now();
	double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();

	printf("Time: %f\n", elapsed_time_ms);
	auto memoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(videoIn->data);
        if(memoryPtr && memoryPtr->getFd() > 0) {
	    printf("Is zerocopy (%d)!\n", memoryPtr->getFd());
	}

    }
    return 0;
}
