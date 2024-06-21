// Includes common necessary includes for development using depthai library
#include <stdexcept>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc < 4 || (argc - 1) % 3 != 0) {
        throw std::runtime_error("USAGE: ./camera_multiple_outputs 1920 1080 0 640 480 1\nWHERE 0 is resize mode: 0 == CROP, 1 == STRETCH, 2 == LETTERBOX");
    }
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> sizes;
    for(int index = 1; index < argc - 1; index += 3) {
        sizes.emplace_back(std::stoul(argv[index]), std::stoul(argv[index + 1]), std::stoul(argv[index + 2]));
    }

    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::Camera>();
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_C);  // TODO(Morato) - change to semantic name

    if(sizes.empty()) {
        throw std::runtime_error("internal error to few sizes");
    }

    std::vector<std::shared_ptr<dai::MessageQueue>> videos;
    for(const auto& size : sizes) {
        dai::ImgFrameCapability cap;
        cap.type = dai::ImgFrame::Type::NV12; // Fastest
        cap.size.value = std::pair{std::get<0>(size), std::get<1>(size)};
        auto mode = std::get<2>(size);
        switch(mode) {
            case 0:
                cap.resizeMode = dai::ImgResizeMode::CROP;
                break;
            case 1:
                cap.resizeMode = dai::ImgResizeMode::STRETCH;
                break;
            case 2:
                cap.resizeMode = dai::ImgResizeMode::LETTERBOX;
            default:
                throw std::runtime_error("Resize mode argument (every 3rd) must be 0, 1 or 2");
        }
        auto* output = camRgb->requestOutput(cap, true);
        videos.push_back(output->createOutputQueue());
    }

    pipeline.start();

    while(pipeline.isRunning()) {
        size_t videoIndex = 0;
        for(const auto& video : videos) {
            auto videoIn = video->tryGet<dai::ImgFrame>();
            // Get BGR frame from NV12 encoded video frame to show with opencv
            // Visualizing the frame on slower hosts might have overhead
            if(videoIn) {
                // std::cout << "Showing frame on index " << videoIndex << " with size: " << videoIn->getWidth() << "x" << videoIn->getHeight()
                //           << " and stride: " << videoIn->getStride() << " and plane 2 offset: " << videoIn->fb.p2Offset
                //           << " for queue name: " << video->getName() << "\n"
                //           << std::flush;
                cv::imshow("video_" + std::to_string(videoIndex), videoIn->getCvFrame());
            }
            ++videoIndex;
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            pipeline.stop();
            return 0;
        }
    }
    return 0;
}
