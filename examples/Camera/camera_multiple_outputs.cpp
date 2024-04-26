// Includes common necessary includes for development using depthai library
#include <stdexcept>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    dai::DeviceInfo info("10.12.110.28");
    info.protocol = X_LINK_TCP_IP;
    info.state = X_LINK_GATE;
    info.platform = X_LINK_RVC4;
    const auto device = std::make_shared<dai::Device>(info);
    if(argc < 3 || (argc - 1) % 2 != 0) {
        throw std::runtime_error("USAGE: ./camera_multiple_outputs 1920 1080 640 480");
    }
    std::vector<std::pair<uint32_t, uint32_t>> sizes;
    for(int index = 1; index < argc - 1; index += 2) {
        sizes.emplace_back(std::stoul(argv[index]), std::stoul(argv[index + 1]));
    }

    dai::Pipeline pipeline(device);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::Camera>();

    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    if(sizes.empty()) {
        throw std::runtime_error("internal error to few sizes");
    }
    // camRgb->setSize(sizes[0]);

    std::vector<std::shared_ptr<dai::MessageQueue>> videos;
    for(const auto& size : sizes) {
        dai::ImgFrameCapability cap;
        cap.size.value = size;
        auto* output = camRgb->requestNewOutput(cap);
        videos.push_back(output->createQueue());
    }

    /*
    camRgb->setSize(1920, 1080);
    auto video = camRgb->video.createQueue();
    */

    /*
    dai::ImgFrameCapability cap;
    cap.size.value = std::pair(1920, 1080);
    auto* output = camRgb->requestNewOutput(cap);
    auto video = output->createQueue();
    */

    pipeline.start();

    while(true) {
        size_t videoIndex = 0;
        for(const auto& video : videos) {
            std::cout << "Waiting for frame on index " << videoIndex << "\n" << std::flush;
            auto videoIn = videoIndex == 0 ? video->get<dai::ImgFrame>() : video->tryGet<dai::ImgFrame>();
            std::cout << "Showing frame on index " << videoIndex << "\n" << std::flush;

            // Get BGR frame from NV12 encoded video frame to show with opencv
            // Visualizing the frame on slower hosts might have overhead
            if(videoIn) {
                cv::imshow("video_" + std::to_string(videoIndex), videoIn->getCvFrame());
            } else {
                std::cout << "Video frame on index " << videoIndex << " was null\n" << std::flush;
            }
            ++videoIndex;
        }

        /*
      auto videoIn = video->get<dai::ImgFrame>();

      // Get BGR frame from NV12 encoded video frame to show with opencv
      // Visualizing the frame on slower hosts might have overhead
      cv::imshow("video", videoIn->getCvFrame());
      */

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            pipeline.stop();
            return 0;
        }
    }
    return 0;
}
