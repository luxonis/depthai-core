// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    dai::DeviceInfo info("10.12.110.28");
    info.protocol = X_LINK_TCP_IP;
    info.state = X_LINK_GATE;
    info.platform = X_LINK_RVC4;
    const auto device = std::make_shared<dai::Device>(info);

    dai::Pipeline pipeline(device);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::Camera>();

    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    camRgb->setSize(1920, 1080);

    std::vector<std::shared_ptr<dai::MessageQueue>> videos;
    for(int index = 0; index < 1; ++index) {
        dai::ImgFrameCapability cap;
        cap.size.value = std::pair(1920, 1080);
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
            cv::imshow("video_" + std::to_string(videoIndex), videoIn->getCvFrame());
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
