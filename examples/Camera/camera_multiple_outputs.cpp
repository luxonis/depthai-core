// Includes common necessary includes for development using depthai library
#include <stdexcept>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    const char* valueCStr = std::getenv("DEPTHAI_TESTS_IP");
    if(valueCStr == nullptr) {
        throw std::runtime_error("Please set device ip using environment variable DEPTHAI_TESTS_IP");
    }
    dai::DeviceInfo info(valueCStr);
    info.protocol = X_LINK_TCP_IP;
    info.state = X_LINK_GATE;
    info.platform = X_LINK_RVC4;
    const auto device = std::make_shared<dai::Device>(info);
    if(argc < 4 || (argc - 1) % 4 != 0) {
        throw std::runtime_error(
            "USAGE: ./camera_multiple_outputs 1920 1080 0 0  640 480 1 1\n"
            "WHERE first 0 is resize mode: 0 == CROP, 1 == STRETCH, 2 == LETTERBOX\n"
            "AND second 0 is CAM_A, ...");
    }
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t, int32_t>> sizes;
    for(int index = 1; index < argc - 1; index += 4) {
        sizes.emplace_back(std::stoul(argv[index]), std::stoul(argv[index + 1]), std::stoul(argv[index + 2]), std::stol(argv[index + 3]));
    }

    dai::Pipeline pipeline(device);

    if(sizes.empty()) {
        throw std::runtime_error("internal error to few sizes");
    }
    std::map<dai::CameraBoardSocket, std::shared_ptr<dai::node::Camera>> cameras;
    std::vector<std::shared_ptr<dai::MessageQueue>> videos;
    for(const auto& size : sizes) {
        dai::ImgFrameCapability cap;
        // cap.encoding = dai::ImgFrame::Type::RGB888i;
        // cap.encoding = dai::ImgFrame::Type::NV12;
        // cap.encoding = dai::ImgFrame::Type::BGR888i;
        cap.encoding = dai::ImgFrame::Type::RGB888i;
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
        if(std::get<3>(size) < static_cast<int32_t>(dai::CameraBoardSocket::CAM_A) || std::get<3>(size) > static_cast<int32_t>(dai::CameraBoardSocket::CAM_J)) {
            throw std::runtime_error("Resize mode argument (every 4th) must be from CAM_A to CAM_J");
        }
        const auto camSocket = static_cast<dai::CameraBoardSocket>(std::get<3>(size));
        std::shared_ptr<dai::node::Camera> camera;
        if(const auto& it = cameras.find(camSocket); it != cameras.end()) {
            std::cout << "FOUND camera using it\n" << std::flush;
            camera = it->second;
        } else {
            std::cout << "DIDN't find camera, creating it\n" << std::flush;
            camera = pipeline.create<dai::node::Camera>();
            camera->setBoardSocket(camSocket);
            cameras[camSocket] = camera;
        }
        auto* output = camera->requestOutput(cap, true);
        videos.push_back(output->createOutputQueue());
    }

    pipeline.start();

    while(true) {
        size_t videoIndex = 0;
        for(const auto& video : videos) {
            // std::cout << "Waiting for frame on index " << videoIndex << "\n" << std::flush;
            auto videoIn = video->tryGet<dai::ImgFrame>();
            // Get BGR frame from NV12 encoded video frame to show with opencv
            // Visualizing the frame on slower hosts might have overhead
            if(videoIn) {
                std::cout << "Showing frame on index " << videoIndex << " with size: " << videoIn->getWidth() << "x" << videoIn->getHeight()
                          << " and stride: " << videoIn->getStride() << " and plane 2 offset: " << videoIn->fb.p2Offset
                          << " for queue name: " << video->getName() << "\n"
                          << std::flush;
                cv::imshow("video_" + std::to_string(videoIndex), videoIn->getCvFrame());
            } else {
                // std::cout << "Video frame on index " << videoIndex << " was null\n" << std::flush;
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
