#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <string>

#include "depthai/depthai.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

namespace {

std::filesystem::path recordingsDir() {
    namespace fs = std::filesystem;
    if(const char* xdg = std::getenv("XDG_CACHE_HOME"); xdg != nullptr && xdg[0] != '\0') {
        fs::path path = fs::path(xdg) / "depthai" / "recordings";
        fs::create_directories(path);
        return path;
    }
#if defined(__APPLE__)
    const char* home = std::getenv("HOME");
    fs::path root = (home != nullptr && home[0] != '\0') ? fs::path(home) / "Library" / "Caches" : fs::path();
#elif defined(_WIN32)
    const char* local = std::getenv("LOCALAPPDATA");
    fs::path root;
    if(local != nullptr && local[0] != '\0') {
        root = fs::path(local);
    } else {
        const char* home = std::getenv("USERPROFILE");
        root = (home != nullptr && home[0] != '\0') ? fs::path(home) / "AppData" / "Local" : fs::path();
    }
#else
    const char* home = std::getenv("HOME");
    fs::path root = (home != nullptr && home[0] != '\0') ? fs::path(home) / ".cache" : fs::path();
#endif
    if(root.empty()) {
        throw std::runtime_error("Could not resolve the user cache directory");
    }
    fs::path path = root / "depthai" / "recordings";
    fs::create_directories(path);
    return path;
}

}  // namespace

int main() {
    constexpr int fps = 30;
    try {
        dai::Pipeline pipeline;
        auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        auto* frames = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, static_cast<float>(fps));
        auto videoQueue = frames->createOutputQueue();

        auto encoder = pipeline.create<dai::node::VideoEncoder>();
        encoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);
        frames->link(encoder->input);

        auto record = pipeline.create<dai::node::RecordVideo>();
        encoder->out.link(record->input);
        const auto directory = recordingsDir();
        const auto name = std::string("start-stop_") + dai::makeRecordingTimestamp();
        record->setOutputDirectory(directory);
        record->setRecordingName(name);
        record->setFps(static_cast<unsigned int>(fps));
        record->setWaitForTrigger(true);
        auto control = record->inputControl.createInputQueue();

        std::cout << "Saving video to " << (directory / name).string() << "\n";
        if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4) {
            std::cout << "/data/depthai/recordings/" << name << "\n";
        }
        std::cout << "Press s to start or stop recording. Press q to quit.\n";

        pipeline.start();
        bool recording = false;
        while(pipeline.isRunning()) {
            std::shared_ptr<dai::ImgFrame> frame;
            try {
                frame = videoQueue->get<dai::ImgFrame>();
            } catch(const dai::MessageQueue::QueueException&) {
                break;
            }
            if(frame == nullptr) continue;
            cv::imshow("video", frame->getCvFrame());
            const int key = cv::waitKey(1) & 0xFF;
            if(key == 'q' || key == 'Q') break;
            if(key == 's' || key == 'S') {
                if(recording) {
                    control->send(dai::RecordControl::stop());
                    std::cout << "stop\n";
                    recording = false;
                } else {
                    control->send(dai::RecordControl::start());
                    std::cout << "start\n";
                    recording = true;
                }
            }
        }
        pipeline.stop();
        if(auto last = record->getStatus()) {
            std::cout << "frames=" << last->frameCount << " path=" << last->path << "\n";
        }
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
    return 0;
}
