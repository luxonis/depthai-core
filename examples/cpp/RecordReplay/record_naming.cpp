// Showcases multiple record nodes with different names.

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"

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
    try {
        dai::Pipeline pipeline;
        const auto directory = recordingsDir();
        const auto name = std::string("naming_") + dai::makeRecordingTimestamp();
        std::shared_ptr<dai::node::RecordVideo> record;
        for(const auto socket : {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B}) {
            auto camera = pipeline.create<dai::node::Camera>()->build(socket);
            auto* frames = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.f);
            auto encoder = pipeline.create<dai::node::VideoEncoder>();
            encoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);
            frames->link(encoder->input);

            record = pipeline.create<dai::node::RecordVideo>();
            record->setKeepOnDevice(false);
            record->setFps(10);
            record->setOutputDirectory(directory);
            record->setStreamName(dai::toString(socket));
            record->setRecordingName(name);
            encoder->out.link(record->input);
        }

        std::cout << "Saving video to " << (directory / name).string() << "\n";
        if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4) {
            std::cout << "/data/depthai/recordings/" << name << "\n";
        }
        pipeline.start();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while(std::chrono::steady_clock::now() < deadline && pipeline.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        pipeline.stop();
        if(record) {
            if(auto last = record->getStatus()) {
                std::cout << "frames=" << last->frameCount << " path=" << last->path << "\n";
            }
        }
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
    return 0;
}
