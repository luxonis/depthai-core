// Showcases record node configuration and basic usage.
// On RVC4 the node records into the device store and, at pipeline stop, if
// --keep-on-device is false copies the video to the host directory and the metadata beside it.

#include <argparse/argparse.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
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

std::optional<dai::VideoEncoderProperties::Profile> videoEncoderProfile(const std::string& name) {
    using Profile = dai::VideoEncoderProperties::Profile;
    if(name == "H264_BASELINE") return Profile::H264_BASELINE;
    if(name == "H264_HIGH") return Profile::H264_HIGH;
    if(name == "H264_MAIN") return Profile::H264_MAIN;
    if(name == "H265_MAIN") return Profile::H265_MAIN;
    if(name == "MJPEG") return Profile::MJPEG;
    return std::nullopt;
}

}  // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("record_basic", "1.0", argparse::default_arguments::help);
    program.add_argument("--duration").default_value(2).scan<'i', int>();
    program.add_argument("--keep-on-device").flag().help("Keep the RVC4 device copy after the host files are written");
    program.add_argument("--fps").default_value(10).scan<'i', int>();
    program.add_argument("--codec").default_value(std::string("H264_MAIN")).help("A codec supported by videoEncoder or none for RAW_FRAMES");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n" << program;
        return 1;
    }

    const int duration = program.get<int>("--duration");
    const bool onDevice = program.get<bool>("--keep-on-device");
    const int fps = program.get<int>("--fps");
    const std::string codecName = program.get<std::string>("--codec");

    std::optional<dai::VideoEncoderProperties::Profile> codec;
    if(codecName != "none") {
        codec = videoEncoderProfile(codecName);
        if(!codec) {
            std::cout << "Invalid codec " << codecName
                      << " specified. Use one of the following: ['H264_BASELINE', 'H264_HIGH', 'H264_MAIN', 'H265_MAIN', 'MJPEG']\n";
            return 1;
        }
    }

    try {
        dai::Pipeline pipeline;
        auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        auto* frames = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, static_cast<float>(fps));
        auto record = pipeline.create<dai::node::RecordVideo>();
        if(!codec) {
            record->setFormat(dai::RecordFormat::RAW_FRAMES);
            frames->link(record->input);
        } else {
            auto encoder = pipeline.create<dai::node::VideoEncoder>();
            encoder->setProfile(*codec);
            frames->link(encoder->input);
            encoder->out.link(record->input);
        }

        const auto directory = recordingsDir();
        const auto name = std::string("basic_") + dai::makeRecordingTimestamp();
        record->setOutputDirectory(directory);
        record->setRecordingName(name);
        record->setKeepOnDevice(onDevice);
        record->setFps(static_cast<unsigned int>(fps));
        std::cout << "Saving " << (codec ? "video" : "raw frames") << " to \"" << (directory / name).string() << "\"\n";
        if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4) {
            std::cout << "/data/depthai/recordings/" << name << "\n";
        }
        std::cout << "Starting recording with " << codecName << " for " << duration << "s at " << fps << "fps.\n";
        std::cout << "The recording will be " << (onDevice ? "kept on device" : "transferred to host") << ".\n";

        pipeline.start();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
        while(std::chrono::steady_clock::now() < deadline && pipeline.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
