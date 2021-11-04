#include <chrono>
#include <string>

#include "depthai/depthai.hpp"

dai::Pipeline getCustomPipeline(std::string imageTuningPath) {
    dai::Pipeline pipeline;
    auto rgbCamera = pipeline.create<dai::node::ColorCamera>();
    rgbCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
    rgbCamera->setInterleaved(false);
    rgbCamera->setPreviewSize(1920, 1080);
    rgbCamera->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    rgbCamera->initialControl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);

    if (!imageTuningPath.empty()) {
        pipeline.setCameraTuningBlobPath(imageTuningPath);
    }

    rgbCamera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    // scaling the output image size to 1080P
    rgbCamera->setIspScale(1, 2);

    // UVC
    auto uvc = pipeline.create<dai::node::UVC>();
    rgbCamera->video.link(uvc->input);

    // LED
    uvc->setGpiosOnInit({{58,0}, {37,0}, {34,0}});
    uvc->setGpiosOnStreamOn({{58,1}, {37,1}, {34,1}});
    uvc->setGpiosOnStreamOff({{58,0}, {37,0}, {34,0}});

    // Create an UAC (USB Audio Class) node
    auto uac = pipeline.create<dai::node::UAC>();
    uac->setStreamBackMic(false);
    uac->setMicGainDecibels(28);

    return pipeline;
}

std::vector<uint8_t> getDapPackage(std::string path) {
    std::ifstream file(path, std::ios::binary);
    if(!file.is_open()) throw std::runtime_error("Cannot open DAP file");

    return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

int main(int argc, char** argv) {
    using namespace std::chrono;

    bool flashBootloader = false;
    bool flashPipeline = false;
    bool flashDap = false;
    std::string dapPath = "";
    std::string tuningPath = ""; // TODO

    if(argc == 2) {
        auto option = std::string(argv[1]);
        if (option == "-fb") {
            flashBootloader = true;
        } else if (option == "-f") {
            flashPipeline = true;
        } else {
            flashDap = true;
            dapPath = option;
        }
    } else {
        std::cout <<
            "Requires one option:\n"
            "    -fb     -- to flash bootloader\n"
            "    -f      -- to flash predefined UVC/UAC application/pipeline\n"
            "    <path>  -- to flash specified DAP file\n"
            ;
        return -1;
    }

    bool found = false;
    dai::DeviceInfo info;
    std::tie(found, info) = dai::DeviceBootloader::getFirstAvailableDevice();
    if(!found) {
        std::cout << "No device found to flash. Exiting\n";
        return -1;
    }
    std::cout << "Connecting to bootloader at MXID: " << info.getMxId() << "\n";
    dai::DeviceBootloader bl(info);

    std::cout << std::setprecision(2) << std::fixed;

    // Create a progress callback lambda
    auto progress = [](float p) {
        std::cout << "Flashing progress... " << (p * 100) << "%\n";
    };

    auto tstart = steady_clock::now();
    bool success = false;
    std::string message;

    if (flashBootloader) {
        std::cout << "Flashing bootloader...\n";
        tstart = steady_clock::now();
        std::tie(success, message) = bl.flashBootloader(progress);
    }

    if (flashPipeline) {
        std::cout << "Flashing firmware and pipeline...\n";
        auto pipeline = getCustomPipeline(tuningPath);
        tstart = steady_clock::now();
        std::tie(success, message) = bl.flash(progress, pipeline);
    }

    if (flashDap) {
        std::cout << "Flashing DAP package...\n";
        auto package = getDapPackage(dapPath);
        tstart = steady_clock::now();
        std::tie(success, message) = bl.flashDepthaiApplicationPackage(progress, package);
    }

    if(success) {
        std::cout << "Flashing successful. Took "
                  << duration_cast<milliseconds>(steady_clock::now() - tstart).count()/1000.
                  << " seconds\n";
    } else {
        std::cout << "Flashing failed: " << message << "\n";
        return -1;
    }

    return 0;
}
