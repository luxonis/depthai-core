// Showcases Pipeline.enableHolisticRecord.
// Holistic record attaches a RecordVideo to every Camera source, and a
// RecordMetadataOnly to the IMU, when the pipeline starts. This program does not
// create those nodes. requestOutput sets the size and fps that holistic record
// copies, and the queue keeps that request linked. An unlinked request is
// rejected at start.

#include <algorithm>
#include <argparse/argparse.hpp>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "example_util.hpp"

namespace {

bool videoSensor(const dai::CameraFeatures& feature) {
    if(feature.supportedTypes.empty()) return true;
    for(const auto sensorType : feature.supportedTypes) {
        if(sensorType == dai::CameraSensorType::COLOR || sensorType == dai::CameraSensorType::MONO || sensorType == dai::CameraSensorType::AUTO) {
            return true;
        }
    }
    return false;
}

bool imuConnected(const std::string& name) {
    return !name.empty() && name != "NONE";
}

}  // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("holistic_record", "1.0", argparse::default_arguments::help);
    program.add_argument("--duration").default_value(5.0).scan<'g', double>().help("seconds to record after every source has produced a message");
    program.add_argument("--fps").default_value(10.0).scan<'g', double>();
    program.add_argument("--width").default_value(640).scan<'i', int>().help("recorded width. 640 is a multiple of 32, which RVC2 encoding requires");
    program.add_argument("--height").default_value(480).scan<'i', int>().help("recorded height. 480 is a multiple of 8, which RVC2 encoding requires");
    program.add_argument("--codec").default_value(std::string("H264_MAIN")).help("a VideoEncoder profile, or none for RAW_FRAMES");
    program.add_argument("--compression").default_value(std::string("NONE")).help("mcap compression: NONE, FASTEST, FAST, DEFAULT, SLOW, SLOWEST");
    program.add_argument("--bitrate").default_value(0).scan<'i', int>().help("encoder bitrate. 0 keeps the encoder default");
    program.add_argument("--quality").default_value(80).scan<'i', int>().help("encoder quality, used by MJPEG");
    program.add_argument("--lossless").flag().help("ask the encoder for a lossless stream");
    program.add_argument("--keep-on-device").flag().help("keep the RVC4 device copy after the host archive is packed");
    program.add_argument("--no-sync").flag().help("record each camera on its own clock instead of Sync");
    program.add_argument("--imu-rate").default_value(200).scan<'i', int>().help("accelerometer and gyroscope report rate, used only when an IMU is connected");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n" << program;
        return 1;
    }

    const double duration = program.get<double>("--duration");
    const auto fps = static_cast<float>(program.get<double>("--fps"));
    const int width = program.get<int>("--width");
    const int height = program.get<int>("--height");
    const std::string codecName = program.get<std::string>("--codec");
    const std::string compressionName = program.get<std::string>("--compression");
    const int bitrate = program.get<int>("--bitrate");
    const int quality = program.get<int>("--quality");
    const bool lossless = program.get<bool>("--lossless");
    const bool keepOnDevice = program.get<bool>("--keep-on-device");
    const bool noSync = program.get<bool>("--no-sync");
    const int imuRate = program.get<int>("--imu-rate");

    std::shared_ptr<dai::Device> device;
    struct CloseDevice {
        std::shared_ptr<dai::Device>* device = nullptr;
        ~CloseDevice() {
            if(device != nullptr && *device && !(*device)->isClosed()) (*device)->close();
        }
    } closeDevice{&device};

    try {
        std::optional<dai::VideoEncoderProperties::Profile> codec;
        if(codecName != "none") {
            codec = rr::videoEncoderProfile(codecName);
            if(!codec) {
                std::cout << "Invalid codec " << codecName << ". Use one of " << rr::videoEncoderProfileList() << ", or none\n";
                return 1;
            }
        }
        const auto compression = rr::compressionLevel(compressionName);
        if(!compression) {
            std::cout << "Invalid compression " << compressionName << ". Use one of " << rr::compressionLevelList() << "\n";
            return 1;
        }

        dai::Pipeline pipeline;
        rr::StopPipeline stop{pipeline};
        device = pipeline.getDefaultDevice();
        const auto features = device->getConnectedCameraFeatures();
        if(features.empty()) {
            std::cout << "No cameras found\n";
            return 1;
        }

        const auto output = rr::outputDir() / rr::uniqueName("holistic");
        std::filesystem::create_directories(output);
        dai::RecordConfig config;
        config.outputDir = output;
        config.keepOnDevice = keepOnDevice;
        config.compressionLevel = *compression;
        config.syncCameraOutputs = !noSync;
        config.videoEncoding.bitrate = bitrate;
        config.videoEncoding.quality = quality;
        config.videoEncoding.lossless = lossless;
        if(!codec) {
            config.format = dai::RecordFormat::RAW_FRAMES;
            config.videoEncoding.enabled = false;
        } else {
            config.format = dai::RecordFormat::VIDEO;
            config.videoEncoding.enabled = true;
            config.videoEncoding.profile = *codec;
        }
        pipeline.enableHolisticRecord(config);

        // Create an output queue for each camera on the device, so they are all captured.
        // Otherwise holistic record only records used cameras.
        const auto size = std::make_pair(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height));
        std::vector<std::shared_ptr<dai::node::Camera>> cameras;
        std::vector<std::shared_ptr<dai::MessageQueue>> queues;
        for(const auto& feature : features) {
            std::ostringstream label;
            label << feature.socket << " " << feature.sensorName << " " << feature.width << "x" << feature.height;
            if(!videoSensor(feature)) {
                std::cout << "Skipping " << label.str() << ": holistic record requests NV12\n";
                continue;
            }
            auto camera = pipeline.create<dai::node::Camera>()->build(feature.socket);
            auto queue = camera->requestOutput(size, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, fps)->createOutputQueue(4, false);
            cameras.push_back(camera);
            queues.push_back(queue);
            std::cout << "Recording " << label.str() << " at " << width << "x" << height << "\n";
        }
        if(cameras.empty()) {
            std::cout << "No color or mono camera to record\n";
            return 1;
        }

        // Record IMU if one is connected.
        const auto imuName = device->getConnectedIMU();
        std::shared_ptr<dai::node::IMU> imu;
        std::shared_ptr<dai::MessageQueue> imuQueue;
        if(imuConnected(imuName)) {
            imu = pipeline.create<dai::node::IMU>();
            imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, static_cast<std::uint32_t>(imuRate));
            imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, static_cast<std::uint32_t>(imuRate));
            imu->setBatchReportThreshold(20);
            imuQueue = imu->out.createOutputQueue(4, false);
            std::cout << "Recording IMU " << imuName << " at " << imuRate << " Hz\n";
        } else {
            const char* shown = imuName.empty() ? "empty" : imuName.c_str();
            std::cout << "No IMU on this device (" << shown << "). IMU will not be recorded.\n";
        }

        const char* mode = codec ? codecName.c_str() : "RAW_FRAMES";
        std::cout << "Starting holistic record " << mode << " compression=" << compressionName << " for " << duration << "s at " << fps
                  << " fps, sync=" << (noSync ? "false" : "true") << ".\n";
        std::cout << "Archive directory \"" << output.string() << "\"\n";
        pipeline.start();

        // Wait for cameras and IMU to start.
        std::vector<std::uint8_t> ready(queues.size(), 0);
        bool imuReady = imuQueue == nullptr;
        const auto startupDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while((!std::all_of(ready.begin(), ready.end(), [](std::uint8_t item) { return item != 0; }) || !imuReady) && pipeline.isRunning()
              && std::chrono::steady_clock::now() < startupDeadline) {
            for(std::size_t index = 0; index < queues.size(); index++) {
                if(queues[index]->tryGet<dai::ImgFrame>() != nullptr) ready[index] = 1;
            }
            if(imuQueue != nullptr && imuQueue->tryGet<dai::IMUData>() != nullptr) imuReady = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        const bool camerasReady = std::all_of(ready.begin(), ready.end(), [](std::uint8_t item) { return item != 0; });
        if(!camerasReady || !imuReady) {
            throw std::runtime_error("A camera or the IMU did not produce data during holistic recording");
        }

        // Print frame count every second.
        std::vector<int> counts(queues.size(), 0);
        int imuCount = 0;
        const auto end =
            std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(duration));
        auto nextPrint = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while(std::chrono::steady_clock::now() < end && pipeline.isRunning()) {
            for(std::size_t index = 0; index < queues.size(); index++) {
                if(queues[index]->tryGet<dai::ImgFrame>() != nullptr) counts[index] += 1;
            }
            if(imuQueue != nullptr && imuQueue->tryGet<dai::IMUData>() != nullptr) imuCount += 1;
            if(std::chrono::steady_clock::now() >= nextPrint) {
                std::ostringstream summary;
                for(std::size_t index = 0; index < counts.size(); index++) {
                    if(index != 0) summary << ' ';
                    summary << "cam" << index << "=" << counts[index];
                }
                if(imuQueue != nullptr) summary << " imu=" << imuCount;
                std::cout << summary.str() << "\n";
                nextPrint += std::chrono::seconds(1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if(!pipeline.isRunning()) {
            throw std::runtime_error("Pipeline stopped before the recording duration elapsed");
        }

        // Stop the pipeline and archive outputs.
        pipeline.stop();
        std::optional<std::filesystem::path> archive;
        for(const auto& entry : std::filesystem::directory_iterator(output)) {
            if(!entry.is_regular_file()) continue;
            const auto& path = entry.path();
            if(path.extension() != ".gz") continue;
            if(path.stem().extension() != ".tar") continue;
            if(entry.file_size() == 0) continue;
            archive = path;
        }
        if(!archive) {
            throw std::runtime_error("Holistic recording did not create a nonempty archive");
        }
        std::cout << "holistic record stopped, archive \"" << archive->string() << "\"\n";
        return 0;
    } catch(const std::exception& ex) {
        return rr::classifyRecordFailure(ex.what());
    }
}
