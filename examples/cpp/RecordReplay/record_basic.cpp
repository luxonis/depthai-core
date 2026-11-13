// Showcases record node configuration and basic usage.
// On RVC4 the node records into the device store and, at pipeline stop, if
// --keep-on-device is false copies the video to the host directory and the metadata beside it.

#include <argparse/argparse.hpp>
#include <chrono>
#include <iostream>
#include <optional>
#include <string>

#include "depthai/depthai.hpp"
#include "example_util.hpp"

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
        codec = rr::videoEncoderProfile(codecName);
        if(!codec) {
            std::cout << "Invalid codec " << codecName << " specified. Use one of the following: " << rr::videoEncoderProfileList() << "\n";
            return 1;
        }
    }

    try {
        dai::Pipeline pipeline;
        rr::StopPipeline stop{pipeline};
        auto* frames = rr::addCameraNv12(pipeline, fps);
        auto record = pipeline.create<dai::node::RecordVideo>();
        if(!codec) {
            record->setFormat(dai::RecordFormat::RAW_FRAMES);
            frames->link(record->input);
        } else {
            rr::linkEncoder(pipeline, frames, record, *codec);
        }
        if(!onDevice) {
            const auto stem = rr::outputDir() / rr::uniqueName("basic");
            record->setOutputDirectory(stem);
            std::cout << "Saving " << (codec ? "video" : "raw frames") << " to \"" << record->getOutputDirectory().string() << "\"\n";
        }
        record->setKeepOnDevice(onDevice);
        record->setFps(static_cast<unsigned int>(fps));
        std::cout << "Starting recording with " << codecName << " for " << duration << "s at " << fps << "fps.\n";
        std::cout << "The recording will be " << (onDevice ? "kept on device" : "transferred to host") << ".\n";
        // Calls pipeline.start and prints record node status.
        return rr::recordFor(pipeline, record, std::chrono::seconds(duration));
    } catch(const std::exception& ex) {
        return rr::classifyRecordFailure(ex.what());
    }
}
