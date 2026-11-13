// Showcases starting and stopping the recording from host.
// Note the usage of H.264 when running on host which creates segments of 30 decodable frames
// and may need a pretrigger buffer to work correctly.

#include <argparse/argparse.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "example_util.hpp"

int main(int argc, char** argv) {
    argparse::ArgumentParser program("record_start_stop", "1.0", argparse::default_arguments::help);
    program.add_argument("--pretrigger-size")
        .default_value(0)
        .scan<'i', int>()
        .help("The amount of frames kept on standby while the node is not recording. Used to see what happened a few seconds before the recording started.");
    program.add_argument("--cycles").default_value(1).scan<'i', int>().help("number of segments recorded");
    program.add_argument("--segment-duration-limit")
        .default_value(-1.0)
        .scan<'g', double>()
        .help("a limit on the length of a segment independent of a stop command");
    program.add_argument("--segment-frames-limit")
        .default_value(-1)
        .scan<'i', int>()
        .help("a limit on the amount of frames a segment can have independent of a stop command");
    program.add_argument("--segment-interval").default_value(1.0).scan<'g', double>().help("time between the STOP and the START command");
    program.add_argument("--segment-duration").default_value(4.0).scan<'g', double>().help("time between the START and the STOP command");
    program.add_argument("--fps").default_value(10).scan<'i', int>();
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n" << program;
        return 1;
    }

    const int fps = program.get<int>("--fps");
    const int cycles = program.get<int>("--cycles");
    const int pretriggerFrames = program.get<int>("--pretrigger-size");
    const double duration = program.get<double>("--segment-duration");
    const double interval = program.get<double>("--segment-interval");
    const int maxFrames = program.get<int>("--segment-frames-limit");
    const double maxDuration = program.get<double>("--segment-duration-limit");

    try {
        dai::Pipeline pipeline;
        rr::StopPipeline stop{pipeline};
        auto* frames = rr::addCameraNv12(pipeline, fps);
        auto record = pipeline.create<dai::node::RecordVideo>();
        rr::linkEncoder(pipeline, frames, record);
        const auto stem = rr::outputDir() / rr::uniqueName("start-stop");
        record->setOutputDirectory(stem);
        std::cout << "Saving video to \"" << record->getRecordVideoFile().string() << "\"\n";
        std::cout << "Saving metadata to \"" << record->getRecordMetadataFile().string() << "\"\n";
        record->setFps(static_cast<unsigned int>(fps));
        if(pretriggerFrames < 0) {
            record->setWaitForTrigger(false);
        } else {
            record->setPreTriggerFrames(pretriggerFrames);
            record->setWaitForTrigger(true);
        }
        auto control = record->inputControl.createInputQueue();
        auto status = record->status.createOutputQueue(16, false);
        std::cout << "RecordVideo runOnHost=" << (record->runOnHost() ? "true" : "false") << "\n";
        const int code = rr::startOrClassify(pipeline);
        if(code != 0) return code;
        const auto maxDurationMs = static_cast<std::int32_t>(maxDuration * 1000.0);
        for(int i = 0; i < cycles; i++) {
            std::this_thread::sleep_for(std::chrono::duration<double>(interval));
            std::cout << "sending start()\n";
            control->send(dai::RecordControl::start(maxFrames, maxDurationMs));
            std::this_thread::sleep_for(std::chrono::duration<double>(duration));
            std::cout << "sending stop()\n";
            control->send(dai::RecordControl::stop());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // Calls pipeline.stop and prints the backlog of record node statuses.
        rr::finishRecording(pipeline, record, status);
        return 0;
    } catch(const std::exception& ex) {
        return rr::classifyRecordFailure(ex.what());
    }
}
