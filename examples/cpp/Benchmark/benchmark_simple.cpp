#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    // Create pipeline without implicit device
    dai::Pipeline pipeline;

    // Create a BenchmarkOut node
    // It will listen on the input to get the first message and then send it out at a specified rate
    // The node sends the same message out (creates new pointers), not deep copies.
    auto benchmarkOut = pipeline.create<dai::node::BenchmarkOut>();
    benchmarkOut->setRunOnHost(true);  // The node can run on host or on device
    benchmarkOut->setFps(30);

    // Create a BenchmarkIn node
    // This node is receiving the messages on the input and measuring the FPS and latency.
    // In the case that the input is with BenchmarkOut, the latency measurement is not always possible, as the message is not deep copied,
    // which means that the timestamps stay the same and latency virtually increases over time.
    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);  // The node can run on host or on device
    benchmarkIn->sendReportEveryNMessages(100);
    benchmarkIn->logReportsAsWarnings(false);
    benchmarkIn->setLogLevel(dai::LogLevel::TRACE);

    benchmarkOut->out.link(benchmarkIn->input);
    auto outputQueue = benchmarkIn->report.createOutputQueue();
    auto inputQueue = benchmarkOut->input.createInputQueue();

    pipeline.start();
    auto imgFrame = std::make_shared<dai::ImgFrame>();
    inputQueue->send(imgFrame);
    while(pipeline.isRunning()) {
        auto benchmarkReport = outputQueue->get<dai::BenchmarkReport>();
        std::cout << "FPS is " << benchmarkReport->fps << std::endl;
    }

    return 0;
}