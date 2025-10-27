#include <atomic>
#include <csignal>
#include <iostream>

#include "depthai/depthai.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int signum) {
    quitEvent = true;
}

void printSystemInformation(const dai::SystemInformation& info) {
    const float m = 1024.0f * 1024.0f;  // MiB
    std::cout << "Ddr used / total - " << info.ddrMemoryUsage.used / m << " / " << info.ddrMemoryUsage.total / m << " MiB" << std::endl;
    std::cout << "Cmx used / total - " << info.cmxMemoryUsage.used / m << " / " << info.cmxMemoryUsage.total / m << " MiB" << std::endl;
    std::cout << "LeonCss heap used / total - " << info.leonCssMemoryUsage.used / m << " / " << info.leonCssMemoryUsage.total / m << " MiB" << std::endl;
    std::cout << "LeonMss heap used / total - " << info.leonMssMemoryUsage.used / m << " / " << info.leonMssMemoryUsage.total / m << " MiB" << std::endl;

    const auto& t = info.chipTemperature;
    std::cout << "Chip temperature - average: " << t.average << ", css: " << t.css << ", mss: " << t.mss << ", upa: " << t.upa << ", dss: " << t.dss
              << std::endl;

    std::cout << "Cpu usage - Leon CSS: " << info.leonCssCpuUsage.average * 100.0f << "%, Leon MSS: " << info.leonMssCpuUsage.average * 100.0f << "%"
              << std::endl;
    std::cout << "----------------------------------------" << std::endl;
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create pipeline
    dai::Pipeline pipeline;

    // Create system logger node
    auto sysLog = pipeline.create<dai::node::SystemLogger>();
    sysLog->setRate(1.0f);  // 1 Hz

    // Create output queue
    auto sysLogQueue = sysLog->out.createOutputQueue(4, false);

    // Start pipeline
    pipeline.start();
    std::cout << "System logger started. Press Ctrl+C to quit." << std::endl;

    while(pipeline.isRunning() && !quitEvent) {
        auto sysInfo = sysLogQueue->get<dai::SystemInformation>();
        if(sysInfo == nullptr) continue;

        printSystemInformation(*sysInfo);
    }

    // Cleanup
    pipeline.stop();
    pipeline.wait();

    return 0;
}
