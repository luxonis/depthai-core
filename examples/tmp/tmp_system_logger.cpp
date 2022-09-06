#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

void printSystemInformation(dai::SystemInformationS3 info) {
    printf("Ddr used / total - %.2f / %.2f MiB\n", info.ddrMemoryUsage.used / (1024.0f * 1024.0f), info.ddrMemoryUsage.total / (1024.0f * 1024.0f));
    const auto& t = info.chipTemperature;
    printf("Chip temperature - average: %.2f, css: %.2f, mss: %.2f, nce: %.2f, soc: %.2f\n", t.average, t.css, t.mss, t.nce, t.soc);
    printf("Cpu average usage: %.2f %%\n", info.cpuAvgUsage.average * 100);
    for(int i = 0; i < info.cpuUsages.size(); i++) {
        printf("Cpu %d usage: %.2f %%\n", i, info.cpuUsages[i].average * 100);
    }
    printf("----------------------------------------\n");
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto sysLog = pipeline.create<dai::node::SystemLogger>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    xout->setStreamName("sysinfo");

    // Properties
    sysLog->setRate(2.0f);  // 1 hz updates

    // Linking
    sysLog->out.link(xout->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the system info
    auto qSysInfo = device.getOutputQueue("sysinfo", 4, false);

    while(true) {
        auto sysInfo = qSysInfo->get<dai::SystemInformationS3>();
        if(sysInfo){
            printSystemInformation(*sysInfo);
        }
    }
    return 0;
}
