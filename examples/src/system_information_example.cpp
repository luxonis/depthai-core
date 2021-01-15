

#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

void printSystemInformation(dai::SystemInformation);

int main(){
    using namespace std;

    dai::Pipeline pipeline;
    auto sysLog = pipeline.create<dai::node::SystemLogger>();
    auto xout = pipeline.create<dai::node::XLinkOut>();
    
    // properties
    sysLog->setRate(1.0f); // 1 hz updates
    xout->setStreamName("sysinfo");

    // links
    sysLog->out.link(xout->input);

    // Connect to device
    dai::Device device(pipeline);

    // Create 'sysinfo' queue
    auto queue = device.getOutputQueue("sysinfo");

    // Query device (before pipeline starts)
    dai::MemoryInfo ddr = device.getDdrMemoryUsage();
    printf("Ddr used / total - %.2f / %.2f MiB\n", ddr.used / (1024.0f*1024.0f), ddr.total / (1024.0f*1024.0f));

    // Start pipeline 
    device.startPipeline();

    while(1){
        auto sysInfo = queue->get<dai::SystemInformation>();
        printSystemInformation(*sysInfo);
    }
}

void printSystemInformation(dai::SystemInformation info){

    printf("Ddr used / total - %.2f / %.2f MiB\n", info.memoryDdrUsage.used / (1024.0f*1024.0f), info.memoryDdrUsage.total / (1024.0f*1024.0f));
    printf("LeonOs heap used / total - %.2f / %.2f MiB\n", info.memoryLeonOsUsage.used / (1024.0f*1024.0f), info.memoryLeonOsUsage.total / (1024.0f*1024.0f));
    printf("LeonRt heap used / total - %.2f / %.2f MiB\n", info.memoryLeonRtUsage.used / (1024.0f*1024.0f), info.memoryLeonRtUsage.total / (1024.0f*1024.0f));

    const auto& t = info.chipTemperature;
    printf("Chip temperature - average: %.2f, css: %.2f, mss: %.2f, upa0: %.2f, upa1: %.2f\n", t.average, t.css, t.mss, t.upa0, t.upa1);

}