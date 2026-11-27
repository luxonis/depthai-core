
#pragma once

#include <cstdint>
#include <depthai/device/Device.hpp>
#include <depthai/device/DeviceBootloader.hpp>
#include <depthai/openvino/OpenVINO.hpp>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace dai {

class XLinkGlobalProfilingLogger {
    XLinkGlobalProfilingLogger();
    ~XLinkGlobalProfilingLogger();

    std::atomic<bool> running{false};
    std::atomic<float> rate{1.0f};
    std::thread thr;

   public:
    static XLinkGlobalProfilingLogger& getInstance();
    XLinkGlobalProfilingLogger(XLinkGlobalProfilingLogger const&) = delete;
    void operator=(XLinkGlobalProfilingLogger const&) = delete;

    void enable(bool enable);
    void setRate(float hz);
    float getRate();
};

}  // namespace dai
