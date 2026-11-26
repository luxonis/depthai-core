#pragma once

#include <mutex>
#include <map>
#include <optional>

namespace dai {
class DeviceTreeDiscovery {

private:
    std::mutex m_mutex;
    std::map<uint32_t, uint32_t> m_camFsyncMap;
    std::optional<uint32_t> m_extFsyncOut;
    
private:
    DeviceTreeDiscovery();
    ~DeviceTreeDiscovery() = default;

    void EnumerateCams();
    void EnumerateSTMFSync();

public:
    DeviceTreeDiscovery(DeviceTreeDiscovery &) = delete;
    void operator=(const DeviceTreeDiscovery &) = delete;

    static DeviceTreeDiscovery& getInstance();

    std::optional<uint32_t> getFsyncPin(uint32_t cameraSocket);
    std::optional<uint32_t> getExternalFrameSyncPin();
};
}
