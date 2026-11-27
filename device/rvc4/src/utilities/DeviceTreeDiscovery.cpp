#include "DeviceTreeDiscovery.hpp"

#include <mutex>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <mutex>
#include <cassert>
#include <chrono>
#include <thread>

#include "FsyncController.hpp"
#include "utilities/ErrorMacros.hpp"
#include "camera/LuxonisSidechannel.hpp"
#include <luxonis_sidechannel_data.h>

namespace dai {

DeviceTreeDiscovery& DeviceTreeDiscovery::getInstance()
{
    static std::mutex mutex_;
    std::lock_guard<std::mutex> lock(mutex_);
    static DeviceTreeDiscovery instance_;
    return instance_;
}

DeviceTreeDiscovery::DeviceTreeDiscovery()
:
    m_extFsyncOut(std::nullopt)
{
    EnumerateCams();
    EnumerateSTMFSync();
}

void DeviceTreeDiscovery::EnumerateCams()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    namespace fs = std::filesystem;

    std::string deviceTreePath = "/proc/device-tree";
    fs::path aliases{deviceTreePath + "/aliases"};
    for(const auto& ep : fs::directory_iterator(aliases)) {
        int index = -1;
        std::string searchPath = aliases.string() + "/cam_";
        std::string searchPathCondition = searchPath + "%d";

        if(sscanf(ep.path().c_str(), searchPathCondition.c_str(), &index) == 1) {
            std::string absPath;
            try {
                std::ifstream alias(ep.path().c_str(), std::ios::binary);
                std::stringstream aliasPath;
                aliasPath << alias.rdbuf();
                alias.close();
                absPath = std::string(aliasPath.str().c_str());  // DTS names end with \0; going through c_str() drops it
            } catch(...) {
                DAI_ERROR("DeviceTreeDiscovery: Error reading alias file for cam_{}!", index);
                continue;
            }

            std::string fsyncOutputProperty = deviceTreePath + absPath + "/fsync-stm-output";

            if (!fs::exists(fsyncOutputProperty)) {
                DAI_ERROR("DeviceTreeDiscovery: fsync-stm-output property not found for cam_{}!", index);
                continue;
            }

            uint32_t fsyncOutput = -1;
            try {
                std::ifstream(fsyncOutputProperty, std::ios::binary).read(reinterpret_cast<char*>(&fsyncOutput), sizeof(fsyncOutput));
                fsyncOutput = __bswap_32(fsyncOutput);
            } catch(...) {
                DAI_ERROR("DeviceTreeDiscovery: Error reading fsync-stm-output property for cam_{}!", index);
                continue;
            }

            std::string csiIndexProperty = deviceTreePath + absPath + "/csiphy-sd-index";

            if (!fs::exists(csiIndexProperty)) {
                DAI_ERROR("DeviceTreeDiscovery: csiphy-sd-index property not found for cam_{}!", index);
                continue;
            }

            uint32_t csiIndex = -1;
            try {
                std::ifstream(csiIndexProperty, std::ios::binary).read(reinterpret_cast<char*>(&csiIndex), sizeof(csiIndex));
                csiIndex = __bswap_32(csiIndex);
            } catch(...) {
                DAI_ERROR("DeviceTreeDiscovery: Error reading csiphy-sd-index property for cam_{}!", index);
                continue;
            }

            auto cb = LuxonisSidechannel::getInstance().ops.camera_socket_for_csi_get;
            lux_sc_camera_socket_for_csi_get_ret_t result;
            if (cb) {
                result = (lux_sc_camera_socket_for_csi_get_ret_t)cb(csiIndex);
            } else {
                DAI_INFO("DeviceTreeDiscovery: No camera_socket_for_csi_get callback found");
                continue;
            }

            if (!result.found) {
                DAI_ERROR("DeviceTreeDiscovery: Couldn't find socket for CSI index {}", csiIndex);
                continue;
            }

            DAI_LOG("Found mapping for cam_{}: csiIndex CSI{}, fsyncOutput {}, socket {}", index, csiIndex, fsyncOutput, result.depthai_socket);

            m_camFsyncMap.emplace(std::make_pair(result.depthai_socket, fsyncOutput));
        }
    }
}

void DeviceTreeDiscovery::EnumerateSTMFSync()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    namespace fs = std::filesystem;

    std::string deviceTreePath = "/proc/device-tree";
    fs::path fsyncAlias{deviceTreePath + "/aliases/stm_fsync"};

    if (!fs::exists(fsyncAlias)) {
        if (!dai::gate::FsyncController::getInstance().supportsExternalFrameSyncMode()) {
            DAI_INFO("stm_fsync device tree alias not found! OS probably does not support dynamic fsync pin assignment.");
        } else {
            DAI_ERROR("stm_fsync device tree alias not found!");
        }
        return;
    }

    std::string absPath;
    try {
        std::ifstream alias(fsyncAlias.c_str(), std::ios::binary);
        std::stringstream aliasPath;
        aliasPath << alias.rdbuf();
        alias.close();
        absPath = std::string(aliasPath.str().c_str());  // DTS names end with \0; going through c_str() drops it
    } catch(...) {
        DAI_ERROR("Error reading stm fsync alias file!")
        return;
    }

    std::string extFsyncOutProperty = deviceTreePath + absPath + "/external-output-pin";

    if (!fs::exists(extFsyncOutProperty)) {
        DAI_ERROR("external-output-pin property not found for stm_fsync!");
        return;
    }

    uint32_t extFsyncOut = 0;
    try {
        std::ifstream(extFsyncOutProperty, std::ios::binary).read(reinterpret_cast<char*>(&extFsyncOut), sizeof(extFsyncOut));
        extFsyncOut = __bswap_32(extFsyncOut);
    } catch(...) {
        DAI_ERROR("Error reading external-output-pin property!")
        return;
    }

    m_extFsyncOut = extFsyncOut;
}

std::optional<uint32_t> DeviceTreeDiscovery::getFsyncPin(uint32_t cameraSocket)
{   
    auto found = m_camFsyncMap.find(cameraSocket);
    if (found == std::end(m_camFsyncMap)) {
        return std::nullopt;
    }

    return std::optional<uint32_t>(found->second);
}

std::optional<uint32_t> DeviceTreeDiscovery::getExternalFrameSyncPin()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_extFsyncOut;
}

}