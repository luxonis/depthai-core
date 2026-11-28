#pragma once

#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "utilities/FsyncController.hpp"

namespace dai {
namespace gate {

struct CameraDescription {
    dai::CameraFeatures camera;
    int id;
    int fsyncId;  // TODO(themarpe) - move to CameraFeatures once properly abstracted
    struct {
        std::string camera;
        std::string sensor;
        std::string lens;
        // TODO flash-led, eeprom...
    } v4lpath;
};

class CameraDiscovery {
    CameraDiscovery();
    ~CameraDiscovery() = default;

    mutable std::mutex mtx;
    std::vector<CameraDescription> cameras;
    std::vector<dai::CameraBoardSocket> cameraSockets;
    std::vector<dai::CameraFeatures> cameraFeatures;
    std::unordered_map<dai::CameraBoardSocket, std::string> cameraNames;

   public:
    static CameraDiscovery& getInstance();
    CameraDiscovery(CameraDiscovery const&) = delete;
    void operator=(CameraDiscovery const&) = delete;

    std::vector<CameraDescription> forceEnumerateCameras();
    std::vector<CameraDescription> getConnectedCameraDescriptions() const;
    std::vector<dai::CameraBoardSocket> getConnectedCameras() const;
    std::vector<dai::CameraFeatures> getConnectedCameraFeatures() const;
    std::unordered_map<dai::CameraBoardSocket, std::string> getConnectedCameraSensorNames() const;

    dai::CameraBoardSocket getCameraByName(std::string) const;                                      // AUTO if not available
    int getCameraId(std::string name) const;                                                        // -1 if not available
    int getCameraId(dai::CameraBoardSocket socket) const;                                           // -1 if not available
    int getCameraId(std::string name, dai::CameraBoardSocket socket) const;                         // -1 if not available; matches name or socket
    dai::CameraFeatures getCameraFeatures(std::string name, dai::CameraBoardSocket socket) const;   // socket = AUTO if not available; matches name or socket
    CameraDescription getCameraDescription(std::string name, dai::CameraBoardSocket socket) const;  // id = -1 if not available; matches name or socket
    std::vector<std::pair<uint32_t, uint32_t>> getAvailableISPScalers(dai::CameraBoardSocket socket) const;
    void forceCameraDiscovery();

    void overrideConnectedCameraFeatures(std::vector<dai::CameraFeatures> features);

   private:
    bool initialized{false};
    void maybeInit();
};

}  // namespace gate

}  // namespace dai
