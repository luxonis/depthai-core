#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Sensor config
 */
struct CameraSensorConfig {
    std::int32_t width = -1, height = -1;
    std::int32_t minFps = -1, maxFps = -1;
    CameraSensorType type;
};
DEPTHAI_SERIALIZE_EXT(CameraSensorConfig, width, height, minFps, maxFps, type);

/**
 * CameraFeatures structure
 *
 * Characterizes detected cameras on board
 */
struct CameraFeatures {
    /**
     * Board socket where the camera was detected
     */
    CameraBoardSocket socket = CameraBoardSocket::AUTO;
    /**
     * Camera sensor name, e.g: "IMX378", "OV9282"
     */
    std::string sensorName;
    /**
     * Maximum sensor resolution
     */
    std::int32_t width = -1, height = -1;
    /**
     * Default camera orientation, board dependent
     */
    CameraImageOrientation orientation = CameraImageOrientation::AUTO;
    /**
     * List of supported types of processing for the given camera.
     *
     * For some sensors it's not possible to determine if they are color or mono
     * (e.g. OV9782 and OV9282), so this could return more than one entry
     */
    std::vector<CameraSensorType> supportedTypes;
    /**
     *  Whether an autofocus VCM IC was detected
     */
    bool hasAutofocus = false;
    /**
     * Camera name or alias
     */
    std::string name;
    /**
     * Available sensor configs
     */
    std::vector<CameraSensorConfig> configs;

    DEPTHAI_SERIALIZE(CameraFeatures, socket, sensorName, width, height, orientation, supportedTypes, hasAutofocus, name, configs);
};

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraFeatures& camera) {
    out << "{socket: " << camera.socket << ", ";
    out << "sensorName: " << camera.sensorName << ", ";
    out << "width: " << camera.width << ", ";
    out << "height: " << camera.height << ", ";
    out << "supportedTypes: [";
    for(size_t i = 0; i < camera.supportedTypes.size(); i++) {
        if(i != 0) {
            out << ", ";
        }
        out << camera.supportedTypes[i];
    }
    out << "], ";
    out << "hasAutofocus: " << camera.hasAutofocus << ", ";
    out << "name: " << camera.name << "}";

    return out;
}

inline std::ostream& operator<<(std::ostream& out, const std::vector<dai::CameraFeatures>& cameras) {
    out << "[";
    for(size_t i = 0; i < cameras.size(); i++) {
        if(i != 0) {
            out << ", ";
        }
        out << cameras.at(i);
    }
    out << "]";

    return out;
}