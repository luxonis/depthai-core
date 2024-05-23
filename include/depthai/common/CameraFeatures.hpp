#pragma once

#include <ostream>
#include <vector>

#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/common/CameraSensorType.hpp"

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraFeatures& camera) {
    out << "{socket: " << camera.socket << ", ";
    out << "sensorName: " << camera.sensorName << ", ";
    out << "width: " << camera.width << ", ";
    out << "height: " << camera.height << ", ";
    out << "orientation: " << camera.orientation << ", ";
    out << "supportedTypes: [";
    for(size_t i = 0; i < camera.supportedTypes.size(); i++) {
        if(i != 0) {
            out << ", ";
        }
        out << camera.supportedTypes[i];
    }
    out << "], ";
    out << "hasAutofocus: " << camera.hasAutofocus << ", ";
    out << "hasAutofocusIC: " << camera.hasAutofocusIC << ", ";
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

inline std::ostream& operator<<(std::ostream& out, const dai::CameraSensorConfig& config) {
    out << "{width: " << config.width << ", ";
    out << "height: " << config.height << ", ";
    out << "minFps: " << config.minFps << ", ";
    out << "maxFps: " << config.maxFps << ", ";
    out << "type: " << config.type << ", ";
    out << "fov: "
        << "{x:" << config.fov.x << ", ";
    out << "y: " << config.fov.y << ", ";
    out << "width: " << config.fov.width << ", ";
    out << "height: " << config.fov.height << "}";
    out << "}";
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const std::vector<dai::CameraSensorConfig>& configs) {
    out << "[";
    for(size_t i = 0; i < configs.size(); i++) {
        if(i != 0) {
            out << ", ";
        }
        out << configs.at(i);
    }
    out << "]";

    return out;
}
