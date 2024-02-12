#pragma once

#include <ostream>
#include <vector>

#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/common/CameraSensorType.hpp"

inline std::ostream& operator<<(std::ostream& out, const dai::CameraSensorConfig& sensorConfig) {
    dai::Point2f x0y0, x1y1;
    x0y0 = sensorConfig.fov.topLeft();
    x1y1 = sensorConfig.fov.bottomRight();
    out << "{width: " << sensorConfig.width << ", ";
    out << "height: " << sensorConfig.height << ", ";
    out << "minFps: " << sensorConfig.minFps << ", ";
    out << "maxFps: " << sensorConfig.maxFps << ", ";
    out << "fov: " << "[" << x0y0.x << ", " << x0y0.y << ", " << x1y1.x << ", " << x1y1.y << "]" << ", ";
    out << "type: " << sensorConfig.type << "}";
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const std::vector<dai::CameraSensorConfig>& sensorConfigs) {
    out << "[";
    for (auto &sensorConfig : sensorConfigs) {
        out << sensorConfig << ", ";
    }
    out << "]";

    return out;
}

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
    out << "configs: " << camera.configs << ", ";
    out << "calibrationResolution: ";
    if (camera.calibrationResolution) {
        out << camera.calibrationResolution.value();
    } else {
        out << "None";
    }
    out << ", ";
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
