#pragma once

// std
#include <cstdint>

// project
#include <stdexcept>
#include <string>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

struct Keypoint {
    Point3f coordinates{};
    float confidence = 0.f;
    uint32_t label = 0;
    std::string labelName;

    Keypoint() = default;
    explicit Keypoint(Point3f coordinates, float conf = 0.f, uint32_t label = 0, std::string name = {})
        : coordinates(coordinates), confidence(conf), label(label), labelName(std::move(name)) {
        if(confidence < 0.f) {
            throw std::invalid_argument("Confidence must be non-negative.");
        }
    }

    Keypoint(Point2f coordinates, float conf = 0.f, uint32_t label = 0, std::string name = {})
        : Keypoint(Point3f{coordinates.x, coordinates.y, 0.f}, conf, label, std::move(name)) {}

    Keypoint(float x, float y, float z, float confidence = 0.f, uint32_t label = 0, std::string name = {})
        : Keypoint(Point3f{x, y, z}, confidence, label, std::move(name)) {}

    DEPTHAI_SERIALIZE(dai::Keypoint, coordinates, confidence, label, labelName);
};

}  // namespace dai