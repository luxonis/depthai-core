#pragma once

#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/utility/Serialization.hpp"

namespace filtertest {

using Rotation = std::array<std::array<float, 3>, 3>;
inline constexpr Rotation IDENTITY = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

inline Rotation rotation(float degrees, bool yaw = false) {
    const float radians = degrees * 3.14159265358979323846f / 180;
    const float c = std::cos(radians), s = std::sin(radians);
    if(yaw) return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
    return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

inline dai::ImgTransformation transformation(std::size_t width = 512,
                                             std::size_t height = 512,
                                             const Rotation& r = IDENTITY,
                                             dai::CameraModel model = dai::CameraModel::Perspective,
                                             const std::vector<float>& distortion = {},
                                             float focal = 0) {
    const float w = static_cast<float>(width), h = static_cast<float>(height);
    const float f = focal == 0 ? w / 2 : focal;
    const Rotation k = {{{f, 0, w / 2}, {0, f, h / 2}, {0, 0, 1}}};
    const std::array<std::array<float, 4>, 4> e = {
        {{r[0][0], r[0][1], r[0][2], 0}, {r[1][0], r[1][1], r[1][2], 0}, {r[2][0], r[2][1], r[2][2], 0}, {0, 0, 0, 1}}};
    return {width, height, k, model, distortion, dai::Extrinsics(e, dai::CameraBoardSocket::CAM_A)};
}

inline dai::ImgTransformation scaled(dai::ImgTransformation t, float scale) {
    return t.addScale(scale, scale);
}

inline dai::ImgTransformation cropped(dai::ImgTransformation t, int x, int y, int width, int height) {
    return t.addCrop(x, y, width, height);
}

struct ExpectedKeypoint {
    float x, y;
    float confidence = 0.9f;
};

// All coordinates are pixels, including keypoints. Keep angle and size literal.
struct ExpectedDetection {
    float x, y, width, height;
    float confidence = 0.9f;
    std::uint32_t label = 0;
    float angle = 0;
    std::string name;
    std::vector<ExpectedKeypoint> keypoints;
};

inline dai::ImgDetection detection(const dai::ImgTransformation& t, const ExpectedDetection& d) {
    const auto size = t.getSize();
    const float w = static_cast<float>(size.first), h = static_cast<float>(size.second);
    dai::ImgDetection result;
    result.label = d.label;
    result.labelName = d.name;
    result.confidence = d.confidence;
    result.setBoundingBox({dai::Point2f(d.x / w, d.y / h, true), dai::Size2f(d.width / w, d.height / h, true), d.angle});
    if(!d.keypoints.empty()) {
        std::vector<dai::Keypoint> points;
        for(const auto& p : d.keypoints) points.emplace_back(dai::Point2f(p.x / w, p.y / h, true), p.confidence);
        result.setKeypoints(points);
    }
    return result;
}

inline std::shared_ptr<dai::ImgDetections> message(const dai::ImgTransformation& t = transformation(),
                                                   const std::vector<ExpectedDetection>& detections = {{100, 100, 64, 64, .9f, 1}}) {
    auto result = std::make_shared<dai::ImgDetections>();
    result->setTransformation(t);
    for(const auto& d : detections) result->detections.push_back(detection(t, d));
    result->setSequenceNum(42);
    result->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(10)));
    result->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::seconds(20)));
    result->setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::seconds(30)));
    return result;
}

inline std::vector<std::uint8_t> maskBytes(const std::string& grid) {
    std::istringstream stream(grid);
    std::vector<std::uint8_t> bytes;
    std::string token;
    while(stream >> token) {
        const int value = token == "." ? 255 : std::stoi(token);
        REQUIRE(value >= 0);
        REQUIRE(value <= 255);
        bytes.push_back(static_cast<std::uint8_t>(value));
    }
    return bytes;
}

inline std::string maskGrid(const std::vector<std::uint8_t>& bytes, std::size_t width) {
    REQUIRE(width > 0);
    REQUIRE(bytes.size() % width == 0);
    std::ostringstream stream;
    for(std::size_t i = 0; i < bytes.size(); ++i) {
        if(i != 0) stream << (i % width == 0 ? '\n' : ' ');
        if(bytes[i] == 255)
            stream << '.';
        else
            stream << static_cast<unsigned>(bytes[i]);
    }
    return stream.str();
}

inline void requireDetection(
    const dai::ImgDetection& actual, float width, float height, const ExpectedDetection& expected, float tolerance = 1e-4f, float angleTolerance = 1e-4f) {
    REQUIRE(actual.label == expected.label);
    REQUIRE(actual.labelName == expected.name);
    REQUIRE_THAT(actual.confidence, Catch::Matchers::WithinAbs(expected.confidence, 1e-6));
    const auto box = actual.getBoundingBox().denormalize(static_cast<unsigned>(width), static_cast<unsigned>(height));
    REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(expected.x, tolerance));
    REQUIRE_THAT(box.center.y, Catch::Matchers::WithinAbs(expected.y, tolerance));
    REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(expected.width, tolerance));
    REQUIRE_THAT(box.size.height, Catch::Matchers::WithinAbs(expected.height, tolerance));
    REQUIRE_THAT(box.angle, Catch::Matchers::WithinAbs(expected.angle, angleTolerance));
    const auto points = actual.getKeypoints();
    REQUIRE(points.size() == expected.keypoints.size());
    for(std::size_t i = 0; i < points.size(); ++i) {
        REQUIRE_THAT(points[i].confidence, Catch::Matchers::WithinAbs(expected.keypoints[i].confidence, 1e-6));
        if(expected.keypoints[i].confidence == 0) continue;
        REQUIRE_THAT(points[i].imageCoordinates.x * width, Catch::Matchers::WithinAbs(expected.keypoints[i].x, tolerance));
        REQUIRE_THAT(points[i].imageCoordinates.y * height, Catch::Matchers::WithinAbs(expected.keypoints[i].y, tolerance));
    }
}

inline void requireOutput(const dai::ImgDetections& output,
                          const dai::ImgTransformation& t,
                          const std::vector<ExpectedDetection>& expected,
                          float tolerance = 1e-4f,
                          float angleTolerance = 1e-4f) {
    REQUIRE(output.getTransformation().has_value());
    REQUIRE(output.getTransformation()->isEqualTransformation(t));
    REQUIRE(output.detections.size() == expected.size());
    const auto size = t.getSize();
    for(std::size_t i = 0; i < expected.size(); ++i) {
        CAPTURE(i);
        requireDetection(output.detections[i], static_cast<float>(size.first), static_cast<float>(size.second), expected[i], tolerance, angleTolerance);
    }
}

inline void requireMask(const dai::ImgDetections& output, std::size_t width, std::size_t height, const std::string& grid) {
    const auto bytes = maskBytes(grid);
    REQUIRE(bytes.size() == width * height);
    REQUIRE(output.getMaskData().has_value());
    REQUIRE(output.getSegmentationMaskWidth() == width);
    REQUIRE(output.getSegmentationMaskHeight() == height);
    REQUIRE(*output.getMaskData() == bytes);
}

struct MessageSnapshot {
    std::vector<std::uint8_t> metadata;
    std::optional<std::vector<std::uint8_t>> mask;

    explicit MessageSnapshot(const dai::ImgDetections& msg) : metadata(dai::utility::serialize(msg)), mask(msg.getMaskData()) {}

    void requireEqual(const dai::ImgDetections& msg) const {
        REQUIRE(dai::utility::serialize(msg) == metadata);
        REQUIRE(msg.getMaskData() == mask);
    }
};

inline void requireMetadata(const dai::ImgDetections& output, const dai::ImgDetections& input) {
    REQUIRE(output.getSequenceNum() == input.getSequenceNum());
    REQUIRE(output.getTimestamp() == input.getTimestamp());
    REQUIRE(output.getTimestampDevice() == input.getTimestampDevice());
    REQUIRE(output.getTimestampSystem() == input.getTimestampSystem());
}

}  // namespace filtertest
