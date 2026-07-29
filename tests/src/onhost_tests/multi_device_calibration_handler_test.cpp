#include <catch2/catch_all.hpp>
#include <cmath>
#include <filesystem>

#include "depthai/depthai.hpp"

namespace {

dai::Extrinsics makeExtrinsics(const dai::CoordinateFrame& to, float tx, float ty = 0.0f, float tz = 0.0f) {
    dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, dai::Point3f(tx, ty, tz), to.socket);
    extrinsics.setReferenceFrame(to);
    return extrinsics;
}

dai::RigEdge makeEdge(const dai::CoordinateFrame& from, const dai::CoordinateFrame& to, float tx) {
    dai::RigEdge edge;
    edge.from = from;
    edge.to = to;
    edge.transform = makeExtrinsics(to, tx);
    edge.source = "test";
    return edge;
}

const dai::CoordinateFrame frameA{"deviceA", dai::CameraBoardSocket::CAM_A};
const dai::CoordinateFrame frameB{"deviceB", dai::CameraBoardSocket::CAM_A};
const dai::CoordinateFrame frameC{"deviceC", dai::CameraBoardSocket::CAM_A};
const dai::CoordinateFrame frameD{"deviceD", dai::CameraBoardSocket::CAM_A};

}  // namespace

TEST_CASE("CoordinateFrame distinguishes the same socket on different devices") {
    REQUIRE(frameA != frameB);
    REQUIRE(frameA.isQualified());
    REQUIRE_FALSE(dai::CoordinateFrame(dai::CameraBoardSocket::CAM_A).isQualified());
    REQUIRE(dai::CoordinateFrame().isUnknown());
    // An unknown device id must not compare equal to a known one
    REQUIRE(dai::CoordinateFrame(dai::CameraBoardSocket::CAM_A) != frameA);
}

TEST_CASE("MultiDeviceCalibrationHandler composes transformations along the rig") {
    dai::MultiDeviceCalibrationData data;
    data.edges = {makeEdge(frameA, frameB, 10.0f), makeEdge(frameB, frameC, 5.0f)};
    dai::MultiDeviceCalibrationHandler handler(data);

    REQUIRE(handler.canTransform(frameA, frameC));
    const auto transform = handler.getTransform(frameA, frameC);
    // T_C<-A == T_C<-B * T_B<-A, translations add up
    REQUIRE(transform[0][3] == Catch::Approx(15.0f));

    const auto inverse = handler.getTransform(frameC, frameA);
    REQUIRE(inverse[0][3] == Catch::Approx(-15.0f));

    // Unit conversion of the translation
    const auto inMillimeters = handler.getTransform(frameA, frameC, dai::LengthUnit::MILLIMETER);
    REQUIRE(inMillimeters[0][3] == Catch::Approx(150.0f));

    // Identity for the same frame, even if it is not part of the rig
    const auto identity = handler.getTransform(frameD, frameD);
    REQUIRE(identity[0][3] == Catch::Approx(0.0f));
    REQUIRE(identity[1][1] == Catch::Approx(1.0f));
}

TEST_CASE("MultiDeviceCalibrationHandler keeps disconnected components apart") {
    dai::MultiDeviceCalibrationData data;
    data.edges = {makeEdge(frameA, frameB, 10.0f), makeEdge(frameC, frameD, 7.0f)};
    dai::MultiDeviceCalibrationHandler handler(data);

    REQUIRE(handler.getComponents().size() == 2);
    REQUIRE_FALSE(handler.canTransform(frameA, frameC));
    REQUIRE_THROWS_AS(handler.getTransform(frameA, frameC), std::runtime_error);

    // Roots are real camera frames, deterministically the lowest-ordered frame of the component
    REQUIRE(handler.getComponentRoot(frameA) == frameA);
    REQUIRE(handler.getComponentRoot(frameB) == frameA);
    REQUIRE(handler.getComponentRoot(frameD) == frameC);
}

TEST_CASE("MultiDeviceCalibrationHandler rejects graphs that are not a forest") {
    dai::MultiDeviceCalibrationData cyclic;
    cyclic.edges = {makeEdge(frameA, frameB, 1.0f), makeEdge(frameB, frameC, 1.0f), makeEdge(frameC, frameA, 1.0f)};
    REQUIRE_THROWS_AS(dai::MultiDeviceCalibrationHandler(cyclic), std::runtime_error);

    dai::MultiDeviceCalibrationData duplicate;
    duplicate.edges = {makeEdge(frameA, frameB, 1.0f), makeEdge(frameB, frameA, 1.0f)};
    REQUIRE_THROWS_AS(dai::MultiDeviceCalibrationHandler(duplicate), std::runtime_error);

    dai::MultiDeviceCalibrationData selfEdge;
    selfEdge.edges = {makeEdge(frameA, frameA, 1.0f)};
    REQUIRE_THROWS_AS(dai::MultiDeviceCalibrationHandler(selfEdge), std::runtime_error);
}

TEST_CASE("MultiDeviceCalibrationHandler edits bump the revision and validate") {
    dai::MultiDeviceCalibrationData data;
    data.edges = {makeEdge(frameA, frameB, 10.0f)};
    dai::MultiDeviceCalibrationHandler handler(data);
    const auto revision = handler.getRevision();

    handler.setEdge(makeEdge(frameB, frameC, 5.0f));
    REQUIRE(handler.getRevision() > revision);
    REQUIRE(handler.getTransform(frameA, frameC)[0][3] == Catch::Approx(15.0f));

    // Replacing an existing edge, in the opposite direction
    handler.setEdge(makeEdge(frameB, frameA, 4.0f));
    REQUIRE(handler.getData().edges.size() == 2);
    REQUIRE(handler.getTransform(frameA, frameB)[0][3] == Catch::Approx(-4.0f));

    // An edge closing a cycle is rejected and leaves the handler untouched
    REQUIRE_THROWS_AS(handler.setEdge(makeEdge(frameA, frameC, 1.0f)), std::runtime_error);
    REQUIRE(handler.getData().edges.size() == 2);

    REQUIRE(handler.removeEdge(frameC, frameB));
    REQUIRE_FALSE(handler.canTransform(frameA, frameC));
    REQUIRE_FALSE(handler.removeEdge(frameC, frameB));
}

TEST_CASE("MultiDeviceCalibrationHandler resolves aliases") {
    const dai::CoordinateFrame front{"front", dai::CameraBoardSocket::CAM_A};
    const dai::CoordinateFrame back{"back", dai::CameraBoardSocket::CAM_A};

    dai::MultiDeviceCalibrationData data;
    data.edges = {makeEdge(front, back, 10.0f)};
    data.aliases = {{"front", "deviceA"}};
    dai::MultiDeviceCalibrationHandler handler(data);

    handler.resolveAliases({{"back", "deviceB"}});
    REQUIRE(handler.getDeviceIds() == std::vector<std::string>{"deviceA", "deviceB"});
    REQUIRE(handler.canTransform(frameA, frameB));
}

TEST_CASE("MultiDeviceCalibrationHandler round-trips through JSON") {
    dai::MultiDeviceCalibrationData data;
    data.edges = {makeEdge(frameA, frameB, 10.0f)};
    const dai::MultiDeviceCalibrationHandler handler(data);

    const auto path = std::filesystem::temp_directory_path() / "multi_device_calibration_test.json";
    REQUIRE(handler.toJsonFile(path));

    const dai::MultiDeviceCalibrationHandler loaded(path);
    REQUIRE(loaded.getData().edges.size() == 1);
    REQUIRE(loaded.getData().edges.front().from == frameA);
    REQUIRE(loaded.getData().edges.front().to == frameB);
    REQUIRE(loaded.getTransform(frameA, frameB)[0][3] == Catch::Approx(10.0f));

    const auto fromJson = dai::MultiDeviceCalibrationHandler::fromJson(handler.toJson());
    REQUIRE(fromJson.getTransform(frameA, frameB)[0][3] == Catch::Approx(10.0f));
    std::filesystem::remove(path);
}

TEST_CASE("MultiDeviceCalibrationHandler re-expresses extrinsics in another frame") {
    dai::MultiDeviceCalibrationData data;
    data.edges = {makeEdge(frameA, frameB, 10.0f)};
    const dai::MultiDeviceCalibrationHandler handler(data);

    // A camera whose pose is known w.r.t. frameA, re-expressed w.r.t. frameB
    auto extrinsics = makeExtrinsics(frameA, 2.0f);
    handler.reexpress(extrinsics, frameB);
    REQUIRE(extrinsics.getReferenceFrame() == frameB);
    REQUIRE(extrinsics.getTransformationMatrix()[0][3] == Catch::Approx(12.0f));

    // Already in the target frame - no change
    handler.reexpress(extrinsics, frameB);
    REQUIRE(extrinsics.getTransformationMatrix()[0][3] == Catch::Approx(12.0f));

    // Unknown reference frame and unreachable target frames throw
    dai::Extrinsics unknown;
    REQUIRE_THROWS_AS(handler.reexpress(unknown, frameB), std::runtime_error);
    auto unreachable = makeExtrinsics(frameA, 2.0f);
    REQUIRE_THROWS_AS(handler.reexpress(unreachable, frameC), std::runtime_error);
}

TEST_CASE("ImgTransformation equality accounts for the reference frame") {
    dai::ImgTransformation transformation(640, 400);
    transformation.setExtrinsics(makeExtrinsics(frameA, 1.0f));

    auto other = transformation;
    REQUIRE(transformation.isEqualTransformation(other));
    REQUIRE(transformation.isAlignedTo(other));

    other.setReferenceFrame(frameB);
    REQUIRE(other.getReferenceFrame() == frameB);
    REQUIRE_FALSE(transformation.isEqualTransformation(other));
    REQUIRE_FALSE(transformation.isAlignedTo(other));
}
