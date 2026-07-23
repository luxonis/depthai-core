#include <catch2/catch_all.hpp>
#include <cstdio>
#include <depthai/depthai.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <vector>

using namespace dai;

namespace {

std::vector<std::vector<float>> identityRotation() {
    return {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
}

CameraInfo makeCameraInfo(CameraBoardSocket toCameraSocket, const Point3f& translation) {
    CameraInfo info;
    info.width = 1920;
    info.height = 1200;
    info.lensPosition = 0;
    info.intrinsicMatrix = identityRotation();
    info.distortionCoeff = std::vector<float>(14, 0.0f);
    info.specHfovDeg = 70.0f;
    info.extrinsics = Extrinsics(identityRotation(), translation, toCameraSocket, LengthUnit::CENTIMETER);
    info.extrinsics.specTranslation = translation;
    return info;
}

CalibrationHandler makeCalibration(const Point3f& camBToCamA = Point3f(10.0f, 0.0f, 0.0f),
                                   const Point3f& camCToCamA = Point3f(0.0f, 20.0f, 0.0f),
                                   const std::string& productName = "",
                                   const Point3f& housingToCamA = Point3f(0.0f, 0.0f, 30.0f)) {
    EepromData data;
    data.version = 7;
    data.productName = productName;
    data.cameraData[CameraBoardSocket::CAM_A] = makeCameraInfo(CameraBoardSocket::AUTO, Point3f());
    data.cameraData[CameraBoardSocket::CAM_B] = makeCameraInfo(CameraBoardSocket::CAM_A, camBToCamA);
    data.cameraData[CameraBoardSocket::CAM_C] = makeCameraInfo(CameraBoardSocket::CAM_A, camCToCamA);
    data.housingExtrinsics = Extrinsics(identityRotation(), housingToCamA, CameraBoardSocket::CAM_A, LengthUnit::CENTIMETER);
    data.housingExtrinsics.specTranslation = housingToCamA;
    return CalibrationHandler(data, true);
}

RigidTransform makeRigTransform(float x = 0.0f, float y = 0.0f, float z = 0.0f, LengthUnit unit = LengthUnit::CENTIMETER) {
    RigidTransform transform;
    transform.translation = Point3f(x, y, z);
    transform.translationUnit = unit;
    return transform;
}

std::vector<std::vector<float>> inverseTransform(std::vector<std::vector<float>> transform) {
    matrix::invertSe3Matrix4x4InPlace(transform);
    return transform;
}

void requireMatricesNear(const std::vector<std::vector<float>>& actual, const std::vector<std::vector<float>>& expected, float epsilon = 1e-5f) {
    REQUIRE(actual.size() == expected.size());
    for(size_t row = 0; row < actual.size(); ++row) {
        REQUIRE(actual[row].size() == expected[row].size());
        for(size_t col = 0; col < actual[row].size(); ++col) {
            REQUIRE(actual[row][col] == Catch::Approx(expected[row][col]).margin(epsilon));
        }
    }
}

std::vector<std::vector<float>> expectedHousingToHousing(const CalibrationHandler& calibration,
                                                         HousingCoordinateSystem srcHousing,
                                                         HousingCoordinateSystem dstHousing,
                                                         bool useSpecTranslation) {
    const auto referenceCamera = calibration.getCameraWithLowestId();
    auto dstFromReference = calibration.getHousingCalibration(referenceCamera, dstHousing, useSpecTranslation, LengthUnit::CENTIMETER);
    auto srcFromReference = calibration.getHousingCalibration(referenceCamera, srcHousing, useSpecTranslation, LengthUnit::CENTIMETER);
    matrix::invertSe3Matrix4x4InPlace(srcFromReference);
    return matrix::matMul(dstFromReference, srcFromReference);
}

}  // namespace

TEST_CASE("MultiDeviceCalibrationHandler serialization round-trip", "[multi-device-calibration]") {
    MultiDeviceCalibrationData data;
    data.timestamp = 123456789ULL;
    data.devices = {
        {"MXID_A", makeCalibration().getEepromData(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform()},
        {"MXID_B",
         makeCalibration(Point3f(12.0f, 0.0f, 0.0f), Point3f(0.0f, 8.0f, 0.0f)).getEepromData(),
         MultiDeviceFrame::camera(CameraBoardSocket::CAM_A),
         makeRigTransform(100.0f, 0.0f, 0.0f)},
    };

    MultiDeviceCalibrationHandler handler(data, true);
    auto fromJson = MultiDeviceCalibrationHandler::fromJson(handler.toJson(), true);

    const auto tempPath = std::filesystem::temp_directory_path() / "multi_device_calibration_handler_roundtrip.json";
    REQUIRE(handler.toJsonFile(tempPath));
    MultiDeviceCalibrationHandler fromFile(tempPath, true);
    std::remove(tempPath.string().c_str());

    REQUIRE(fromJson.getData().timestamp == data.timestamp);
    REQUIRE(fromJson.getDeviceIds() == std::vector<std::string>({"MXID_A", "MXID_B"}));
    REQUIRE(fromJson.getDevice("MXID_A").anchorFrame == handler.getDevice("MXID_A").anchorFrame);
    REQUIRE(fromFile.getDevice("MXID_B").rigFromAnchor.translation.x == Catch::Approx(100.0f).margin(1e-6));
    REQUIRE(fromFile.getDevice("MXID_B").rigFromAnchor.translationUnit == LengthUnit::CENTIMETER);

    requireMatricesNear(fromJson.getCameraExtrinsics("MXID_A", CameraBoardSocket::CAM_B, "MXID_B", CameraBoardSocket::CAM_C, false, LengthUnit::CENTIMETER),
                        handler.getCameraExtrinsics("MXID_A", CameraBoardSocket::CAM_B, "MXID_B", CameraBoardSocket::CAM_C, false, LengthUnit::CENTIMETER));
}

TEST_CASE("MultiDeviceCalibrationHandler rejects duplicate MXIDs", "[multi-device-calibration]") {
    MultiDeviceCalibrationData data;
    data.devices = {
        {"MXID_DUP", makeCalibration().getEepromData(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform()},
        {"MXID_DUP", makeCalibration().getEepromData(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(10.0f, 0.0f, 0.0f)},
    };

    REQUIRE_THROWS_WITH(MultiDeviceCalibrationHandler(data, true), Catch::Matchers::ContainsSubstring("Duplicate MXID"));
}

TEST_CASE("MultiDeviceCalibrationHandler rejects invalid anchors", "[multi-device-calibration]") {
    MultiDeviceCalibrationData data;
    data.devices = {
        {"MXID_A", makeCalibration().getEepromData(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_D), makeRigTransform()},
    };

    REQUIRE_THROWS_WITH(MultiDeviceCalibrationHandler(data, true), Catch::Matchers::ContainsSubstring("Anchor frame"));
}

TEST_CASE("MultiDeviceCalibrationHandler composes anchor transforms with correct direction", "[multi-device-calibration]") {
    MultiDeviceCalibrationHandler handler;
    handler.setDevice("MXID_A", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform());
    handler.setDevice("MXID_B", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(100.0f, 0.0f, 0.0f));

    const auto actual = handler.getDeviceExtrinsics("MXID_A", "MXID_B", LengthUnit::CENTIMETER);
    const auto expected = std::vector<std::vector<float>>{
        {1.0f, 0.0f, 0.0f, -100.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f},
    };

    requireMatricesNear(actual, expected);
}

TEST_CASE("MultiDeviceCalibrationHandler matches same-device CalibrationHandler camera transforms", "[multi-device-calibration]") {
    const auto calibration = makeCalibration(Point3f(15.0f, 1.0f, 0.0f), Point3f(-2.0f, 7.0f, 0.0f));

    MultiDeviceCalibrationHandler handler;
    handler.setDevice("MXID_A", calibration, MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform());

    requireMatricesNear(handler.getCameraExtrinsics("MXID_A", CameraBoardSocket::CAM_B, "MXID_A", CameraBoardSocket::CAM_C),
                        calibration.getCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C));
}

TEST_CASE("MultiDeviceCalibrationHandler composes cross-device camera transforms", "[multi-device-calibration]") {
    const auto calibrationA = makeCalibration(Point3f(10.0f, 0.0f, 0.0f), Point3f(0.0f, 4.0f, 0.0f));
    const auto calibrationB = makeCalibration(Point3f(6.0f, 0.0f, 0.0f), Point3f(0.0f, 8.0f, 0.0f));

    MultiDeviceCalibrationHandler handler;
    handler.setDevice("MXID_A", calibrationA, MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(0.0f, 0.0f, 0.0f));
    handler.setDevice("MXID_B", calibrationB, MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(100.0f, 5.0f, 0.0f));

    const auto rigFromCamB = calibrationA.getCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_A, false, LengthUnit::CENTIMETER);
    auto rigFromCamC = calibrationB.getCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_A, false, LengthUnit::CENTIMETER);
    rigFromCamC = matrix::matMul(matrix::toVecMatrix4x4(matrix::createTransformationMatrix(
                                     std::array<std::array<float, 3>, 3>{
                                         std::array<float, 3>{1.0f, 0.0f, 0.0f},
                                         std::array<float, 3>{0.0f, 1.0f, 0.0f},
                                         std::array<float, 3>{0.0f, 0.0f, 1.0f},
                                     },
                                     Point3f(100.0f, 5.0f, 0.0f))),
                                 rigFromCamC);
    auto expected = inverseTransform(rigFromCamC);
    expected = matrix::matMul(expected, rigFromCamB);

    requireMatricesNear(handler.getCameraExtrinsics("MXID_A", CameraBoardSocket::CAM_B, "MXID_B", CameraBoardSocket::CAM_C), expected);
}

TEST_CASE("MultiDeviceCalibrationHandler resolves camera and housing combinations", "[multi-device-calibration]") {
    const auto housingCalibrationA = makeCalibration(Point3f(10.0f, 0.0f, 0.0f), Point3f(0.0f, 12.0f, 0.0f), "OAK-4-D-AF", Point3f(0.0f, 0.0f, 30.0f));
    const auto housingCalibrationB = makeCalibration(Point3f(7.0f, 0.0f, 0.0f), Point3f(0.0f, 9.0f, 0.0f), "OAK-4-D-AF", Point3f(0.0f, 0.0f, 30.0f));

    MultiDeviceCalibrationHandler handler;
    handler.setDevice("MXID_A", housingCalibrationA, MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform());
    handler.setDevice("MXID_B", housingCalibrationB, MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(80.0f, 0.0f, 0.0f));

    requireMatricesNear(
        handler.getFrameExtrinsics(
            "MXID_A", MultiDeviceFrame::camera(CameraBoardSocket::CAM_B), "MXID_A", MultiDeviceFrame::housingFrame(HousingCoordinateSystem::FRONT_CAM_A), true),
        housingCalibrationA.getHousingCalibration(CameraBoardSocket::CAM_B, HousingCoordinateSystem::FRONT_CAM_A, true, LengthUnit::CENTIMETER));

    requireMatricesNear(
        handler.getFrameExtrinsics(
            "MXID_A", MultiDeviceFrame::housingFrame(HousingCoordinateSystem::FRONT_CAM_A), "MXID_A", MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), true),
        inverseTransform(
            housingCalibrationA.getHousingCalibration(CameraBoardSocket::CAM_A, HousingCoordinateSystem::FRONT_CAM_A, true, LengthUnit::CENTIMETER)));

    requireMatricesNear(handler.getFrameExtrinsics("MXID_A",
                                                   MultiDeviceFrame::housingFrame(HousingCoordinateSystem::FRONT_CAM_A),
                                                   "MXID_A",
                                                   MultiDeviceFrame::housingFrame(HousingCoordinateSystem::CAM_A),
                                                   true),
                        expectedHousingToHousing(housingCalibrationA, HousingCoordinateSystem::FRONT_CAM_A, HousingCoordinateSystem::CAM_A, true));

    auto rigFromCamC =
        matrix::matMul(matrix::toVecMatrix4x4(matrix::createTransformationMatrix(
                           std::array<std::array<float, 3>, 3>{
                               std::array<float, 3>{1.0f, 0.0f, 0.0f},
                               std::array<float, 3>{0.0f, 1.0f, 0.0f},
                               std::array<float, 3>{0.0f, 0.0f, 1.0f},
                           },
                           Point3f(80.0f, 0.0f, 0.0f))),
                       housingCalibrationB.getCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_A, false, LengthUnit::CENTIMETER));
    auto rigFromHousing = inverseTransform(
        housingCalibrationA.getHousingCalibration(CameraBoardSocket::CAM_A, HousingCoordinateSystem::FRONT_CAM_A, true, LengthUnit::CENTIMETER));
    auto expectedCrossDevice = inverseTransform(rigFromHousing);
    expectedCrossDevice = matrix::matMul(expectedCrossDevice, rigFromCamC);

    requireMatricesNear(
        handler.getFrameExtrinsics(
            "MXID_B", MultiDeviceFrame::camera(CameraBoardSocket::CAM_C), "MXID_A", MultiDeviceFrame::housingFrame(HousingCoordinateSystem::FRONT_CAM_A), true),
        expectedCrossDevice);
}

TEST_CASE("MultiDeviceCalibrationHandler converts translation units", "[multi-device-calibration]") {
    MultiDeviceCalibrationHandler handler;
    handler.setDevice("MXID_A", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform());
    handler.setDevice("MXID_B", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(100.0f, 0.0f, 0.0f));

    const auto centimeters = handler.getDeviceExtrinsics("MXID_A", "MXID_B", LengthUnit::CENTIMETER);
    const auto millimeters = handler.getDeviceExtrinsics("MXID_A", "MXID_B", LengthUnit::MILLIMETER);
    const auto meters = handler.getDeviceExtrinsics("MXID_A", "MXID_B", LengthUnit::METER);

    REQUIRE(centimeters[0][3] == Catch::Approx(-100.0f).margin(1e-6));
    REQUIRE(millimeters[0][3] == Catch::Approx(-1000.0f).margin(1e-6));
    REQUIRE(meters[0][3] == Catch::Approx(-1.0f).margin(1e-6));
}

TEST_CASE("MultiDeviceCalibrationHandler removal and replacement keeps unrelated devices valid", "[multi-device-calibration]") {
    MultiDeviceCalibrationHandler handler;
    handler.setDevice("MXID_A", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform());
    handler.setDevice("MXID_B", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(100.0f, 0.0f, 0.0f));
    handler.setDevice("MXID_C", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(200.0f, 0.0f, 0.0f));

    handler.removeDevice("MXID_B");

    REQUIRE_THROWS_WITH(handler.getDeviceExtrinsics("MXID_A", "MXID_B"), Catch::Matchers::ContainsSubstring("Unknown device MXID"));
    requireMatricesNear(handler.getDeviceExtrinsics("MXID_A", "MXID_C"),
                        std::vector<std::vector<float>>{
                            {1.0f, 0.0f, 0.0f, -200.0f},
                            {0.0f, 1.0f, 0.0f, 0.0f},
                            {0.0f, 0.0f, 1.0f, 0.0f},
                            {0.0f, 0.0f, 0.0f, 1.0f},
                        });

    handler.setDevice("MXID_D", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform(300.0f, 0.0f, 0.0f));
    requireMatricesNear(handler.getDeviceExtrinsics("MXID_A", "MXID_D"),
                        std::vector<std::vector<float>>{
                            {1.0f, 0.0f, 0.0f, -300.0f},
                            {0.0f, 1.0f, 0.0f, 0.0f},
                            {0.0f, 0.0f, 1.0f, 0.0f},
                            {0.0f, 0.0f, 0.0f, 1.0f},
                        });
}

TEST_CASE("MultiDeviceCalibrationHandler returns identity for identical frame queries", "[multi-device-calibration]") {
    MultiDeviceCalibrationHandler handler;
    handler.setDevice("MXID_A", makeCalibration(), MultiDeviceFrame::camera(CameraBoardSocket::CAM_A), makeRigTransform());

    requireMatricesNear(
        handler.getFrameExtrinsics("MXID_A", MultiDeviceFrame::camera(CameraBoardSocket::CAM_B), "MXID_A", MultiDeviceFrame::camera(CameraBoardSocket::CAM_B)),
        std::vector<std::vector<float>>{
            {1.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f},
        });
}
