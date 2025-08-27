#include <DynamicCalibration.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

using namespace dai;
using namespace node;

namespace {

constexpr int width = 640;
constexpr int height = 480;

std::vector<std::vector<float>> testIntrinsics = {
    {600.0f, 0.0f, 320.0f},
    {0.0f, 600.0f, 240.0f},
    {0.0f, 0.0f, 1.0f},
};

std::vector<float> testDistortion = {0.1f, -0.05f, 0.001f, 0.0005f};

std::vector<std::vector<float>> testRotation = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

std::vector<float> testTranslation = {10.0f, 5.0f, 2.5f};

TEST_CASE("DclUtils::createDclCalibration produces valid handle", "[DclUtils]") {
    auto handle = DclUtils::createDclCalibration(testIntrinsics, testDistortion, testRotation, testTranslation);
    REQUIRE(handle != nullptr);

    dcl::scalar_t camMat[9];
    dcl::scalar_t dist[14];
    dcl::scalar_t tvec[3];
    dcl::scalar_t rvec[3];

    handle->getCameraMatrix(camMat);
    handle->getDistortion(dist);
    handle->getTvec(tvec);
    handle->getRvec(rvec);

    REQUIRE(camMat[0] == Catch::Approx(600.0f));
    REQUIRE(dist[0] == Catch::Approx(0.1f));
    REQUIRE(tvec[0] == Catch::Approx(0.1f));  // 10cm => 0.1m
}

TEST_CASE("Convert Dai -> DCL -> Dai round trip keeps translation", "[DclUtils]") {
    CalibrationHandler calibHandler;

    calibHandler.setCameraIntrinsics(CameraBoardSocket::CAM_A, testIntrinsics, width, height);
    calibHandler.setCameraIntrinsics(CameraBoardSocket::CAM_B, testIntrinsics, width, height);
    calibHandler.setDistortionCoefficients(CameraBoardSocket::CAM_A, testDistortion);
    calibHandler.setDistortionCoefficients(CameraBoardSocket::CAM_B, testDistortion);
    calibHandler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, testRotation, testTranslation, testTranslation);

    auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(calibHandler, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, width, height);

    CalibrationHandler newHandler;
    // Need valid baseline first
    std::vector<std::vector<float>> zeroRotation = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
    };
    newHandler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, zeroRotation, {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});

    DclUtils::convertDclCalibrationToDai(newHandler, calibA, calibB, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, width, height);

    auto backIntrinsics = newHandler.getCameraIntrinsics(CameraBoardSocket::CAM_B, width, height);
    auto backDist = newHandler.getDistortionCoefficients(CameraBoardSocket::CAM_B);
    auto backTrans = newHandler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false);

    // REQUIRE(backIntrinsics[0][0] == Catch::Approx(testIntrinsics[0][0]));
    // REQUIRE(backDist[0] == Catch::Approx(testDistortion[0]));
    REQUIRE(backTrans[0] == Catch::Approx(testTranslation[0]).margin(1e-2f));
}

TEST_CASE("Convert DCL -> Dai sets expected values", "[DclUtils]") {
    CalibrationHandler calibHandler;

    std::vector<std::vector<float>> zeroRotation = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    calibHandler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, zeroRotation, {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});

    std::vector<float> localDistortion = {0.1f, -0.05f, 0.001f, 0.0005f};
    std::vector<float> zeroTranslation = {0.0f, 0.0f, 0.0f};

    auto handleA = DclUtils::createDclCalibration(testIntrinsics, localDistortion, zeroRotation, zeroTranslation);
    auto handleB = DclUtils::createDclCalibration(testIntrinsics, localDistortion, testRotation, testTranslation);

    DclUtils::convertDclCalibrationToDai(calibHandler, handleA, handleB, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, width, height);

    auto outIntrinsics = calibHandler.getCameraIntrinsics(CameraBoardSocket::CAM_B, width, height);
    auto outDist = calibHandler.getDistortionCoefficients(CameraBoardSocket::CAM_B);
    auto outTrans = calibHandler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false);

    REQUIRE(outIntrinsics[0][0] == Catch::Approx(testIntrinsics[0][0]));
    REQUIRE(outDist[0] == Catch::Approx(localDistortion[0]));
    REQUIRE(outTrans[0] == Catch::Approx(testTranslation[0]));
}

TEST_CASE("cvMatToImageData: valid 8UC1 converts OK", "[DclUtils]") {
    cv::Mat mat = cv::Mat::zeros(10, 20, CV_8UC1);
    auto imageData = DclUtils::cvMatToImageData(mat);

    REQUIRE(imageData.width == 20u);
    REQUIRE(imageData.height == 10u);
    REQUIRE(imageData.format == dcl::DCL_8UC1);
    REQUIRE(imageData.data.size() == 200u);
}

TEST_CASE("cvMatToImageData: valid 8UC3 converts OK", "[DclUtils]") {
    cv::Mat mat = cv::Mat::zeros(5, 5, CV_8UC3);
    auto imageData = DclUtils::cvMatToImageData(mat);

    REQUIRE(imageData.width == 5u);
    REQUIRE(imageData.height == 5u);
    REQUIRE(imageData.format == dcl::DCL_8UC3);
    REQUIRE(imageData.data.size() == 5u * 5u * 3u);
}

TEST_CASE("cvMatToImageData: throws on empty Mat", "[DclUtils]") {
    cv::Mat empty;
    REQUIRE_THROWS_AS(DclUtils::cvMatToImageData(empty), std::runtime_error);
}

TEST_CASE("cvMatToImageData: throws on unsupported type", "[DclUtils]") {
    cv::Mat floatMat(10, 10, CV_32FC1);
    REQUIRE_THROWS_AS(DclUtils::cvMatToImageData(floatMat), std::runtime_error);
}

}  // namespace
