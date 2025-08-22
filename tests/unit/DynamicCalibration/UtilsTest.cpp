#include <gtest/gtest.h>

#include <DynamicCalibration.hpp>

#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

using namespace dai;
using namespace node;

namespace {

constexpr int width = 640;
constexpr int height = 480;

std::vector<std::vector<float>> testIntrinsics = {{600.0f, 0.0f, 320.0f}, {0.0f, 600.0f, 240.0f}, {0.0f, 0.0f, 1.0f}};

std::vector<float> testDistortion = {0.1f, -0.05f, 0.001f, 0.0005f};

std::vector<std::vector<float>> testRotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

std::vector<float> testTranslation = {10.0f, 5.0f, 2.5f};

TEST(DclUtilsTest, CreateDclCalibration_ProducesValidHandle) {
    auto handle = DclUtils::createDclCalibration(testIntrinsics, testDistortion, testRotation, testTranslation);
    ASSERT_NE(handle, nullptr);

    dcl::scalar_t camMat[9];
    dcl::scalar_t dist[14];
    dcl::scalar_t tvec[3];
    dcl::scalar_t rvec[3];

    handle->getCameraMatrix(camMat);
    handle->getDistortion(dist);
    handle->getTvec(tvec);
    handle->getRvec(rvec);

    EXPECT_FLOAT_EQ(camMat[0], 600.0f);
    EXPECT_FLOAT_EQ(dist[0], 0.1f);
    EXPECT_FLOAT_EQ(tvec[0], 0.1f);  // 10cm => 0.1m
}

TEST(DclUtilsTest, ConvertDaiCalibrationToDcl_ThenBackToDai_RoundTripConsistent) {
    CalibrationHandler calibHandler;

    calibHandler.setCameraIntrinsics(CameraBoardSocket::CAM_A, testIntrinsics, width, height);
    calibHandler.setCameraIntrinsics(CameraBoardSocket::CAM_B, testIntrinsics, width, height);
    calibHandler.setDistortionCoefficients(CameraBoardSocket::CAM_A, testDistortion);
    calibHandler.setDistortionCoefficients(CameraBoardSocket::CAM_B, testDistortion);
    calibHandler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, testRotation, testTranslation, testTranslation);

    auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(calibHandler, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, width, height);

    CalibrationHandler newHandler;
    // we need to set the Extricsics for a reason first
    std::vector<std::vector<float>> zeroRotation = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    newHandler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, zeroRotation, {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});

    DclUtils::convertDclCalibrationToDai(newHandler, calibA, calibB, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, width, height);

    auto backIntrinsics = newHandler.getCameraIntrinsics(CameraBoardSocket::CAM_B, width, height);
    auto backDist = newHandler.getDistortionCoefficients(CameraBoardSocket::CAM_B);
    auto backTrans = newHandler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false);

    // EXPECT_FLOAT_EQ(backIntrinsics[0][0], testIntrinsics[0][0]);
    // EXPECT_FLOAT_EQ(backDist[0], testDistortion[0]);
    EXPECT_NEAR(backTrans[0], testTranslation[0], 1e-2f);
}

TEST(DclUtilsTest, ConvertDclCalibrationToDai_SetsCorrectValues) {
    CalibrationHandler calibHandler;
    std::vector<std::vector<float>> zeroRotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    calibHandler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, zeroRotation, {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});

    std::vector<float> testDistortion = {0.1f, -0.05f, 0.001f, 0.0005f};

    std::vector<float> zeroTranslation = {0.0f, 0.0f, 0.0f};

    auto handleA = DclUtils::createDclCalibration(testIntrinsics, testDistortion, zeroRotation, zeroTranslation);
    auto handleB = DclUtils::createDclCalibration(testIntrinsics, testDistortion, testRotation, testTranslation);

    DclUtils::convertDclCalibrationToDai(calibHandler, handleA, handleB, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, width, height);

    auto outIntrinsics = calibHandler.getCameraIntrinsics(CameraBoardSocket::CAM_B, width, height);
    auto outDist = calibHandler.getDistortionCoefficients(CameraBoardSocket::CAM_B);
    auto outTrans = calibHandler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false);

    // EXPECT_FLOAT_EQ(outIntrinsics[0][0], testIntrinsics[0][0]);
    // EXPECT_FLOAT_EQ(outDist[0], testDistortion[0]);
    EXPECT_FLOAT_EQ(outTrans[0], testTranslation[0]);
}

TEST(DclUtilsTest, CvMatToImageData_Valid8UC1_ConversionOK) {
    cv::Mat mat = cv::Mat::zeros(10, 20, CV_8UC1);
    auto imageData = DclUtils::cvMatToImageData(mat);

    EXPECT_EQ(imageData.width, 20u);
    EXPECT_EQ(imageData.height, 10u);
    EXPECT_EQ(imageData.format, dcl::DCL_8UC1);
    EXPECT_EQ(imageData.data.size(), 200u);
}

TEST(DclUtilsTest, CvMatToImageData_Valid8UC3_ConversionOK) {
    cv::Mat mat = cv::Mat::zeros(5, 5, CV_8UC3);
    auto imageData = DclUtils::cvMatToImageData(mat);

    EXPECT_EQ(imageData.width, 5u);
    EXPECT_EQ(imageData.height, 5u);
    EXPECT_EQ(imageData.format, dcl::DCL_8UC3);
    EXPECT_EQ(imageData.data.size(), 5 * 5 * 3);
}

TEST(DclUtilsTest, CvMatToImageData_ThrowsOnEmptyMat) {
    cv::Mat empty;
    EXPECT_THROW(DclUtils::cvMatToImageData(empty), std::runtime_error);
}

TEST(DclUtilsTest, CvMatToImageData_ThrowsOnUnsupportedType) {
    cv::Mat floatMat(10, 10, CV_32FC1);
    EXPECT_THROW(DclUtils::cvMatToImageData(floatMat), std::runtime_error);
}

}  // namespace
