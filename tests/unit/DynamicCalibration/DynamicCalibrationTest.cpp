#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "depthai/pipeline/node/DynamicCalibration.hpp"

class MockDynamicCalibration : public dai::node::DynamicCalibration {
   public:
    Properties& getPropertiesPublic() {
        return getProperties();
    }

    MockDynamicCalibration(std::unique_ptr<dcl::DynamicCalibration> dc) : DynamicCalibration(std::move(dc)) {}
    MockDynamicCalibration() : DynamicCalibration(std::make_unique<dcl::DynamicCalibration>()) {}
};

// Test case for the add function
TEST(DynamicCalibration, SetAndGetRunOnHost) {
    MockDynamicCalibration dynCalib;
    dynCalib.setRunOnHost(true);
    EXPECT_TRUE(dynCalib.runOnHost());

    dynCalib.setRunOnHost(false);
    EXPECT_FALSE(dynCalib.runOnHost());
}

TEST(DynamicCalibration, SetPerformanceMode) {
    MockDynamicCalibration dynCalib;
    dynCalib.setPerformanceMode(dcl::PerformanceMode::SKIP_CHECKS);
    EXPECT_EQ(dynCalib.getPropertiesPublic().performanceMode, dcl::PerformanceMode::SKIP_CHECKS);
}

TEST(DynamicCalibration, SetContinuousMode) {
    MockDynamicCalibration dynCalib;
    dynCalib.setContinuousMode();
    EXPECT_EQ(dynCalib.getPropertiesPublic().recalibrationMode, dai::DynamicCalibrationProperties::RecalibrationMode::CONTINUOUS);
}

TEST(DynamicCalibration, SetTimeFrequency) {
    MockDynamicCalibration dynCalib;
    dynCalib.setTimeFrequency(42);
    EXPECT_EQ(dynCalib.getPropertiesPublic().loadImageFrequency, 42);
}

class MockDclDynamicCalibration : public dcl::DynamicCalibration {
   public:
    MOCK_METHOD((dcl::Result<dcl::CalibrationQuality>),
                checkCalibration,
                (const std::shared_ptr<const dcl::Device>, const dcl::socket_t, const dcl::socket_t, const dcl::PerformanceMode),
                (const, override));

    MOCK_METHOD((dcl::Result<std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>>>),
                findNewCalibration,
                (const std::shared_ptr<const dcl::Device>, const dcl::socket_t, const dcl::socket_t, const dcl::PerformanceMode),
                (const, override));
};

TEST(DynamicCalibration, RunQualityCheckFailure) {
    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    MockDynamicCalibration dynCalib(std::move(mockDcl));

    // Force performance mode
    bool force = true;

    // Mock Result with failure
    dcl::Result<dcl::CalibrationQuality> failedResult = dcl::Result<dcl::CalibrationQuality>::error(1);

    // clang-format off
    EXPECT_CALL(*mockDclRaw, checkCalibration(testing::_, testing::_, testing::_, dcl::PerformanceMode::SKIP_CHECKS))
        .WillOnce(testing::Return(failedResult));
    // clang-format on

    auto err = dynCalib.runQualityCheck(force);

    EXPECT_EQ(err, dai::node::DynamicCalibration::ErrorCode::QUALITY_CHECK_FAILED);
}

TEST(DynamicCalibration, RunQualityCheckSuccess) {
    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    MockDynamicCalibration dynCalib(std::move(mockDcl));

    // Use properties performance mode
    bool force = false;
    dynCalib.setPerformanceMode(dcl::PerformanceMode::OPTIMIZE_PERFORMANCE);

    dcl::CalibrationQuality quality;

    dcl::Result<dcl::CalibrationQuality> successResult = dcl::Result<dcl::CalibrationQuality>::ok(quality);

    // clang-format off
    EXPECT_CALL(*mockDclRaw, checkCalibration(testing::_, testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE))
        .WillOnce(testing::Return(successResult));
    // clang-format om

    auto err = dynCalib.runQualityCheck(force);
    EXPECT_EQ(err, dai::node::DynamicCalibration::ErrorCode::OK);
}

TEST(DynamicCalibration, RunCalibrationFailure) {
    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    MockDynamicCalibration dynCalib(std::move(mockDcl));

    bool force = true;

    // Mock a failed result
    auto failedResult = dcl::Result<std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>>>::error(-1);

    // clang-format off
    EXPECT_CALL(*mockDclRaw, findNewCalibration(testing::_, testing::_, testing::_, testing::_))
        .WillOnce(testing::Return(failedResult));
    // clang-format on

    auto err = dynCalib.runCalibration(force);
    EXPECT_EQ(err, dai::node::DynamicCalibration::ErrorCode::CALIBRATION_FAILED);
}

TEST(DynamicCalibration, RunCalibrationSuccess) {
    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    MockDynamicCalibration dynCalib(std::move(mockDcl));

    bool force = false;
    dynCalib.setPerformanceMode(dcl::PerformanceMode::OPTIMIZE_PERFORMANCE);

    const dcl::scalar_t rotation[3] = {0., 0., 0.};
    const dcl::scalar_t translation[3] = {0., 0., 0.};
    const dcl::scalar_t cameraMatrix[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
    const dcl::scalar_t distortion[14] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

    auto calibHandleA = std::make_shared<dcl::CameraCalibrationHandle>(rotation, translation, cameraMatrix, distortion);
    auto calibHandleB = std::make_shared<dcl::CameraCalibrationHandle>(rotation, translation, cameraMatrix, distortion);

    // clang-format off
    std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> resultPair = {
      calibHandleA, calibHandleB
    };
    // clang-format on
    auto successResult = dcl::Result<std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>>>::ok(resultPair);

    // clang-format off
    EXPECT_CALL(*mockDclRaw, findNewCalibration(testing::_, testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE))
        .WillOnce(testing::Return(successResult));
    // clang-format on

    auto err = dynCalib.runCalibration(force);
    EXPECT_EQ(err, dai::node::DynamicCalibration::ErrorCode::OK);
}
