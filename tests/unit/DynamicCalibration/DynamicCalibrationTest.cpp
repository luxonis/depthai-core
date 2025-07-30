#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <DynamicCalibration.hpp>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

class MockDynamicCalibration : public dai::node::DynamicCalibration {
   public:
    using dai::node::DynamicCalibration::DynamicCalibration;
    ~MockDynamicCalibration() override = default;

    Properties& getPropertiesPublic() {
        return getProperties();
    }

    constexpr static const char* NAME = "MockDynamicCalibration";

    void run() override {
        std::cout << "DynamicCalibration::run(), type = " << typeid(*this).name() << "\n";
    }
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
    dai::DynamicCalibrationConfig config;
    config.performanceMode = dcl::PerformanceMode::SKIP_CHECKS;
    dynCalib.setInitialConfig(config);

    EXPECT_EQ(dynCalib.getPropertiesPublic().initialConfig.performanceMode, dcl::PerformanceMode::SKIP_CHECKS);
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

    MOCK_METHOD((dcl::Result<void>),
                loadStereoImagePair,
                (const dcl::ImageData& imageA,
                 const dcl::ImageData& imageB,
                 const dcl::mxid_t& deviceName,
                 const dcl::socket_t socketA,
                 const dcl::socket_t socketB,
                 const dcl::timestamp_t timestamp),
                (override));

    MOCK_METHOD((dcl::Result<dcl::CoverageData>),
                computeCoverage,
                (const std::shared_ptr<const dcl::CameraSensorHandle> sensorHandleA,
                 const std::shared_ptr<const dcl::CameraSensorHandle> sensorHandleB,
                 const dcl::PerformanceMode mode),
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
    dai::DynamicCalibrationConfig config;
    config.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib.setInitialConfig(config);

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

    dai::CalibrationHandler calibHandler;

    auto err = dynCalib.runCalibration(calibHandler, force);
    EXPECT_EQ(err, dai::node::DynamicCalibration::ErrorCode::CALIBRATION_FAILED);
}

TEST(DynamicCalibration, RunCalibrationSuccess) {
    dai::Pipeline pipeline(false);
    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>(std::move(mockDcl));

    auto returnedData = dynCalib->calibrationOutput.createOutputQueue();  // tryGet<dai::DatatypeEnum::CoverageData>();

    bool force = false;
    dai::DynamicCalibrationConfig config;
    config.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib->setInitialConfig(config);

    dynCalib->setWidth(1280);
    dynCalib->setHeight(800);

    const dcl::scalar_t zeroRotation[3] = {0., 0., 0.};
    const dcl::scalar_t zeroTranslation[3] = {0., 0., 0.};
    const dcl::scalar_t rotation[3] = {1., 2., 3.};
    const dcl::scalar_t translation[3] = {4., 5., 6.};
    const dcl::scalar_t cameraMatrix[9] = {1., 0., 1., 0., 1., 1., 0., 0., 1.};
    const dcl::scalar_t distortion[14] = {4., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

    auto calibHandleA = std::make_shared<dcl::CameraCalibrationHandle>(zeroRotation, zeroTranslation, cameraMatrix, distortion);
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

    dai::CalibrationHandler calibHandler;
    std::vector<std::vector<float>> zeroRotationMat = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    calibHandler.setCameraExtrinsics(
        dai::CameraBoardSocket::CAM_B,
	dai::CameraBoardSocket::CAM_C,
	zeroRotationMat,
	{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    // clang-format on

    auto err = dynCalib->runCalibration(calibHandler, force);
    EXPECT_EQ(err, dai::node::DynamicCalibration::ErrorCode::OK);

    EXPECT_EQ(returnedData->getSize(), 1);
    auto returnedCalibration = returnedData->tryGet<dai::DynamicCalibrationResult>();
    EXPECT_EQ(returnedData->getSize(), 0);

    auto calibHandlerReturned = returnedCalibration->calibration;

    if(calibHandlerReturned.has_value()) {
        // clang-format off
        auto returnedCameraMatrixA = calibHandlerReturned->getCameraIntrinsics(
	    dynCalib->getBorderSockerA(), dynCalib->getWidth(), dynCalib->getHeight());
        auto returnedCameraMatrixB = calibHandlerReturned->getCameraIntrinsics(
	    dynCalib->getBorderSockerB(), dynCalib->getWidth(), dynCalib->getHeight());
        auto returnedDistortionA = calibHandlerReturned->getDistortionCoefficients(dynCalib->getBorderSockerA());
        auto returnedDistortionB = calibHandlerReturned->getDistortionCoefficients(dynCalib->getBorderSockerB());

        auto returnedRotationMatrix = calibHandlerReturned->getCameraRotationMatrix(dynCalib->getBorderSockerA(), dynCalib->getBorderSockerB());

        auto returnedTranslation = calibHandlerReturned->getCameraTranslationVector(
	    dynCalib->getBorderSockerA(), dynCalib->getBorderSockerB(), false);

        // clang-format on
        EXPECT_EQ(returnedCameraMatrixA[0][0], 1.0f);
        EXPECT_EQ(returnedCameraMatrixB[0][0], 1.0f);
        EXPECT_EQ(returnedCameraMatrixA[1][0], 0.0f);
        EXPECT_EQ(returnedCameraMatrixB[1][0], 0.0f);
        EXPECT_EQ(returnedDistortionA[0], 4.0f);
        EXPECT_EQ(returnedDistortionB[0], 4.0f);
        EXPECT_EQ(returnedDistortionA[1], 2.0f);
        EXPECT_EQ(returnedDistortionB[1], 2.0f);
        EXPECT_NE(returnedRotationMatrix[0][0], 0.0f);
        EXPECT_NE(returnedRotationMatrix[0][1], 0.0f);
        EXPECT_EQ(returnedTranslation[1], 500.0f);
    } else {
        FAIL() << "CalibrationHandler not available.";
    }
}

TEST(DynamicCalibration, LoadImage_Empty) {
    MockDynamicCalibration dynCalib;
    auto error = dynCalib.runLoadImage();
    EXPECT_EQ(error, dai::node::DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE);
}

TEST(DynamicCalibration, LoadImage_EmptyMessage) {
    auto leftFrame = std::make_shared<dai::ImgFrame>();
    auto rightFrame = std::make_shared<dai::ImgFrame>();
    {
        MockDynamicCalibration dynCalib;
        auto msgGroup = std::make_shared<dai::MessageGroup>();
        dynCalib.syncInput.send(msgGroup);
        auto error = dynCalib.runLoadImage();
        EXPECT_EQ(error, dai::node::DynamicCalibration::ErrorCode::MISSING_IMAGE);
    }
    {
        MockDynamicCalibration dynCalib;
        auto msgGroup = std::make_shared<dai::MessageGroup>();
        msgGroup->add(dynCalib.rightInputName, rightFrame);
        dynCalib.syncInput.send(msgGroup);
        auto error = dynCalib.runLoadImage();
        EXPECT_EQ(error, dai::node::DynamicCalibration::ErrorCode::MISSING_IMAGE);
    }
    {
        MockDynamicCalibration dynCalib;
        auto msgGroup = std::make_shared<dai::MessageGroup>();
        msgGroup->add(dynCalib.leftInputName, leftFrame);
        dynCalib.syncInput.send(msgGroup);
        auto error = dynCalib.runLoadImage();
        EXPECT_EQ(error, dai::node::DynamicCalibration::ErrorCode::MISSING_IMAGE);
    }
}

TEST(DynamicCalibration, LoadImage_OK) {
    uint8_t leftData[] = {1, 2, 3, 4};
    cv::Mat leftMat(2, 2, CV_8UC1, leftData);
    auto leftFrame = std::make_shared<dai::ImgFrame>();
    leftFrame->setWidth(2);
    leftFrame->setHeight(2);
    leftFrame->setCvFrame(leftMat, dai::ImgFrame::Type::GRAY8);

    uint8_t rightData[] = {5, 6, 7, 8};
    cv::Mat rightMat(2, 2, CV_8UC1, rightData);
    auto rightFrame = std::make_shared<dai::ImgFrame>();
    rightFrame->setWidth(2);
    rightFrame->setHeight(2);
    rightFrame->setSourceSize(2, 2);
    rightFrame->setCvFrame(rightMat, dai::ImgFrame::Type::GRAY8);
    auto rightCvFrame = rightFrame->getCvFrame();

    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    MockDynamicCalibration dynCalib(std::move(mockDcl));

    auto successResult = dcl::Result<void>::ok();

    EXPECT_CALL(*mockDclRaw,
                loadStereoImagePair(testing::Truly([](const dcl::ImageData& imageA) {
                                        if(imageA.width != 2 || imageA.height != 2) return false;
                                        if(imageA.size() != 4) return false;
                                        const auto fdata = imageA.data.data();
                                        return fdata[0] == 1 && fdata[1] == 2 && fdata[2] == 3 && fdata[3] == 4;
                                    }),
                                    testing::Truly([](const dcl::ImageData& imageB) {
                                        if(imageB.width != 2 || imageB.height != 2) return true;
                                        if(imageB.size() != 4) return false;
                                        const auto fdata = imageB.data.data();
                                        return fdata[0] == 5 && fdata[1] == 6 && fdata[2] == 7 && fdata[3] == 8;
                                    }),
                                    testing::_,
                                    testing::_,
                                    testing::_,
                                    testing::_))
        .WillOnce(testing::Return(successResult));

    auto msgGroup = std::make_shared<dai::MessageGroup>();
    msgGroup->add(dynCalib.rightInputName, rightFrame);
    msgGroup->add(dynCalib.leftInputName, leftFrame);
    dynCalib.syncInput.send(msgGroup);
    auto error = dynCalib.runLoadImage();
    EXPECT_EQ(error, dai::node::DynamicCalibration::ErrorCode::OK);
}

TEST(DynamicCalibration, ComputeCoverage_Error) {
    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    MockDynamicCalibration dynCalib(std::move(mockDcl));

    auto coverageResult = dcl::Result<dcl::CoverageData>::error(1);

    dai::DynamicCalibrationConfig config;
    config.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib.setInitialConfig(config);

    // clang-format off
    EXPECT_CALL(*mockDclRaw, computeCoverage(testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE)).WillOnce(testing::Return(coverageResult));
    // clang-format on

    EXPECT_THROW(dynCalib.computeCoverage(), std::runtime_error);
}

TEST(DynamicCalibration, ComputeCoverage_OK) {
    dai::Pipeline pipeline(false);

    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>(std::move(mockDcl));

    dcl::CoverageData coverageData;
    coverageData.coveragePerCellA = {{1.0f, 2.0f}, {3.0f, 4.0f}};
    coverageData.coveragePerCellB = {{5.0f, 6.0f}, {7.0f, 8.0f}};
    coverageData.meanCoverage = 4.2;

    auto coverageResult = dcl::Result<dcl::CoverageData>::ok(coverageData);

    // clang-format off
    EXPECT_CALL(*mockDclRaw, computeCoverage(testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE))
        .WillOnce(testing::Return(coverageResult));
    // clang-format on

    dai::DynamicCalibrationConfig config;
    config.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib->setInitialConfig(config);

    auto returnedData = dynCalib->coverageOutput.createOutputQueue();  // tryGet<dai::DatatypeEnum::CoverageData>();

    EXPECT_NO_THROW(dynCalib->computeCoverage());
    EXPECT_EQ(returnedData->getSize(), 1);
    auto returnedCoverage = returnedData->tryGet<dai::CoverageData>();
    EXPECT_EQ(returnedData->getSize(), 0);
    EXPECT_TRUE(returnedCoverage);
    EXPECT_EQ(returnedCoverage->coveragePerCellA[0][0], 1.0f);
    EXPECT_EQ(returnedCoverage->coveragePerCellA[0][1], 2.0f);
    EXPECT_EQ(returnedCoverage->coveragePerCellA[1][0], 3.0f);
    EXPECT_EQ(returnedCoverage->coveragePerCellA[1][1], 4.0f);

    EXPECT_EQ(returnedCoverage->coveragePerCellB[0][0], 5.0f);
    EXPECT_EQ(returnedCoverage->coveragePerCellB[0][1], 6.0f);
    EXPECT_EQ(returnedCoverage->coveragePerCellB[1][0], 7.0f);
    EXPECT_EQ(returnedCoverage->coveragePerCellB[1][1], 8.0f);
}

TEST(DynamicCalibration, DoWork) {
    dai::Pipeline pipeline(false);

    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>(std::move(mockDcl));

    auto command = std::make_shared<dai::StartRecalibrationCommand>();

    auto inputCommandQueue = dynCalib->commandInput.createInputQueue();
    inputCommandQueue->send(command);
}

TEST(DynamicCalibration, DoWork_CallsInnerMockDclMethods) {
    using namespace std::chrono;

    dai::Pipeline pipeline(false);

    // Create mock and raw pointer
    auto mockDcl = std::make_unique<MockDclDynamicCalibration>();
    MockDclDynamicCalibration* mockDclRaw = mockDcl.get();
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>(std::move(mockDcl));

    // Set sockets and performance mode for consistent behavior
    dai::DynamicCalibrationConfig config;
    config.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib->setInitialConfig(config);

    dynCalib->setWidth(1280);
    dynCalib->setHeight(800);

    // Setup previous time to force action
    auto previousLoadingTime = steady_clock::now() - seconds(10);
    dynCalib->properties.initialConfig.calibrationPeriod = 1;

    uint8_t leftData[] = {1, 2, 3, 4};
    cv::Mat leftMat(2, 2, CV_8UC1, leftData);
    auto leftFrame = std::make_shared<dai::ImgFrame>();
    leftFrame->setWidth(2);
    leftFrame->setHeight(2);
    leftFrame->setCvFrame(leftMat, dai::ImgFrame::Type::GRAY8);

    uint8_t rightData[] = {5, 6, 7, 8};
    cv::Mat rightMat(2, 2, CV_8UC1, rightData);
    auto rightFrame = std::make_shared<dai::ImgFrame>();
    rightFrame->setWidth(2);
    rightFrame->setHeight(2);
    rightFrame->setSourceSize(2, 2);
    rightFrame->setCvFrame(rightMat, dai::ImgFrame::Type::GRAY8);
    auto rightCvFrame = rightFrame->getCvFrame();

    {
        dcl::CoverageData coverageData;
        auto coverageResult = dcl::Result<dcl::CoverageData>::ok(coverageData);
        // // clang-format off
        // EXPECT_CALL(*mockDclRaw, computeCoverage(testing::_, testing::_, testing::_))
        //     .WillOnce(testing::Return(coverageResult));
        // // clang-format on
        // auto command = std::make_shared<dai::LoadImageCommand>();
        // dynCalib->commandInput.send(command);
        // auto err1 = dynCalib->doWork(previousLoadingTime);
        // EXPECT_EQ(err1, dai::node::DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE);

        // clang-format off
	EXPECT_CALL(*mockDclRaw, computeCoverage(testing::_, testing::_, testing::_))
	    .WillOnce(testing::Return(coverageResult));
        // clang-format on

        auto successResult = dcl::Result<void>::ok();

        EXPECT_CALL(*mockDclRaw, loadStereoImagePair(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
            .WillOnce(testing::Return(successResult));

        auto msgGroup = std::make_shared<dai::MessageGroup>();
        msgGroup->add(dynCalib->rightInputName, rightFrame);
        msgGroup->add(dynCalib->leftInputName, leftFrame);
        dynCalib->syncInput.send(msgGroup);

        auto stopCmd = std::make_shared<dai::LoadImageCommand>();
        dynCalib->commandInput.send(stopCmd);
        auto err2 = dynCalib->doWork(previousLoadingTime);
        EXPECT_EQ(err2, dai::node::DynamicCalibration::ErrorCode::OK);
    }
    {
        // Expect checkCalibration to be called when forcing quality check
        EXPECT_CALL(*mockDclRaw, checkCalibration(testing::_, testing::_, testing::_, dcl::PerformanceMode::SKIP_CHECKS))
            .Times(1)
            .WillOnce(testing::Return(dcl::Result<dcl::CalibrationQuality>::ok({})));

        auto forceQcCmd = std::make_shared<dai::CalibrationQualityCommand>();
        forceQcCmd->force = true;
        dynCalib->commandInput.send(forceQcCmd);

        auto err1 = dynCalib->doWork(previousLoadingTime);
        EXPECT_EQ(err1, dai::node::DynamicCalibration::ErrorCode::OK);
    }
    {
        // Expect checkCalibration to be called when forcing quality check
        EXPECT_CALL(*mockDclRaw, checkCalibration(testing::_, testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE))
            .Times(1)
            .WillOnce(testing::Return(dcl::Result<dcl::CalibrationQuality>::ok({})));

        auto qcCmd = std::make_shared<dai::CalibrationQualityCommand>();
        qcCmd->force = false;
        dynCalib->commandInput.send(qcCmd);

        auto err1 = dynCalib->doWork(previousLoadingTime);
        EXPECT_EQ(err1, dai::node::DynamicCalibration::ErrorCode::OK);
    }
    {
        auto failedResult = dcl::Result<std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>>>::error(1);
        // Expect checkCalibration to be called when forcing quality check
        EXPECT_CALL(*mockDclRaw, findNewCalibration(testing::_, testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE))
            .Times(1)
            .WillOnce(testing::Return(failedResult));

        auto successResult = dcl::Result<void>::ok();

        EXPECT_CALL(*mockDclRaw,
                    loadStereoImagePair(testing::Truly([](const dcl::ImageData& imageA) {
                                            if(imageA.width != 2 || imageA.height != 2) return false;
                                            if(imageA.size() != 4) return false;
                                            const auto fdata = imageA.data.data();
                                            return fdata[0] == 1 && fdata[1] == 2 && fdata[2] == 3 && fdata[3] == 4;
                                        }),
                                        testing::Truly([](const dcl::ImageData& imageB) {
                                            if(imageB.width != 2 || imageB.height != 2) return true;
                                            if(imageB.size() != 4) return false;
                                            const auto fdata = imageB.data.data();
                                            return fdata[0] == 5 && fdata[1] == 6 && fdata[2] == 7 && fdata[3] == 8;
                                        }),
                                        testing::_,
                                        testing::_,
                                        testing::_,
                                        testing::_))
            .WillOnce(testing::Return(successResult));

        dcl::CoverageData coverageData;

        auto coverageResult = dcl::Result<dcl::CoverageData>::ok(coverageData);
        // clang-format off
	EXPECT_CALL(*mockDclRaw, computeCoverage(testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE))
	    .WillOnce(testing::Return(coverageResult));
        // clang-format on

        auto recalcCmd = std::make_shared<dai::StartRecalibrationCommand>();
        dynCalib->commandInput.send(recalcCmd);

        auto err1 = dynCalib->doWork(previousLoadingTime);
        EXPECT_EQ(err1, dai::node::DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE);

        auto msgGroup = std::make_shared<dai::MessageGroup>();
        msgGroup->add(dynCalib->rightInputName, rightFrame);
        msgGroup->add(dynCalib->leftInputName, leftFrame);
        dynCalib->syncInput.send(msgGroup);

        // clang-format off
	EXPECT_CALL(*mockDclRaw, computeCoverage(testing::_, testing::_, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE))
	    .WillOnce(testing::Return(coverageResult));
        // clang-format on

        auto err2 = dynCalib->doWork(previousLoadingTime);
        EXPECT_EQ(err2, dai::node::DynamicCalibration::ErrorCode::CALIBRATION_FAILED);
    }
}
