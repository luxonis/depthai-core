#include "depthai/pipeline/node/DynamicCalibration.hpp"

#include <DynamicCalibration.hpp>
#include <opencv2/opencv.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

DynamicCalibration::Properties& DynamicCalibration::getProperties() {
    return properties;
}

void DynamicCalibration::setPerformanceMode(dcl::PerformanceMode mode) {
    properties.performanceMode = mode;
}

void DynamicCalibration::setContinousMode() {
    properties.recalibrationMode = dai::DynamicCalibrationProperties::RecalibrationMode::CONTINUOUS;
}

void DynamicCalibration::setTimeFrequency(int time) {
    properties.timeFrequency = time;
}

void DynamicCalibration::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool DynamicCalibration::runOnHost() const {
    return runOnHostVar;
}

void DynamicCalibration::buildInternal() {
    sync->out.link(inSync);
    sync->setRunOnHost(true);
}

std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> DclUtils::convertDaiCalibrationToDcl(
    const CalibrationHandler currentCalibration,
    const CameraBoardSocket boardSocketA,
    const CameraBoardSocket boardSocketB,
    const int width,
    const int height) {
    // clang-format off
    std::shared_ptr<dcl::CameraCalibrationHandle> calibA = DclUtils::createDclCalibration(
        currentCalibration.getCameraIntrinsics(boardSocketA, width, height),
        currentCalibration.getDistortionCoefficients(boardSocketA),
	{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
	{0.0f, 0.0f, 0.0f}
    );
    std::shared_ptr<dcl::CameraCalibrationHandle> calibB = DclUtils::createDclCalibration(
        currentCalibration.getCameraIntrinsics(boardSocketB, width, height),
        currentCalibration.getDistortionCoefficients(boardSocketB),
	currentCalibration.getCameraRotationMatrix(boardSocketA, boardSocketB),
	currentCalibration.getCameraTranslationVector(boardSocketA, boardSocketB, false)
    );
    // clang-format on
    return std::make_pair(calibA, calibB);
}

std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::createDclCalibration(const std::vector<std::vector<float>> cameraMatrix,
                                                                             const std::vector<float> distortionCoefficients,
                                                                             const std::vector<std::vector<float>> rotationMatrix,
                                                                             const std::vector<float> translationVector) {
    dcl::scalar_t cameraMatrixArr[9];
    dcl::scalar_t distortion[14] = {0};
    dcl::scalar_t rvec[3];
    dcl::scalar_t tvec[3];

    // Convert cameraMatrix
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraMatrixArr[i * 3 + j] = static_cast<dcl::scalar_t>(cameraMatrix[i][j]);
        }
    }

    // Convert distortion
    for(size_t i = 0; i < distortionCoefficients.size() && i < 14; ++i) {
        distortion[i] = static_cast<dcl::scalar_t>(distortionCoefficients[i]);
    }

    // Convert rotation to vector
    std::vector<float> rvecVec = matrix::rotationMatrixToVector(rotationMatrix);
    for(int i = 0; i < 3; ++i) {
        rvec[i] = static_cast<dcl::scalar_t>(rvecVec[i]);
    }

    for(int i = 0; i < 3; ++i) {
        tvec[i] = static_cast<dcl::scalar_t>(translationVector[i] / 100.0f);  // Convert to m
    }

    return std::make_shared<dcl::CameraCalibrationHandle>(rvec, tvec, cameraMatrixArr, distortion);
}

void DclUtils::convertDclCalibrationToDai(CalibrationHandler& calibHandler,
                                          const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibration,
                                          const CameraBoardSocket socketSrc,
                                          const CameraBoardSocket socketDest,
                                          const int width,
                                          const int height) {
    dcl::scalar_t distortion[14];
    daiCalibration->getDistortion(distortion);
    calibHandler.setDistortionCoefficients(socketDest, std::vector<float>(distortion, distortion + 14));

    dcl::scalar_t cameraMatrix[9];
    daiCalibration->getCameraMatrix(cameraMatrix);
    // clang-format off
    std::vector<std::vector<float>> mat = {
        {static_cast<float>(cameraMatrix[0]), static_cast<float>(cameraMatrix[1]), static_cast<float>(cameraMatrix[2])},
        {static_cast<float>(cameraMatrix[3]), static_cast<float>(cameraMatrix[4]), static_cast<float>(cameraMatrix[5])},
        {static_cast<float>(cameraMatrix[6]), static_cast<float>(cameraMatrix[7]), static_cast<float>(cameraMatrix[8])}
    };
    // clang-format on 
    calibHandler.setCameraIntrinsics(socketDest, mat, width, height);

    dcl::scalar_t tvec[3];
    daiCalibration->getTvec(tvec);
    auto translation = std::vector<float>(tvec, tvec + 3);
    for(auto& val : translation) {
        val *= 100.0f;
    }
    dcl::scalar_t rvec[3];
    daiCalibration->getRvec(rvec);
    auto rotationMatrix = matrix::rvecToRotationMatrix(rvec);
    auto specTranslation = calibHandler.getCameraTranslationVector(socketSrc, socketDest, true);

    calibHandler.setCameraExtrinsics(socketSrc, socketDest, rotationMatrix, translation, specTranslation);
}

dcl::ImageData DclUtils::cvMatToImageData(const cv::Mat& mat) {
    if(mat.empty()) {
        throw std::runtime_error("cv::Mat is empty");
    }

    dcl::ImageData img;
    img.width = static_cast<unsigned int>(mat.cols);
    img.height = static_cast<unsigned int>(mat.rows);
    img.data.assign(mat.data, mat.data + mat.total() * mat.elemSize());

    int type = mat.type();
    switch(type) {
        case CV_8UC1:
            img.format = dcl::DCL_8UC1;
            break;
        case CV_8UC3:
            img.format = dcl::DCL_8UC3;
            break;
        default:
            throw std::runtime_error("Unsupported cv::Mat type: " + std::to_string(type));
    }

    return img;
}

dai::DynamicCalibrationResults::CalibrationQualityResult DynamicCalibration::calibQualityfromDCL(const dcl::CalibrationQuality& src) {
    dai::DynamicCalibrationResults::CalibrationQualityResult out_report;
    auto& out = out_report.report.emplace();
    out.dataAcquired = src.dataAquired;
    if(src.coverageQuality.has_value()) {
        auto& cov = out.coverageQuality.emplace();
        cov.coveragePerCellA = src.coverageQuality->coveragePerCellA;
        cov.coveragePerCellB = src.coverageQuality->coveragePerCellB;
        cov.meanCoverage = src.coverageQuality->meanCoverage;
    }

    if(src.calibrationQuality.has_value()) {
        auto& cal = out.calibrationQuality.emplace();
        for(int i = 0; i < 3; ++i) {
            cal.rotationChange[i] = src.calibrationQuality->rotationChange[i];
        }
        cal.epipolarErrorChange = src.calibrationQuality->epipolarErrorChange;
        cal.depthErrorDifference = src.calibrationQuality->depthDistanceDifference;
    }
    return out_report;
}

void DynamicCalibration::setDclCalibration(
    std::shared_ptr<Device> device, const CameraBoardSocket boardSocketA, const CameraBoardSocket boardSocketB, const int width, const int height) {
    CalibrationHandler currentCalibration = device->getCalibration();
    auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(currentCalibration, boardSocketA, boardSocketB, width, height);
    dynCalibImpl->setNewCalibration(deviceName, socketA, calibA->getCalibration());
    dynCalibImpl->setNewCalibration(deviceName, socketB, calibB->getCalibration());
}

void DynamicCalibration::resetResults() {
    calibQuality.calibrationQuality = dcl::CalibrationData{};
    calibQuality.coverageQuality = dcl::CoverageData{};
    calibQuality.dataAquired = 0.f;
    dynResult.calibOverallQuality = calibQualityfromDCL(calibQuality);
    dynResult.newCalibration = DynamicCalibrationResults::CalibrationResult::Invalid();
    dynResult.calibOverallQuality = DynamicCalibrationResults::CalibrationQualityResult::Invalid();
    dynResult.newCalibration = DynamicCalibrationResults::CalibrationResult::Invalid();
    dynResult.info = "";
}

DynamicCalibration::ErrorCode DynamicCalibration::runQualityCheck() {
    auto result = dynCalibImpl->checkCalibration(dcDevice, socketA, socketB, properties.performanceMode)

    if (!result.passed()) {
      return DynamicCalibration::ErrorCode::QUALITY_CHECK_FAILED; 
    }

    dynResult.info = result.errorMessage();
    dynResult.calibOverallQuality = calibQualityfromDCL(calibQuality);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::runCalibration(const bool force) {
    auto result = dynCalibImpl->findNewCalibration(dcDevice, socketA, socketB, properties.performanceMode)

    if (!result.passed()) {
      return DynamicCalibration::ErrorCode::CALIBRATION_FAILED; 
    }

    dynResult.info = result.errorMessage();
    dynResult.calibOverallQuality = calibQualityfromDCL(calibQuality);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::runLoadImage() {
    auto inSyncGroup = inSync.tryGet<dai::MessageGroup>();
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::NoSyncGroup;
    };
    auto leftFrame = inSyncGroup->get<dai::ImgFrame>(leftInputName);
    auto rightFrame = inSyncGroup->get<dai::ImgFrame>(rightInputName);
    if(!leftFrame || !rightFrame) {
        return DynamicCalibration::ErrorCode::MissingImage;
    };

    dynCalibImpl->loadStereoImagePair(
        DclUtils::cvMatToImageData(leftFrame->getCvFrame()),
        DclUtils::cvMatToImageData(rightFrame->getCvFrame()),
	deviceName, socketA, socketB, timestamp);
    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::computeCoverage() {
    auto resultCoverage = dunCalibImpl->computeCoverage(sensorA, sensorB, properties.performanceMode);

    if (!resultCoverage.paseed()) {
        std::runtime_error("Coverage check failed!");
    }

    auto& coverage = resultCoverage.value;
    dynResult.calibOverallQuality = {
       .coveragePerCellA = coverage.coveragePerCellA,
       .coveragePerCellB = coverage.coveragePerCellB,
       .meanCoverage =coverage.meanCoverage;
    }
    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::initializePipeline() {
    for (int i = 0; i < maxNumberOfAttempts; ++i) {
        auto inSyncGroup = inSync.tryGet<dai::MessageGroup>();
        if(!inSyncGroup) {
            continue;
        };
        auto leftFrame = inSyncGroup->get<dai::ImgFrame>(leftInputName);
        auto rightFrame = inSyncGroup->get<dai::ImgFrame>(rightInputName);
        if(!leftFrame || !rightFrame) {
            continue;
        };
        auto width = leftFrame->getWidth();
        auto height = rightFrame->getHeight();

	daiSocketA = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
	daiSocketB = static_cast<CameraBoardSocket>(rightFrame->instanceNum);

        socketA = static_cast<dcl::socket_t>(boardSocketA);
        socketB = static_cast<dcl::socket_t>(boardSocketB);

	CalibrationHandler currentCalibration = device->getCalibration();

	auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(currentCalibration, daiSocketA, daiSocketB, width, height);

        // set up the dynamic calibration
	dynCalibImpl = std::make_unique<dcl::DynamicCalibration>();
	deviceName = device->getDeviceId();
	dcDevice = dynCalibImpl->addDevice(deviceName);
	const dcl::resolution_t resolution = {.width = width, .height = height};
	sensorA = std::make_shared<dcl::CameraSensorHandle>(calibA, resolution);
	sensorB = std::make_shared<dcl::CameraSensorHandle>(calibB, resolution);
	dynCalibImpl->addSensor(deviceName, sensorA, socketA);
	dynCalibImpl->addSensor(deviceName, sensorB, socketB);

	return DynamicCalibration::ErrorCode::OK;
    }
    return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
}

void DynamicCalibration::run() {
    if(!device) {
        logger::error("Dynamic calibration node does not have access to any device.");
        return;
    }

    logger::info("DynamicCalibration node is running");
    auto lastAutoTrigger = std::chrono::steady_clock::now();
    auto previousLoadingTime = std::chrono::steady_clock::now();
    initializePipeline();
    if(!properties.recalibrationMode == dai::DynamicCalibrationProperties::RecalibrationMode::CONTINUOUS) {
        while (isRunning()) {
            auto calibrationCommand = properties.calibrationCommand;
            auto now = std::chrono::steady_clock::now();

	    if (!calibrationCommand.has_value()) {
		if (now - previousLoadingTime > std::chrono::seconds(properties.calibrationFrequency)) {
		    runLoadImage();
		    computeCoverage();
		    previousLoadingTime = std::chrono::steady_clock::now();
		}
                continue;
	    }

            switch (calibrationCommand.value()) {
                case START_CALIBRATION_QUALITY_CHECK:
        	    runQualityCheck();
        	    break;
                	
                case START_RECALIBRATION:
        	    runCalibration();
                    break;
                	
                case START_FORCE_RECALIBRATION:
        	    runCalibration(force=true);
                    break;
                	
                default:
		    std::runtime_error("Command not found!");
            }
	    calibrationCommand = std::nullopt;
        }
    } else {
        // Continuous mode 
        auto previousCalibrationTime = std::chrono::steady_clock::now();
        while(isRunning()) {
            auto calibrationCommand = properties.calibrationCommand;
            auto now = std::chrono::steady_clock::now();
	    if (now - previousLoadingTime > std::chrono::seconds(properties.loadImageFrequency)) {
	        runLoadImage();
		// do we want to return the coverage in the continuous mode? -> computeCoverage();
		previousLoadingTime = std::chrono::steady_clock::now();
	    }
	    if (now - previousCalibrationTime > std::chrono::seconds(properties.loadImageFrequency)) {
	        runCalibration();
		previousCalibrationTime = std::chrono::steady_clock::now();
	    }
        }
    }
}

}  // namespace node
}  // namespace dai
