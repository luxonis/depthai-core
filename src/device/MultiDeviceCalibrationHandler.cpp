#include "device/MultiDeviceCalibrationHandler.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include "depthai/common/MultiDeviceCalibrationData.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "nlohmann/json.hpp"

namespace dai {

namespace {

constexpr uint32_t kSupportedMultiDeviceCalibrationVersion = 1;
constexpr float kRotationValidationTolerance = 1e-4f;

float determinant3x3(const std::array<std::array<float, 3>, 3>& matrix) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
           - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
           + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

std::array<std::array<float, 3>, 3> transpose3x3(const std::array<std::array<float, 3>, 3>& matrix) {
    std::array<std::array<float, 3>, 3> transpose{};
    for(size_t row = 0; row < 3; ++row) {
        for(size_t col = 0; col < 3; ++col) {
            transpose[row][col] = matrix[col][row];
        }
    }
    return transpose;
}

void validateFiniteTranslation(const Point3f& translation, const std::string& context) {
    if(std::isfinite(translation.x) && std::isfinite(translation.y) && std::isfinite(translation.z)) {
        return;
    }

    throw std::runtime_error(context + " has a non-finite translation component");
}

void validateRotationMatrix(const std::array<std::array<float, 3>, 3>& rotationMatrix, const std::string& context) {
    for(const auto& row : rotationMatrix) {
        for(float value : row) {
            if(!std::isfinite(value)) {
                throw std::runtime_error(context + " has a non-finite rotation matrix element");
            }
        }
    }

    const auto transpose = transpose3x3(rotationMatrix);
    const auto shouldBeIdentity = matrix::matMul(transpose, rotationMatrix);
    const std::array<std::array<float, 3>, 3> identity{
        std::array<float, 3>{1.0f, 0.0f, 0.0f},
        std::array<float, 3>{0.0f, 1.0f, 0.0f},
        std::array<float, 3>{0.0f, 0.0f, 1.0f},
    };
    if(!matrix::mateq(shouldBeIdentity, identity, kRotationValidationTolerance)) {
        throw std::runtime_error(context + " must satisfy R^T R ~= I");
    }

    const float determinant = determinant3x3(rotationMatrix);
    if(std::fabs(determinant - 1.0f) > kRotationValidationTolerance) {
        std::ostringstream message;
        message << context << " must have determinant ~= +1, got " << determinant;
        throw std::runtime_error(message.str());
    }
}

}  // namespace

MultiDeviceCalibrationHandler::MultiDeviceCalibrationHandler(const MultiDeviceCalibrationData& data, std::optional<bool> validate) : data(data) {
    if(validate.value_or(false)) {
        this->validate();
    }
}

MultiDeviceCalibrationHandler::MultiDeviceCalibrationHandler(std::filesystem::path path, std::optional<bool> validate) {
    std::ifstream jsonStream(path);
    if(!jsonStream.is_open()) {
        throw std::runtime_error("Multi-device calibration data file doesn't exist at the provided path. Please provide an absolute path.");
    }
    if(!jsonStream.good() || jsonStream.bad()) {
        throw std::runtime_error("Multi-device calibration data file not found or corrupted");
    }

    data = nlohmann::json::parse(jsonStream).get<MultiDeviceCalibrationData>();
    if(validate.value_or(false)) {
        this->validate();
    }
}

MultiDeviceCalibrationHandler MultiDeviceCalibrationHandler::fromJson(const nlohmann::json& json, std::optional<bool> validate) {
    MultiDeviceCalibrationHandler handler(json.get<MultiDeviceCalibrationData>(), false);
    if(validate.value_or(false)) {
        handler.validate();
    }
    return handler;
}

MultiDeviceCalibrationData MultiDeviceCalibrationHandler::getData() const {
    return data;
}

nlohmann::json MultiDeviceCalibrationHandler::toJson() const {
    return data;
}

bool MultiDeviceCalibrationHandler::toJsonFile(std::filesystem::path path) const {
    std::ofstream output(path);
    output << std::setw(4) << toJson() << std::endl;
    return true;
}

bool MultiDeviceCalibrationHandler::hasDevice(const std::string& mxid) const {
    return std::any_of(data.devices.begin(), data.devices.end(), [&mxid](const auto& device) { return device.mxid == mxid; });
}

std::vector<std::string> MultiDeviceCalibrationHandler::getDeviceIds() const {
    std::vector<std::string> ids;
    ids.reserve(data.devices.size());
    for(const auto& device : data.devices) {
        ids.push_back(device.mxid);
    }
    return ids;
}

MultiDeviceCalibrationDevice MultiDeviceCalibrationHandler::getDevice(const std::string& mxid) const {
    return getDeviceRef(mxid);
}

CalibrationHandler MultiDeviceCalibrationHandler::getDeviceCalibration(const std::string& mxid) const {
    return CalibrationHandler(getDeviceRef(mxid).calibration);
}

void MultiDeviceCalibrationHandler::setDevice(const std::string& mxid,
                                              const CalibrationHandler& calibration,
                                              const MultiDeviceFrame& anchorFrame,
                                              const RigidTransform& rigFromAnchor) {
    MultiDeviceCalibrationData proposed = data;
    proposed.devices.erase(std::remove_if(proposed.devices.begin(), proposed.devices.end(), [&mxid](const auto& device) { return device.mxid == mxid; }),
                           proposed.devices.end());

    MultiDeviceCalibrationDevice replacement;
    replacement.mxid = mxid;
    replacement.calibration = calibration.getEepromData();
    replacement.anchorFrame = anchorFrame;
    replacement.rigFromAnchor = rigFromAnchor;
    proposed.devices.push_back(replacement);

    validateData(proposed, true);
    data = std::move(proposed);
}

void MultiDeviceCalibrationHandler::removeDevice(const std::string& mxid) {
    data.devices.erase(std::remove_if(data.devices.begin(), data.devices.end(), [&mxid](const auto& device) { return device.mxid == mxid; }),
                       data.devices.end());
}

void MultiDeviceCalibrationHandler::validate(bool throwOnError) const {
    validateData(data, throwOnError);
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getFrameExtrinsics(const std::string& srcMxid,
                                                                                  const MultiDeviceFrame& srcFrame,
                                                                                  const std::string& dstMxid,
                                                                                  const MultiDeviceFrame& dstFrame,
                                                                                  bool useSpecTranslation,
                                                                                  LengthUnit unit) const {
    validateFrameState(srcFrame, "source query frame");
    validateFrameState(dstFrame, "destination query frame");

    if(srcMxid == dstMxid && srcFrame == dstFrame) {
        return createIdentityTransform();
    }

    const auto& srcDevice = getDeviceRef(srcMxid);
    const auto& dstDevice = getDeviceRef(dstMxid);

    auto rigFromSrc = getRigFromLocalFrame(srcDevice, srcFrame, useSpecTranslation);
    auto rigFromDst = getRigFromLocalFrame(dstDevice, dstFrame, useSpecTranslation);

    matrix::invertSe3Matrix4x4InPlace(rigFromDst);
    auto dstFromSrc = matrix::matMul(rigFromDst, rigFromSrc);
    scaleTranslationFromCentimetersInPlace(dstFromSrc, unit);
    return dstFromSrc;
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getDeviceExtrinsics(const std::string& srcMxid,
                                                                                   const std::string& dstMxid,
                                                                                   LengthUnit unit) const {
    if(srcMxid == dstMxid) {
        return createIdentityTransform();
    }

    auto srcRigFromAnchor = rigFromAnchorToCentimeterMatrix(getDeviceRef(srcMxid).rigFromAnchor);
    auto dstRigFromAnchor = rigFromAnchorToCentimeterMatrix(getDeviceRef(dstMxid).rigFromAnchor);
    matrix::invertSe3Matrix4x4InPlace(dstRigFromAnchor);

    auto dstFromSrc = matrix::matMul(dstRigFromAnchor, srcRigFromAnchor);
    scaleTranslationFromCentimetersInPlace(dstFromSrc, unit);
    return dstFromSrc;
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getCameraExtrinsics(const std::string& srcMxid,
                                                                                   CameraBoardSocket srcCamera,
                                                                                   const std::string& dstMxid,
                                                                                   CameraBoardSocket dstCamera,
                                                                                   bool useSpecTranslation,
                                                                                   LengthUnit unit) const {
    return getFrameExtrinsics(srcMxid, MultiDeviceFrame::camera(srcCamera), dstMxid, MultiDeviceFrame::camera(dstCamera), useSpecTranslation, unit);
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getCameraToRigExtrinsics(const std::string& mxid,
                                                                                        CameraBoardSocket camera,
                                                                                        bool useSpecTranslation,
                                                                                        LengthUnit unit) const {
    auto rigFromCamera = getRigFromLocalFrame(getDeviceRef(mxid), MultiDeviceFrame::camera(camera), useSpecTranslation);
    scaleTranslationFromCentimetersInPlace(rigFromCamera, unit);
    return rigFromCamera;
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getRigToCameraExtrinsics(const std::string& mxid,
                                                                                        CameraBoardSocket camera,
                                                                                        bool useSpecTranslation,
                                                                                        LengthUnit unit) const {
    auto cameraFromRig = getRigFromLocalFrame(getDeviceRef(mxid), MultiDeviceFrame::camera(camera), useSpecTranslation);
    matrix::invertSe3Matrix4x4InPlace(cameraFromRig);
    scaleTranslationFromCentimetersInPlace(cameraFromRig, unit);
    return cameraFromRig;
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getHousingToRigExtrinsics(const std::string& mxid,
                                                                                         HousingCoordinateSystem housing,
                                                                                         bool useSpecTranslation,
                                                                                         LengthUnit unit) const {
    auto rigFromHousing = getRigFromLocalFrame(getDeviceRef(mxid), MultiDeviceFrame::housingFrame(housing), useSpecTranslation);
    scaleTranslationFromCentimetersInPlace(rigFromHousing, unit);
    return rigFromHousing;
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getRigToHousingExtrinsics(const std::string& mxid,
                                                                                         HousingCoordinateSystem housing,
                                                                                         bool useSpecTranslation,
                                                                                         LengthUnit unit) const {
    auto housingFromRig = getRigFromLocalFrame(getDeviceRef(mxid), MultiDeviceFrame::housingFrame(housing), useSpecTranslation);
    matrix::invertSe3Matrix4x4InPlace(housingFromRig);
    scaleTranslationFromCentimetersInPlace(housingFromRig, unit);
    return housingFromRig;
}

const MultiDeviceCalibrationDevice& MultiDeviceCalibrationHandler::getDeviceRef(const std::string& mxid) const {
    const auto it = std::find_if(data.devices.begin(), data.devices.end(), [&mxid](const auto& device) { return device.mxid == mxid; });
    if(it == data.devices.end()) {
        throw std::runtime_error("Unknown device MXID '" + mxid + "'");
    }
    return *it;
}

void MultiDeviceCalibrationHandler::validateData(const MultiDeviceCalibrationData& data, bool throwOnError) {
    try {
        if(data.version != kSupportedMultiDeviceCalibrationVersion) {
            std::ostringstream message;
            message << "Unsupported MultiDeviceCalibrationData version " << data.version << ", expected " << kSupportedMultiDeviceCalibrationVersion;
            throw std::runtime_error(message.str());
        }

        std::unordered_set<std::string> mxids;
        for(const auto& device : data.devices) {
            if(device.mxid.empty()) {
                throw std::runtime_error("Device entry contains an empty MXID");
            }
            if(!mxids.insert(device.mxid).second) {
                throw std::runtime_error("Duplicate MXID '" + device.mxid + "'");
            }

            validateFrameState(device.anchorFrame, "anchor frame for device '" + device.mxid + "'");
            validateRotationMatrix(device.rigFromAnchor.rotationMatrix, "rigFromAnchor rotation for device '" + device.mxid + "'");
            validateFiniteTranslation(device.rigFromAnchor.translation, "rigFromAnchor for device '" + device.mxid + "'");

            CalibrationHandler calibration(device.calibration);
            if(!calibration.hasCalibrationData()) {
                throw std::runtime_error("Embedded calibration for device '" + device.mxid + "' has no supported camera calibration data");
            }

            try {
                calibration.validateCalibrationHandler();
            } catch(const std::exception& ex) {
                throw std::runtime_error("Embedded calibration for device '" + device.mxid + "' is invalid: " + std::string(ex.what()));
            }

            if(device.anchorFrame.type == MultiDeviceFrameType::CAMERA_SOCKET && !calibration.hasCameraCalibration(device.anchorFrame.cameraSocket)) {
                throw std::runtime_error("Anchor frame " + frameToString(device.anchorFrame) + " is absent from embedded calibration for device '" + device.mxid
                                         + "'");
            }

            try {
                if(device.anchorFrame.type == MultiDeviceFrameType::HOUSING) {
                    const auto referenceCamera = calibration.getCameraWithLowestId();
                    (void)calibration.getHousingCalibration(referenceCamera, device.anchorFrame.housing, false, LengthUnit::CENTIMETER);
                }
            } catch(const std::exception& ex) {
                throw std::runtime_error("Anchor frame " + frameToString(device.anchorFrame) + " is unreachable for device '" + device.mxid
                                         + "': " + std::string(ex.what()));
            }
        }
    } catch(const std::exception& ex) {
        if(throwOnError) {
            throw;
        }
        std::cout << "Warning: " << ex.what() << std::endl;
    }
}

void MultiDeviceCalibrationHandler::validateFrameState(const MultiDeviceFrame& frame, const std::string& context) {
    switch(frame.type) {
        case MultiDeviceFrameType::CAMERA_SOCKET:
            if(frame.cameraSocket == CameraBoardSocket::AUTO || frame.housing != HousingCoordinateSystem::AUTO) {
                throw std::runtime_error(context + " must use cameraSocket != AUTO and housing == AUTO for CAMERA_SOCKET");
            }
            return;
        case MultiDeviceFrameType::HOUSING:
            if(frame.housing == HousingCoordinateSystem::AUTO || frame.cameraSocket != CameraBoardSocket::AUTO) {
                throw std::runtime_error(context + " must use housing != AUTO and cameraSocket == AUTO for HOUSING");
            }
            return;
        default:
            throw std::runtime_error(context + " has an unknown frame type");
    }
}

std::string MultiDeviceCalibrationHandler::frameToString(const MultiDeviceFrame& frame) {
    std::ostringstream stream;
    switch(frame.type) {
        case MultiDeviceFrameType::CAMERA_SOCKET:
            stream << "CAMERA_SOCKET(" << toString(frame.cameraSocket) << ")";
            break;
        case MultiDeviceFrameType::HOUSING:
            stream << "HOUSING(" << toString(frame.housing) << ")";
            break;
        default:
            stream << "UNKNOWN";
            break;
    }
    return stream.str();
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::createIdentityTransform() {
    return {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::rigFromAnchorToCentimeterMatrix(const RigidTransform& transform) {
    const float scale = getDistanceUnitScale(LengthUnit::CENTIMETER, transform.translationUnit);
    const Point3f translationInCentimeters{
        transform.translation.x * scale,
        transform.translation.y * scale,
        transform.translation.z * scale,
    };
    return matrix::toVecMatrix4x4(matrix::createTransformationMatrix(transform.rotationMatrix, translationInCentimeters));
}

void MultiDeviceCalibrationHandler::scaleTranslationFromCentimetersInPlace(std::vector<std::vector<float>>& transform, LengthUnit unit) {
    const float scale = getDistanceUnitScale(unit, LengthUnit::CENTIMETER);
    if(scale == 1.0f) {
        return;
    }

    for(size_t row = 0; row < 3; ++row) {
        transform[row][3] *= scale;
    }
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getAnchorFromLocalFrame(const MultiDeviceCalibrationDevice& device,
                                                                                       const MultiDeviceFrame& frame,
                                                                                       bool useSpecTranslation) const {
    validateFrameState(frame, "query frame for device '" + device.mxid + "'");
    validateFrameState(device.anchorFrame, "anchor frame for device '" + device.mxid + "'");

    if(frame == device.anchorFrame) {
        return createIdentityTransform();
    }

    CalibrationHandler calibration(device.calibration);
    try {
        if(device.anchorFrame.type == MultiDeviceFrameType::CAMERA_SOCKET) {
            if(frame.type == MultiDeviceFrameType::CAMERA_SOCKET) {
                return calibration.getCameraExtrinsics(frame.cameraSocket, device.anchorFrame.cameraSocket, useSpecTranslation, LengthUnit::CENTIMETER);
            }

            auto anchorFromHousing =
                calibration.getHousingCalibration(device.anchorFrame.cameraSocket, frame.housing, useSpecTranslation, LengthUnit::CENTIMETER);
            matrix::invertSe3Matrix4x4InPlace(anchorFromHousing);
            return anchorFromHousing;
        }

        if(frame.type == MultiDeviceFrameType::CAMERA_SOCKET) {
            return calibration.getHousingCalibration(frame.cameraSocket, device.anchorFrame.housing, useSpecTranslation, LengthUnit::CENTIMETER);
        }

        const auto referenceCamera = calibration.getCameraWithLowestId();
        auto anchorFromReference = calibration.getHousingCalibration(referenceCamera, device.anchorFrame.housing, useSpecTranslation, LengthUnit::CENTIMETER);
        auto frameFromReference = calibration.getHousingCalibration(referenceCamera, frame.housing, useSpecTranslation, LengthUnit::CENTIMETER);
        matrix::invertSe3Matrix4x4InPlace(frameFromReference);
        return matrix::matMul(anchorFromReference, frameFromReference);
    } catch(const std::exception& ex) {
        throw std::runtime_error("failed to resolve " + frameToString(frame) + " relative to anchor " + frameToString(device.anchorFrame) + ": " + ex.what());
    }
}

std::vector<std::vector<float>> MultiDeviceCalibrationHandler::getRigFromLocalFrame(const MultiDeviceCalibrationDevice& device,
                                                                                    const MultiDeviceFrame& frame,
                                                                                    bool useSpecTranslation) const {
    auto rigFromAnchor = rigFromAnchorToCentimeterMatrix(device.rigFromAnchor);
    auto anchorFromFrame = getAnchorFromLocalFrame(device, frame, useSpecTranslation);
    return matrix::matMul(rigFromAnchor, anchorFromFrame);
}

}  // namespace dai
