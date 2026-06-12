#include "CalibrationHandlerBindings.hpp"

#include <optional>
#include <vector>

#include "depthai/common/Point2f.hpp"
#include "depthai/device/CalibrationHandler.hpp"

void CalibrationHandlerBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::class_<CalibrationHandler> calibrationHandler(m, "CalibrationHandler", DOC(dai, CalibrationHandler));
    py::class_<CBACalibrationHandler> cbaCalibrationHandler(m, "CBACalibrationHandler", DOC(dai, CBACalibrationHandler));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Bindings
    calibrationHandler.def(py::init<>(), DOC(dai, CalibrationHandler, CalibrationHandler))
        .def(py::init<std::filesystem::path, bool>(),
             py::arg("eepromDataPath"),
             py::arg("validateExtrinsics") = std::nullopt,
             DOC(dai, CalibrationHandler, CalibrationHandler, 2))
        .def(py::init<std::filesystem::path, std::filesystem::path, bool>(),
             py::arg("calibrationDataPath"),
             py::arg("boardConfigPath"),
             py::arg("validateExtrinsics") = std::nullopt,
             DOC(dai, CalibrationHandler, CalibrationHandler, 3))
        .def(py::init<EepromData, bool>(),
             py::arg("eepromData"),
             py::arg("validateExtrinsics") = std::nullopt,
             DOC(dai, CalibrationHandler, CalibrationHandler, 4))

        .def_static("fromJson",
                    &CalibrationHandler::fromJson,
                    py::arg("eepromDataJson"),
                    py::arg("validateExtrinsics") = std::nullopt,
                    DOC(dai, CalibrationHandler, fromJson))

        .def("getEepromData", &CalibrationHandler::getEepromData, DOC(dai, CalibrationHandler, getEepromData))
        .def("hasCalibrationData", &CalibrationHandler::hasCalibrationData, DOC(dai, CalibrationHandler, hasCalibrationData))
        .def("hasCameraCalibration", &CalibrationHandler::hasCameraCalibration, py::arg("cameraId"), DOC(dai, CalibrationHandler, hasCameraCalibration))

        .def("getCameraIntrinsics",
             py::overload_cast<CameraBoardSocket, int, int, Point2f, Point2f, bool>(&CalibrationHandler::getCameraIntrinsics, py::const_),
             py::arg("cameraId"),
             py::arg("resizeWidth") = -1,
             py::arg("resizeHeight") = -1,
             py::arg("topLeftPixelId") = Point2f(),
             py::arg("bottomRightPixelId") = Point2f(),
             py::arg("keepAspectRatio") = true,
             DOC(dai, CalibrationHandler, getCameraIntrinsics))
        .def("getCameraIntrinsics",
             py::overload_cast<CameraBoardSocket, Size2f, Point2f, Point2f, bool>(&CalibrationHandler::getCameraIntrinsics, py::const_),
             py::arg("cameraId"),
             py::arg("destShape"),
             py::arg("topLeftPixelId") = Point2f(),
             py::arg("bottomRightPixelId") = Point2f(),
             py::arg("keepAspectRatio") = true,
             DOC(dai, CalibrationHandler, getCameraIntrinsics, 2))
        .def("getCameraIntrinsics",
             py::overload_cast<CameraBoardSocket, std::tuple<int, int>, Point2f, Point2f, bool>(&CalibrationHandler::getCameraIntrinsics, py::const_),
             py::arg("cameraId"),
             py::arg("destShape"),
             py::arg("topLeftPixelId") = Point2f(),
             py::arg("bottomRightPixelId") = Point2f(),
             py::arg("keepAspectRatio") = true,
             DOC(dai, CalibrationHandler, getCameraIntrinsics, 3))

        .def("getDefaultIntrinsics", &CalibrationHandler::getDefaultIntrinsics, py::arg("cameraId"), DOC(dai, CalibrationHandler, getDefaultIntrinsics))
        .def("getSourceHeight", &CalibrationHandler::getSourceHeight, py::arg("cameraId"), DOC(dai, CalibrationHandler, getSourceHeight))
        .def("getSourceWidth", &CalibrationHandler::getSourceWidth, py::arg("cameraId"), DOC(dai, CalibrationHandler, getSourceWidth))
        .def("getDistortionCoefficients",
             &CalibrationHandler::getDistortionCoefficients,
             py::arg("cameraId"),
             DOC(dai, CalibrationHandler, getDistortionCoefficients))

        .def("getFov", &CalibrationHandler::getFov, py::arg("cameraId"), py::arg("useSpec") = true, DOC(dai, CalibrationHandler, getFov))
        .def("getLensPosition", &CalibrationHandler::getLensPosition, py::arg("cameraId"), DOC(dai, CalibrationHandler, getLensPosition))
        .def("getDistortionModel", &CalibrationHandler::getDistortionModel, py::arg("cameraId"), DOC(dai, CalibrationHandler, getDistortionModel))
        .def("getCameraWithLowestId", &CalibrationHandler::getCameraWithLowestId, DOC(dai, CalibrationHandler, getCameraWithLowestId))
        .def("getCameraExtrinsics",
             &CalibrationHandler::getCameraExtrinsics,
             py::arg("srcCamera"),
             py::arg("dstCamera"),
             py::arg("useSpecTranslation") = false,
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, CalibrationHandler, getCameraExtrinsics))

        .def("getHousingCalibration",
             &CalibrationHandler::getHousingCalibration,
             py::arg("srcCamera"),
             py::arg("housingCS"),
             py::arg("useSpecTranslation") = true,
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, CalibrationHandler, getHousingCalibration))

        .def("getCameraTranslationVector",
             &CalibrationHandler::getCameraTranslationVector,
             py::arg("srcCamera"),
             py::arg("dstCamera"),
             py::arg("useSpecTranslation") = true,
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, CalibrationHandler, getCameraTranslationVector))
        .def("getCameraRotationMatrix",
             &CalibrationHandler::getCameraRotationMatrix,
             py::arg("srcCamera"),
             py::arg("dstCamera"),
             DOC(dai, CalibrationHandler, getCameraRotationMatrix))
        .def("getBaselineDistance",
             &CalibrationHandler::getBaselineDistance,
             py::arg("cam1") = dai::CameraBoardSocket::CAM_C,
             py::arg("cam2") = dai::CameraBoardSocket::CAM_B,
             py::arg("useSpecTranslation") = true,
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, CalibrationHandler, getBaselineDistance))

        .def("getCameraToImuExtrinsics",
             &CalibrationHandler::getCameraToImuExtrinsics,
             py::arg("cameraId"),
             py::arg("useSpecTranslation") = false,
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, CalibrationHandler, getCameraToImuExtrinsics))
        .def("getImuToCameraExtrinsics",
             &CalibrationHandler::getImuToCameraExtrinsics,
             py::arg("cameraId"),
             py::arg("useSpecTranslation") = false,
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, CalibrationHandler, getImuToCameraExtrinsics))

        .def("getStereoRightRectificationRotation",
             &CalibrationHandler::getStereoRightRectificationRotation,
             DOC(dai, CalibrationHandler, getStereoRightRectificationRotation))
        .def("getStereoLeftRectificationRotation",
             &CalibrationHandler::getStereoLeftRectificationRotation,
             DOC(dai, CalibrationHandler, getStereoLeftRectificationRotation))
        .def("getStereoLeftCameraId", &CalibrationHandler::getStereoLeftCameraId, DOC(dai, CalibrationHandler, getStereoLeftCameraId))
        .def("getStereoRightCameraId", &CalibrationHandler::getStereoRightCameraId, DOC(dai, CalibrationHandler, getStereoRightCameraId))
        .def("getAccelerometerCalibration", &CalibrationHandler::getAccelerometerCalibration, DOC(dai, CalibrationHandler, getAccelerometerCalibration))
        .def("getGyroscopeCalibration", &CalibrationHandler::getGyroscopeCalibration, DOC(dai, CalibrationHandler, getGyroscopeCalibration))
        .def("getImuNoiseParameters", &CalibrationHandler::getImuNoiseParameters, DOC(dai, CalibrationHandler, getImuNoiseParameters))
        .def("getImuParameters", &CalibrationHandler::getImuParameters, DOC(dai, CalibrationHandler, getImuParameters))

        .def("eepromToJsonFile", &CalibrationHandler::eepromToJsonFile, py::arg("destPath"), DOC(dai, CalibrationHandler, eepromToJsonFile))
        .def("eepromToJson", &CalibrationHandler::eepromToJson, DOC(dai, CalibrationHandler, eepromToJson))

        .def("setBoardInfo",
             py::overload_cast<std::string, std::string>(&CalibrationHandler::setBoardInfo),
             py::arg("boardName"),
             py::arg("boardRev"),
             DOC(dai, CalibrationHandler, setBoardInfo))
        .def("setBoardInfo",
             py::overload_cast<std::string, std::string, std::string, std::string, std::string, std::string, uint64_t, uint32_t, std::string>(
                 &CalibrationHandler::setBoardInfo),
             py::arg("productName"),
             py::arg("boardName"),
             py::arg("boardRev"),
             py::arg("boardConf"),
             py::arg("hardwareConf"),
             py::arg("batchName"),
             py::arg("batchTime"),
             py::arg("boardOptions"),
             py::arg("boardCustom") = "",
             DOC(dai, CalibrationHandler, setBoardInfo, 2))
        .def("setBoardInfo",
             py::overload_cast<std::string, std::string, std::string, std::string, std::string, std::string, std::string, uint64_t, uint32_t, std::string>(
                 &CalibrationHandler::setBoardInfo),
             py::arg("deviceName"),
             py::arg("productName"),
             py::arg("boardName"),
             py::arg("boardRev"),
             py::arg("boardConf"),
             py::arg("hardwareConf"),
             py::arg("batchName"),
             py::arg("batchTime"),
             py::arg("boardOptions"),
             py::arg("boardCustom") = "",
             DOC(dai, CalibrationHandler, setBoardInfo, 3))

        .def("setDeviceName", &CalibrationHandler::setDeviceName, py::arg("deviceName"), DOC(dai, CalibrationHandler, setDeviceName))
        .def("setProductName", &CalibrationHandler::setProductName, py::arg("productName"), DOC(dai, CalibrationHandler, setProductName))

        .def("setCameraIntrinsics",
             py::overload_cast<CameraBoardSocket, std::vector<std::vector<float>>, Size2f>(&CalibrationHandler::setCameraIntrinsics),
             py::arg("cameraId"),
             py::arg("intrinsics"),
             py::arg("frameSize"),
             DOC(dai, CalibrationHandler, setCameraIntrinsics))
        .def("setCameraIntrinsics",
             py::overload_cast<CameraBoardSocket, std::vector<std::vector<float>>, int, int>(&CalibrationHandler::setCameraIntrinsics),
             py::arg("cameraId"),
             py::arg("intrinsics"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, CalibrationHandler, setCameraIntrinsics, 2))
        .def("setCameraIntrinsics",
             py::overload_cast<CameraBoardSocket, std::vector<std::vector<float>>, std::tuple<int, int>>(&CalibrationHandler::setCameraIntrinsics),
             py::arg("cameraId"),
             py::arg("intrinsics"),
             py::arg("frameSize"),
             DOC(dai, CalibrationHandler, setCameraIntrinsics, 3))

        .def("setDistortionCoefficients",
             &CalibrationHandler::setDistortionCoefficients,
             py::arg("cameraId"),
             py::arg("distortionCoefficients"),
             DOC(dai, CalibrationHandler, setDistortionCoefficients))
        .def("setFov", &CalibrationHandler::setFov, py::arg("cameraId"), py::arg("hfov"), DOC(dai, CalibrationHandler, setFov))

        .def("setLensPosition",
             &CalibrationHandler::setLensPosition,
             py::arg("cameraId"),
             py::arg("lensPosition"),
             DOC(dai, CalibrationHandler, setLensPosition))
        .def("setCameraType", &CalibrationHandler::setCameraType, py::arg("cameraId"), py::arg("cameraModel"), DOC(dai, CalibrationHandler, setCameraType))

        .def("setCameraExtrinsics",
             &CalibrationHandler::setCameraExtrinsics,
             py::arg("srcCameraId"),
             py::arg("destCameraId"),
             py::arg("rotationMatrix"),
             py::arg("translation"),
             py::arg("specTranslation") = std::vector<float>(3, 0),
             DOC(dai, CalibrationHandler, setCameraExtrinsics))
        .def("setImuExtrinsics",
             &CalibrationHandler::setImuExtrinsics,
             py::arg("destCameraId"),
             py::arg("rotationMatrix"),
             py::arg("translation"),
             py::arg("specTranslation") = std::vector<float>(3, 0),
             DOC(dai, CalibrationHandler, setImuExtrinsics))

        .def(
            "setStereoLeft", &CalibrationHandler::setStereoLeft, py::arg("cameraId"), py::arg("rectifiedRotation"), DOC(dai, CalibrationHandler, setStereoLeft))
        .def("setStereoRight",
             &CalibrationHandler::setStereoRight,
             py::arg("cameraId"),
             py::arg("rectifiedRotation"),
             DOC(dai, CalibrationHandler, setStereoRight))
        .def("setAccelerometerCalibration",
             &CalibrationHandler::setAccelerometerCalibration,
             py::arg("calibration"),
             DOC(dai, CalibrationHandler, setAccelerometerCalibration))
        .def("setGyroscopeCalibration",
             &CalibrationHandler::setGyroscopeCalibration,
             py::arg("calibration"),
             DOC(dai, CalibrationHandler, setGyroscopeCalibration))
        .def("setImuParameters", &CalibrationHandler::setImuParameters, py::arg("imuParameters"), DOC(dai, CalibrationHandler, setImuParameters))
        .def("validateCalibrationHandler",
             &CalibrationHandler::validateCalibrationHandler,
             py::arg("throwOnError") = true,
             DOC(dai, CalibrationHandler, validateCalibrationHandler));

    cbaCalibrationHandler.def(py::init<>(), DOC(dai, CBACalibrationHandler, CBACalibrationHandler))
        .def(py::init<EepromData, std::optional<bool>>(),
             py::arg("eepromData"),
             py::arg("validateExtrinsics") = std::nullopt,
             DOC(dai, CBACalibrationHandler, CBACalibrationHandler, 2))
        .def_static("fromJson",
                    &CBACalibrationHandler::fromJson,
                    py::arg("eepromDataJson"),
                    py::arg("validateExtrinsics") = std::nullopt,
                    DOC(dai, CBACalibrationHandler, fromJson))
        .def(
            "getEepromData", [](const CBACalibrationHandler& self) { return self.getEepromData(); }, DOC(dai, CalibrationHandler, getEepromData))
        .def(
            "hasCalibrationData", [](const CBACalibrationHandler& self) { return self.hasCalibrationData(); }, DOC(dai, CalibrationHandler, hasCalibrationData))
        .def(
            "eepromToJsonFile",
            [](const CBACalibrationHandler& self, std::filesystem::path destPath) { return self.eepromToJsonFile(destPath); },
            py::arg("destPath"),
            DOC(dai, CalibrationHandler, eepromToJsonFile))
        .def(
            "eepromToJson", [](const CBACalibrationHandler& self) { return self.eepromToJson(); }, DOC(dai, CalibrationHandler, eepromToJson))
        .def(
            "validateCalibrationHandler",
            [](const CBACalibrationHandler& self, bool throwOnError) { self.validateCalibrationHandler(throwOnError); },
            py::arg("throwOnError") = true,
            DOC(dai, CalibrationHandler, validateCalibrationHandler))
        .def("hasCameraCalibration",
             py::overload_cast<>(&CBACalibrationHandler::hasCameraCalibration, py::const_),
             DOC(dai, CBACalibrationHandler, hasCameraCalibration))

        .def("getCameraIntrinsics",
             py::overload_cast<int, int, Point2f, Point2f, bool>(&CBACalibrationHandler::getCameraIntrinsics, py::const_),
             py::arg("resizeWidth") = -1,
             py::arg("resizeHeight") = -1,
             py::arg("topLeftPixelId") = Point2f(),
             py::arg("bottomRightPixelId") = Point2f(),
             py::arg("keepAspectRatio") = true,
             DOC(dai, CBACalibrationHandler, getCameraIntrinsics))
        .def("getCameraIntrinsics",
             py::overload_cast<Size2f, Point2f, Point2f, bool>(&CBACalibrationHandler::getCameraIntrinsics, py::const_),
             py::arg("destShape"),
             py::arg("topLeftPixelId") = Point2f(),
             py::arg("bottomRightPixelId") = Point2f(),
             py::arg("keepAspectRatio") = true,
             DOC(dai, CBACalibrationHandler, getCameraIntrinsics, 2))
        .def("getCameraIntrinsics",
             py::overload_cast<std::tuple<int, int>, Point2f, Point2f, bool>(&CBACalibrationHandler::getCameraIntrinsics, py::const_),
             py::arg("destShape"),
             py::arg("topLeftPixelId") = Point2f(),
             py::arg("bottomRightPixelId") = Point2f(),
             py::arg("keepAspectRatio") = true,
             DOC(dai, CBACalibrationHandler, getCameraIntrinsics, 3))
        .def("getDefaultIntrinsics",
             py::overload_cast<>(&CBACalibrationHandler::getDefaultIntrinsics, py::const_),
             DOC(dai, CBACalibrationHandler, getDefaultIntrinsics))
        .def("getSourceHeight", py::overload_cast<>(&CBACalibrationHandler::getSourceHeight, py::const_), DOC(dai, CBACalibrationHandler, getSourceHeight))
        .def("getSourceWidth", py::overload_cast<>(&CBACalibrationHandler::getSourceWidth, py::const_), DOC(dai, CBACalibrationHandler, getSourceWidth))
        .def("getDistortionCoefficients",
             py::overload_cast<>(&CBACalibrationHandler::getDistortionCoefficients, py::const_),
             DOC(dai, CBACalibrationHandler, getDistortionCoefficients))
        .def("getFov", py::overload_cast<bool>(&CBACalibrationHandler::getFov, py::const_), py::arg("useSpec") = true, DOC(dai, CBACalibrationHandler, getFov))
        .def("getLensPosition", py::overload_cast<>(&CBACalibrationHandler::getLensPosition, py::const_), DOC(dai, CBACalibrationHandler, getLensPosition))
        .def("getDistortionModel",
             py::overload_cast<>(&CBACalibrationHandler::getDistortionModel, py::const_),
             DOC(dai, CBACalibrationHandler, getDistortionModel))

        .def("setCameraIntrinsics",
             py::overload_cast<std::vector<std::vector<float>>, Size2f>(&CBACalibrationHandler::setCameraIntrinsics),
             py::arg("intrinsics"),
             py::arg("frameSize"),
             DOC(dai, CBACalibrationHandler, setCameraIntrinsics))
        .def("setCameraIntrinsics",
             py::overload_cast<std::vector<std::vector<float>>, int, int>(&CBACalibrationHandler::setCameraIntrinsics),
             py::arg("intrinsics"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, CBACalibrationHandler, setCameraIntrinsics, 2))
        .def("setCameraIntrinsics",
             py::overload_cast<std::vector<std::vector<float>>, std::tuple<int, int>>(&CBACalibrationHandler::setCameraIntrinsics),
             py::arg("intrinsics"),
             py::arg("frameSize"),
             DOC(dai, CBACalibrationHandler, setCameraIntrinsics, 3))
        .def("setDistortionCoefficients",
             py::overload_cast<std::vector<float>>(&CBACalibrationHandler::setDistortionCoefficients),
             py::arg("distortionCoefficients"),
             DOC(dai, CBACalibrationHandler, setDistortionCoefficients))
        .def("setFov", py::overload_cast<float>(&CBACalibrationHandler::setFov), py::arg("hfov"), DOC(dai, CBACalibrationHandler, setFov))
        .def("setLensPosition",
             py::overload_cast<uint8_t>(&CBACalibrationHandler::setLensPosition),
             py::arg("lensPosition"),
             DOC(dai, CBACalibrationHandler, setLensPosition))
        .def("setCameraType",
             py::overload_cast<CameraModel>(&CBACalibrationHandler::setCameraType),
             py::arg("cameraModel"),
             DOC(dai, CBACalibrationHandler, setCameraType));
}
