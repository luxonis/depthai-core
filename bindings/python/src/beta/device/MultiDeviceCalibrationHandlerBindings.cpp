#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <filesystem>
#include <vector>

#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "pybind11_common.hpp"

void bind_beta_multidevicecalibrationhandler(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using dai::beta::MultiDeviceCalibrationHandler;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<MultiDeviceCalibrationHandler> multiDeviceCalibrationHandler(
        betaModule, "MultiDeviceCalibrationHandler", DOC(dai, beta, MultiDeviceCalibrationHandler));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    multiDeviceCalibrationHandler.def(py::init<>())
        .def(py::init<std::vector<MultiDeviceExtrinsics>>(), py::arg("graph"))
        .def(py::init<std::filesystem::path>(), py::arg("calibrationDataPath"), DOC(dai, beta, MultiDeviceCalibrationHandler, MultiDeviceCalibrationHandler, 3))
        .def_static(
            "fromJson", &MultiDeviceCalibrationHandler::fromJson, py::arg("calibrationDataJson"), DOC(dai, beta, MultiDeviceCalibrationHandler, fromJson))
        .def("toJson", &MultiDeviceCalibrationHandler::toJson, DOC(dai, beta, MultiDeviceCalibrationHandler, toJson))
        .def("toJsonFile", &MultiDeviceCalibrationHandler::toJsonFile, py::arg("destPath"), DOC(dai, beta, MultiDeviceCalibrationHandler, toJsonFile))
        .def("getGraph", &MultiDeviceCalibrationHandler::getGraph, DOC(dai, beta, MultiDeviceCalibrationHandler, getGraph))
        .def("getDeviceSocket",
             &MultiDeviceCalibrationHandler::getDeviceSocket,
             py::arg("deviceId"),
             DOC(dai, beta, MultiDeviceCalibrationHandler, getDeviceSocket))
        .def("getExtrinsicsToOrigin",
             &MultiDeviceCalibrationHandler::getExtrinsicsToOrigin,
             py::arg("deviceId"),
             py::arg("localOriginSocket"),
             DOC(dai, beta, MultiDeviceCalibrationHandler, getExtrinsicsToOrigin));
}
