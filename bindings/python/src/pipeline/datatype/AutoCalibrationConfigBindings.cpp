#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/AutoCalibrationConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_auto_calibration_config(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    // 1. Define the main class
    py::class_<AutoCalibrationConfig, Py<AutoCalibrationConfig>, Buffer, std::shared_ptr<AutoCalibrationConfig>> dynCalibConfig(
        m, "AutoCalibrationConfig", DOC(dai, AutoCalibrationConfig));

    // 2. Bind the nested Enum
    py::enum_<AutoCalibrationConfig::Mode>(dynCalibConfig, "Mode", DOC(dai, AutoCalibrationConfig, Mode))
        .value("ON_START", AutoCalibrationConfig::Mode::ON_START)
        .value("CONTINUOUS", AutoCalibrationConfig::Mode::CONTINUOUS)
        .export_values();

    // 3. Handle DepthAI callstack logic (Internal boilerplate)
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // 4. Define methods and properties
    dynCalibConfig.def(py::init<>())
        .def_readwrite("mode", &AutoCalibrationConfig::mode, DOC(dai, AutoCalibrationConfig, mode))
        .def_readwrite("sleepingTime", &AutoCalibrationConfig::sleepingTime, DOC(dai, AutoCalibrationConfig, sleepingTime))
        .def_readwrite("calibrationConfidenceThreshold",
                       &AutoCalibrationConfig::calibrationConfidenceThreshold,
                       DOC(dai, AutoCalibrationConfig, calibrationConfidenceThreshold))
        .def_readwrite("dataConfidenceThreshold", &AutoCalibrationConfig::dataConfidenceThreshold, DOC(dai, AutoCalibrationConfig, dataConfidenceThreshold))
        .def_readwrite("maxIterations", &AutoCalibrationConfig::maxIterations, DOC(dai, AutoCalibrationConfig, maxIterations))
        .def_readwrite(
            "maxImagesPerRecalibration", &AutoCalibrationConfig::maxImagesPerRecalibration, DOC(dai, AutoCalibrationConfig, maxImagesPerRecalibration))
        .def_readwrite("flashCalibration", &AutoCalibrationConfig::flashCalibration, DOC(dai, AutoCalibrationConfig, flashCalibration))
        .def_readwrite("validationSetSize", &AutoCalibrationConfig::validationSetSize, DOC(dai, AutoCalibrationConfig, validationSetSize))
        .def("__repr__", [](const AutoCalibrationConfig& c) {
            return "<AutoCalibrationConfig mode=" + std::to_string(static_cast<int>(c.mode)) + " sleepingTime=" + std::to_string(c.sleepingTime) + ">";
        });
}
