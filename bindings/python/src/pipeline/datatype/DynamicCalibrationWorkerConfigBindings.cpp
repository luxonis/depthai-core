#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/DynamicCalibrationWorkerConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_dynamic_calibration_worker_config(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    // 1. Define the main class
    py::class_<DynamicCalibrationWorkerConfig, Py<DynamicCalibrationWorkerConfig>, Buffer, std::shared_ptr<DynamicCalibrationWorkerConfig>> dynCalibConfig(
        m, "DynamicCalibrationWorkerConfig", DOC(dai, DynamicCalibrationWorkerConfig));

    // 2. Bind the nested Enum
    py::enum_<DynamicCalibrationWorkerConfig::Mode>(dynCalibConfig, "Mode", DOC(dai, DynamicCalibrationWorkerConfig, Mode))
        .value("ON_START", DynamicCalibrationWorkerConfig::Mode::ON_START)
        .value("CONTINUOUS", DynamicCalibrationWorkerConfig::Mode::CONTINUOUS)
        .export_values();

    // 3. Handle DepthAI callstack logic (Internal boilerplate)
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // 4. Define methods and properties
    dynCalibConfig.def(py::init<>())
        .def_readwrite("mode", &DynamicCalibrationWorkerConfig::mode, DOC(dai, DynamicCalibrationWorkerConfig, mode))
        .def_readwrite("sleepingTime", &DynamicCalibrationWorkerConfig::sleepingTime, DOC(dai, DynamicCalibrationWorkerConfig, sleepingTime))
        .def_readwrite("calibrationConfidenceThreshold",
                       &DynamicCalibrationWorkerConfig::calibrationConfidenceThreshold,
                       DOC(dai, DynamicCalibrationWorkerConfig, calibrationConfidenceThreshold))
        .def_readwrite("dataConfidenceThreshold",
                       &DynamicCalibrationWorkerConfig::dataConfidenceThreshold,
                       DOC(dai, DynamicCalibrationWorkerConfig, dataConfidenceThreshold))
        .def_readwrite("maxIterations", &DynamicCalibrationWorkerConfig::maxIterations, DOC(dai, DynamicCalibrationWorkerConfig, maxIterations))
        .def_readwrite("flashCalibration", &DynamicCalibrationWorkerConfig::flashCalibration, DOC(dai, DynamicCalibrationWorkerConfig, flashCalibration))
        .def_readwrite("validationSetSize", &DynamicCalibrationWorkerConfig::validationSetSize, DOC(dai, DynamicCalibrationWorkerConfig, validationSetSize))
        .def("__repr__", [](const DynamicCalibrationWorkerConfig& c) {
            return "<DynamicCalibrationWorkerConfig mode=" + std::to_string(static_cast<int>(c.mode)) + " sleepingTime=" + std::to_string(c.sleepingTime) + ">";
        });
}
