#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/AutoCalibrationResult.hpp"

void bind_auto_calibration_result(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    namespace py = pybind11;
    using namespace pybind11::literals;

    py::class_<AutoCalibrationResult, Buffer, std::shared_ptr<AutoCalibrationResult>>(m, "AutoCalibrationResult")
        .def(py::init<>())
        .def(py::init<double, double, bool, CalibrationHandler>(), "dataConfidence"_a, "calibrationConfidence"_a, "passed"_a, "calibration"_a)
        .def_readwrite("dataConfidence", &AutoCalibrationResult::dataConfidence)
        .def_readwrite("calibrationConfidence", &AutoCalibrationResult::calibrationConfidence)
        .def_readwrite("passed", &AutoCalibrationResult::passed)
        .def_readwrite("calibration", &AutoCalibrationResult::calibration);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling (standard pattern for DepthAI bindings)
    ///////////////////////////////////////////////////////////////////////
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
