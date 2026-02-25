#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationWorkerResult.hpp"

void bind_dynamic_calibration_worker_result(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    namespace py = pybind11;
    using namespace pybind11::literals;

    py::class_<DynamicCalibrationWorkerResult, Buffer, std::shared_ptr<DynamicCalibrationWorkerResult>>(m, "DynamicCalibrationWorkerResult")
        .def(py::init<>())
        .def(py::init<double, double, bool, CalibrationHandler>(), "dataQuality"_a, "calibrationConfidence"_a, "passed"_a, "calibration"_a)
        .def_readwrite("dataQuality", &DynamicCalibrationWorkerResult::dataQuality)
        .def_readwrite("calibrationConfidence", &DynamicCalibrationWorkerResult::calibrationConfidence)
        .def_readwrite("passed", &DynamicCalibrationWorkerResult::passed)
        .def_readwrite("calibration", &DynamicCalibrationWorkerResult::calibration);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling (standard pattern for DepthAI bindings)
    ///////////////////////////////////////////////////////////////////////
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
