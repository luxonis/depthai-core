#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/common/MultiDeviceCalibrationData.hpp"
#include "depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp"

void bind_multi_device_calibration_result(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    namespace py = pybind11;
    using namespace pybind11::literals;

    py::class_<MultiDeviceCalibrationResult, Buffer, std::shared_ptr<MultiDeviceCalibrationResult>>(m, "MultiDeviceCalibrationResult")
        .def(py::init<>())
        .def(py::init<MultiDeviceCalibrationData, double, std::string>(), "calibration"_a, "dataConfidence"_a, "info"_a = "")
        .def_readwrite("calibration", &MultiDeviceCalibrationResult::calibration)
        .def_readwrite("dataConfidence", &MultiDeviceCalibrationResult::dataConfidence)
        .def_readwrite("passed", &MultiDeviceCalibrationResult::passed)
        .def_readwrite("info", &MultiDeviceCalibrationResult::info);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling (standard pattern for DepthAI bindings)
    ///////////////////////////////////////////////////////////////////////
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
