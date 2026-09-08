#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_multi_device_calibration_result(pybind11::module& m, void* pCallstack) {
    namespace py = pybind11;
    using namespace dai;
    py::class_<MultiDeviceCalibrationResult, Buffer, std::shared_ptr<MultiDeviceCalibrationResult>>(m, "MultiDeviceCalibrationResult")
        .def(py::init<>())
        .def(py::init<std::string>(), py::arg("info"))
        .def_readwrite("handler", &MultiDeviceCalibrationResult::handler)
        .def_readwrite("passed", &MultiDeviceCalibrationResult::passed)
        .def_readwrite("dataConfidence", &MultiDeviceCalibrationResult::dataConfidence)
        .def_readwrite("sampsonError", &MultiDeviceCalibrationResult::sampsonError)
        .def_readwrite("info", &MultiDeviceCalibrationResult::info);

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
