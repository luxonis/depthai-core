#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_multi_device_calibration_result(pybind11::module& m, void* pCallstack) {
    namespace py = pybind11;
    using namespace dai;
    using Diagnostic = MultiDeviceCalibrationResult::EdgeDiagnostic;

    py::class_<Diagnostic>(m, "MultiDeviceCalibrationEdgeDiagnostic")
        .def(py::init<>())
        .def_readwrite("fromDeviceId", &Diagnostic::fromDeviceId)
        .def_readwrite("fromSocket", &Diagnostic::fromSocket)
        .def_readwrite("toDeviceId", &Diagnostic::toDeviceId)
        .def_readwrite("toSocket", &Diagnostic::toSocket)
        .def_readwrite("accepted", &Diagnostic::accepted)
        .def_readwrite("dclConfidence", &Diagnostic::dclConfidence)
        .def_readwrite("reprojectionError", &Diagnostic::reprojectionError)
        .def_readwrite("sampsonError", &Diagnostic::sampsonError)
        .def_readwrite("scaleSource", &Diagnostic::scaleSource)
        .def_readwrite("scaleResidual", &Diagnostic::scaleResidual)
        .def_readwrite("rejectionReason", &Diagnostic::rejectionReason);

    py::class_<MultiDeviceCalibrationResult, Buffer, std::shared_ptr<MultiDeviceCalibrationResult>>(m, "MultiDeviceCalibrationResult")
        .def(py::init<>())
        .def(py::init<std::string>(), py::arg("info"))
        .def_readwrite("handler", &MultiDeviceCalibrationResult::handler)
        .def_readwrite("passed", &MultiDeviceCalibrationResult::passed)
        .def_readwrite("complete", &MultiDeviceCalibrationResult::complete)
        .def_readwrite("dataConfidence", &MultiDeviceCalibrationResult::dataConfidence)
        .def_readwrite("info", &MultiDeviceCalibrationResult::info)
        .def_readwrite("diagnostics", &MultiDeviceCalibrationResult::diagnostics);

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
