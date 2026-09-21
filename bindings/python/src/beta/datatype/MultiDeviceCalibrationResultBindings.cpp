#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/MultiDeviceCalibrationResult.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_multidevicecalibrationresult(pybind11::module& m, void* pCallstack) {
    namespace py = pybind11;
    using namespace dai;
    using Result = beta::MultiDeviceCalibrationResult;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<Result, Buffer, std::shared_ptr<Result>>(betaModule, "MultiDeviceCalibrationResult", DOC(dai, beta, MultiDeviceCalibrationResult))
        .def(py::init<>())
        .def(py::init<std::string>(), py::arg("info"))
        .def_readwrite("handler", &Result::handler)
        .def_readwrite("passed", &Result::passed)
        .def_readwrite("dataConfidence", &Result::dataConfidence)
        .def_readwrite("sampsonError", &Result::sampsonError)
        .def_readwrite("info", &Result::info);

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
