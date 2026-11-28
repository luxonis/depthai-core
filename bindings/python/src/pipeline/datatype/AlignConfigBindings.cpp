#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/AlignConfig.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_alignconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<AlignConfig, Py<AlignConfig>, Buffer, py::smart_holder> alignConfig(m, "AlignConfig", DOC(dai, AlignConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    alignConfig.def(py::init<>())
        .def("__repr__", &AlignConfig::str)
        .def_readwrite("staticDepthPlane", &AlignConfig::staticDepthPlane, DOC(dai, AlignConfig, staticDepthPlane));
}
