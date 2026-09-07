#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/AlignConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_alignconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<AlignConfig, Py<AlignConfig>, Buffer, std::shared_ptr<AlignConfig>> alignConfig(m, "AlignConfig", DOC(dai, AlignConfig));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Message
    alignConfig.def(py::init<>())
        .def("__repr__", &AlignConfig::str)
        .def_readwrite("staticDepthPlane", &AlignConfig::staticDepthPlane, DOC(dai, AlignConfig, staticDepthPlane));
}
