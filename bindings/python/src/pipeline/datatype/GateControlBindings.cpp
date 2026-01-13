#include <unordered_map>
#include <vector>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/GateControl.hpp"

void bind_gate_control(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    py::class_<GateControl, Buffer, std::shared_ptr<GateControl>> gateControl(m, "GateControl", DOC(dai, GateControl));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    gateControl.def(py::init<>())
        .def_readwrite("value", &GateControl::value, DOC(dai, GateControl, value))
        .def("start", &GateControl::start, DOC(dai, GateControl, start))
        .def("stop", &GateControl::stop, DOC(dai, GateControl, stop));
}
