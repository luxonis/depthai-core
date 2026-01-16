
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
        .def(py::init<bool, int>(), "open"_a, "numMessages"_a)
        // Member variables
        .def_readwrite("open", &GateControl::open, DOC(dai, GateControl, open))
        .def_readwrite("numMessages", &GateControl::numMessages, DOC(dai, GateControl, numMessages))
        // Static factory methods
        .def_static("openGate", py::overload_cast<int>(&GateControl::openGate), "numMessages"_a, DOC(dai, GateControl, openGate))
        .def_static("openGate", py::overload_cast<>(&GateControl::openGate), DOC(dai, GateControl, openGate, 2))
        .def_static("closeGate", &GateControl::closeGate, DOC(dai, GateControl, closeGate))
        // Metadata helper
        .def("getDatatype", &GateControl::getDatatype, DOC(dai, GateControl, getDatatype));
}
