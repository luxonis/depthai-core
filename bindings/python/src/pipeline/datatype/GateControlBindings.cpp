
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
        .def(py::init<bool, int, int>(), "open"_a, "numMessages"_a, "fps"_a)
        // Member variables
        .def_readwrite("open", &GateControl::open, DOC(dai, GateControl, open))
        .def_readwrite("numMessages", &GateControl::numMessages, DOC(dai, GateControl, numMessages))
        .def_readwrite("fps", &GateControl::fps, DOC(dai, GateControl, fps))
        // Static factory methods
        .def_static("openGate", py::overload_cast<int, int>(&GateControl::openGate), "numMessages"_a, "fps"_a = -1, DOC(dai, GateControl, openGate))
        .def_static("openGate", py::overload_cast<>(&GateControl::openGate), DOC(dai, GateControl, openGate, 2))
        .def_static("closeGate", &GateControl::closeGate, DOC(dai, GateControl, closeGate))
        // Metadata helper
        .def("getDatatype", &GateControl::getDatatype, DOC(dai, GateControl, getDatatype));
}
