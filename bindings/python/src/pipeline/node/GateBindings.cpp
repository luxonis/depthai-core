#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Gate.hpp"

void bind_gate(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    py::class_<GateProperties> gateProperties(m, "GateProperties", DOC(dai, GateProperties));
    auto gate = ADD_NODE(Gate);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    gate.def_readonly("input", &Gate::input, DOC(dai, node, Gate, input))

        .def_readonly("inputControl", &Gate::inputControl, DOC(dai, node, Gate, inputControl))

        .def("runOnHost", &Gate::runOnHost, DOC(dai, node, Gate, runOnHost))

        .def("setRunOnHost", &Gate::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, Gate, setRunOnHost))

        .def_readonly("output", &Gate::output, DOC(dai, node, Gate, output));

    daiNodeModule.attr("Gate").attr("Properties") = gateProperties;
}
