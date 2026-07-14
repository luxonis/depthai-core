#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/BatchAssembler.hpp"

void bind_gather(pybind11::module& m, void* pCallstack) {
    using namespace dai::node;

    auto gather = ADD_NODE(BatchAssembler);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    gather.def_readonly("input", &BatchAssembler::input)
        .def_readonly("output", &BatchAssembler::output)
        .def("setRunOnHost", &BatchAssembler::setRunOnHost, pybind11::arg("runOnHost"));
}
