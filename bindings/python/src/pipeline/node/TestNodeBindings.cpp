#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/TestNode.hpp"

void bind_testnode(pybind11::module& m, void* pCallstack) {
    using namespace dai::node;

    auto testNode = ADD_NODE(TestNode);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    testNode.def_readonly("input", &TestNode::input)
        .def_readonly("output", &TestNode::output)
        .def("setRunOnHost", &TestNode::setRunOnHost, pybind11::arg("runOnHost"));
}
