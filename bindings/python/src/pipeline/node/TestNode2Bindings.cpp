#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/TestNode2.hpp"

void bind_testnode2(pybind11::module& m, void* pCallstack) {
    using namespace dai::node;

    auto testNode2 = ADD_NODE_DERIVED_DOC(TestNode2, ThreadedHostNode, "Host test node that mutates BatchItem ImgFrame payloads and forwards them.");

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    testNode2.def_readonly("input", &TestNode2::input).def_readonly("output", &TestNode2::output);
}
