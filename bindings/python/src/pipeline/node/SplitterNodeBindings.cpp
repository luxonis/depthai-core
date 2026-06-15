#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SplitterNode.hpp"

void bind_splitternode(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<SplitterNodeProperties> splitterNodeProperties(m, "SplitterNodeProperties", DOC(dai, SplitterNodeProperties));
    auto splitterNode = ADD_NODE(SplitterNode);

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

    splitterNode.def_readonly("input", &SplitterNode::input, DOC(dai, node, SplitterNode, input))
        .def_readonly("output", &SplitterNode::output, DOC(dai, node, SplitterNode, output))
        .def("setRunOnHost", &SplitterNode::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, SplitterNode, setRunOnHost))
        .def("runOnHost", &SplitterNode::runOnHost, DOC(dai, node, SplitterNode, runOnHost));

    daiNodeModule.attr("SplitterNode").attr("Properties") = splitterNodeProperties;
}
