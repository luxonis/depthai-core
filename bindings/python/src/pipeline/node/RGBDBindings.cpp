
#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"

#include <pybind11/eval.h>

extern py::handle daiNodeModule;

void bind_rgbd(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto rgbdNode = ADD_NODE_DERIVED(RGBD, ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // RGBD Node
    rgbdNode
        .def_property_readonly("inColor", [](RGBD& node){ return &node.inColor; }, py::return_value_policy::reference_internal)
        .def_property_readonly("inDepth", [](RGBD& node){ return &node.inColor; }, py::return_value_policy::reference_internal)
        .def_readonly("pcl", &RGBD::pcl, DOC(dai, node, RGBD, pcl))
        .def("build", &RGBD::build, DOC(dai, node, RGBD, build));
}
