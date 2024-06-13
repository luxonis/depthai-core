#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/basalt/BasaltVIO.hpp"

#include <pybind11/eval.h>

extern py::handle daiNodeModule;



void bind_basaltnode(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto basaltNode = ADD_NODE_DERIVED(BasaltVIO, ThreadedHostNode);

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

    // BasaltVIO Node
    basaltNode
        .def_property_readonly("left", [](BasaltVIO& node){ return node.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly("right", [](BasaltVIO& node){ return node.right; }, py::return_value_policy::reference_internal)
        .def_readonly("imu", &BasaltVIO::imu, DOC(dai, node, BasaltVIO, imu))
        .def_readonly("transform", &BasaltVIO::transform, DOC(dai, node, BasaltVIO, transform))
        .def_readonly("passthrough", &BasaltVIO::passthrough, DOC(dai, node, BasaltVIO, passthrough))
        .def("build", &BasaltVIO::build, DOC(dai, node, BasaltVIO, build))
        .def("setImuUpdateRate", &BasaltVIO::setImuUpdateRate, py::arg("rate"), DOC(dai, node, BasaltVIO, setImuUpdateRate))
        .def("setConfigPath", &BasaltVIO::setConfigPath, py::arg("path"), DOC(dai, node, BasaltVIO, setConfigPath))
        .def("setLocalTransform", &BasaltVIO::setLocalTransform, py::arg("transform"), DOC(dai, node, BasaltVIO, setLocalTransform));
}