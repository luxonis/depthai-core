
#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/Rectification.hpp"

void bind_rectification(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<RectificationProperties> RectificationProperties(m, "RectificationProperties", DOC(dai, RectificationProperties));

    // Declare node upfront
    auto rectification = ADD_NODE(Rectification);

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

    // Properties
    RectificationProperties.def_readwrite("enableSync", &RectificationProperties::enableSync, DOC(dai, RectificationProperties, enableSync));

    // Nodes
    rectification.def_readonly("passthrough1", &Rectification::passthrough1, DOC(dai, node, Rectification, passthrough1))
        .def_readonly("passthrough2", &Rectification::passthrough2, DOC(dai, node, Rectification, passthrough2))
        .def_readonly("output1", &Rectification::output1, DOC(dai, node, Rectification, output1))
        .def_readonly("output2", &Rectification::output2, DOC(dai, node, Rectification, output2))
        .def_property_readonly(
            "input1", [](Rectification& node) { return &node.input1; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "input2", [](Rectification& node) { return &node.input2; }, py::return_value_policy::reference_internal)
        .def("setRunOnHost", &Rectification::setRunOnHost, py::arg("num"), DOC(dai, node, Rectification, setRunOnHost));

    // Keep the Properties class attached to the node type
    daiNodeModule.attr("Rectification").attr("Properties") = RectificationProperties;
}