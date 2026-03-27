#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/GPUStereo.hpp"

void bind_gpustereo(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<GPUStereoProperties> properties(m, "GPUStereoProperties", DOC(dai, GPUStereoProperties));
    auto node = ADD_NODE(GPUStereo);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    properties.def_readwrite("initialConfig", &GPUStereoProperties::initialConfig, DOC(dai, GPUStereoProperties, initialConfig))
        .def_readwrite("numFramesPool", &GPUStereoProperties::numFramesPool, DOC(dai, GPUStereoProperties, numFramesPool));

    node.def_property_readonly(
            "left",
            [](const GPUStereo& n) { return &n.sync->inputs["left"]; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, GPUStereo, left))
        .def_property_readonly(
            "right",
            [](const GPUStereo& n) { return &n.sync->inputs["right"]; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, GPUStereo, right))
        .def_property_readonly(
            "rectifiedLeft",
            [](const GPUStereo& n) { return &n.rectification->output1; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, GPUStereo, rectifiedLeft))
        .def_property_readonly(
            "rectifiedRight",
            [](const GPUStereo& n) { return &n.rectification->output2; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, GPUStereo, rectifiedRight))
        .def("setRectification", &GPUStereo::setRectification, py::arg("enable"), DOC(dai, node, GPUStereo, setRectification))
        .def_readonly("inputConfig", &GPUStereo::inputConfig, DOC(dai, node, GPUStereo, inputConfig))
        .def_readonly("initialConfig", &GPUStereo::initialConfig, DOC(dai, node, GPUStereo, initialConfig))
        .def_readonly("disparity", &GPUStereo::disparity, DOC(dai, node, GPUStereo, disparity), DOC(dai, node, GPUStereo, disparity))
        .def_readonly("depth", &GPUStereo::depth, DOC(dai, node, GPUStereo, depth), DOC(dai, node, GPUStereo, depth))
        .def("build", &GPUStereo::build, py::arg("leftInput"), py::arg("rightInput"), DOC(dai, node, GPUStereo, build))
        .def_property_readonly(
            "sync", [](GPUStereo& n) { return &(*n.sync); }, py::return_value_policy::reference_internal, DOC(dai, node, GPUStereo, sync))
        .def_property_readonly(
            "messageDemux",
            [](GPUStereo& n) { return &(*n.messageDemux); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, GPUStereo, messageDemux))
        .def_property_readonly(
            "rectification",
            [](GPUStereo& n) { return &(*n.rectification); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, GPUStereo, rectification));

    daiNodeModule.attr("GPUStereo").attr("Properties") = properties;
}
