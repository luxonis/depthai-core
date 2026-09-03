#include "depthai/beta/node/ToFStereoFusion.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_tofstereofusion(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    py::class_<beta::ToFStereoFusionConfig, std::shared_ptr<beta::ToFStereoFusionConfig>>(m, "ToFStereoFusionConfig")
        .def(py::init<>())
        .def_readwrite("confidenceThreshold", &beta::ToFStereoFusionConfig::confidenceThreshold)
        .def("setConfidenceThreshold", &beta::ToFStereoFusionConfig::setConfidenceThreshold, py::arg("threshold"));

    m.def_submodule("beta", "Experimental APIs");
    auto node = addBetaNode<ToFStereoFusion>("ToFStereoFusion", DOC(dai, beta, node, ToFStereoFusion));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    node.def_property_readonly(
            "depth", [](ToFStereoFusion& n) -> Node::Output& { return n.depth; }, DOC(dai, beta, node, ToFStereoFusion, depth))
        .def_readonly("initialConfig", &ToFStereoFusion::initialConfig)
        .def_property_readonly(
            "confidence", [](ToFStereoFusion& n) -> Node::Output& { return n.confidence; }, DOC(dai, beta, node, ToFStereoFusion, confidence))
        .def_property_readonly(
            "tof", [](ToFStereoFusion& n) -> dai::node::ToF& { return *n.tof; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "neuralDepth", [](ToFStereoFusion& n) -> dai::node::NeuralDepth& { return *n.neuralDepth; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "neuralNetwork", [](ToFStereoFusion& n) -> dai::node::NeuralNetwork& { return *n.neuralNetwork; }, py::return_value_policy::reference_internal)
        .def("build",
             py::overload_cast<const std::shared_ptr<dai::node::Camera>&, const std::shared_ptr<dai::node::Camera>&>(&ToFStereoFusion::build),
             py::arg("left"),
             py::arg("right"),
             DOC(dai, beta, node, ToFStereoFusion, build));

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    node.def_property_readonly(
            "inputLeft",
            [](ToFStereoFusion& n) { return &n.inputLeft; },
            py::return_value_policy::reference_internal,
            DOC(dai, beta, node, ToFStereoFusion, inputLeft))
        .def_property_readonly(
            "inputRight",
            [](ToFStereoFusion& n) { return &n.inputRight; },
            py::return_value_policy::reference_internal,
            DOC(dai, beta, node, ToFStereoFusion, inputRight));
#endif
}
