#include "depthai/beta/node/ToFStereoFusion.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_tofstereofusion(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    py::class_<beta::ToFStereoFusionConfig, std::shared_ptr<beta::ToFStereoFusionConfig>>(m, "ToFStereoFusionConfig")
        .def(py::init<>())
        .def_readwrite("cropToOverlap", &beta::ToFStereoFusionConfig::cropToOverlap)
        .def("setCropToOverlap", &beta::ToFStereoFusionConfig::setCropToOverlap, py::arg("enabled"), DOC(dai, beta, ToFStereoFusionConfig, setCropToOverlap))
        .def_readwrite("confidenceThreshold", &beta::ToFStereoFusionConfig::confidenceThreshold)
        .def("setConfidenceThreshold",
             &beta::ToFStereoFusionConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, ToFStereoFusionConfig, setConfidenceThreshold));

    m.def_submodule("beta", "Experimental APIs").attr("ToFStereoFusionConfig") = m.attr("ToFStereoFusionConfig");
    auto node = addBetaNode<ToFStereoFusion>("ToFStereoFusion", DOC(dai, beta, node, ToFStereoFusion));
    py::enum_<beta::ToFStereoFusionProperties::Model> model(node, "Model", DOC(dai, beta, ToFStereoFusionProperties, Model));
    py::enum_<beta::ToFStereoFusionProperties::Backend> backend(node, "Backend", DOC(dai, beta, ToFStereoFusionProperties, Backend));
    py::enum_<beta::ToFStereoFusionProperties::ToFConfidenceScale> confidenceScale(
        node, "ToFConfidenceScale", DOC(dai, beta, ToFStereoFusionProperties, ToFConfidenceScale));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    model.value("ALPHA_BLEND", beta::ToFStereoFusionProperties::Model::ALPHA_BLEND)
        .value("STEREO_NETWORK", beta::ToFStereoFusionProperties::Model::STEREO_NETWORK);
    backend.value("AUTO", beta::ToFStereoFusionProperties::Backend::AUTO)
        .value("CPU", beta::ToFStereoFusionProperties::Backend::CPU)
        .value("GPU", beta::ToFStereoFusionProperties::Backend::GPU);
    confidenceScale.value("FUSION", beta::ToFStereoFusionProperties::ToFConfidenceScale::FUSION)
        .value("VENDOR", beta::ToFStereoFusionProperties::ToFConfidenceScale::VENDOR);

    node.def("setModel", &ToFStereoFusion::setModel, py::arg("model"), DOC(dai, beta, node, ToFStereoFusion, setModel))
        .def("getModel", &ToFStereoFusion::getModel, DOC(dai, beta, node, ToFStereoFusion, getModel))
        .def("setBackend", &ToFStereoFusion::setBackend, py::arg("backend"), DOC(dai, beta, node, ToFStereoFusion, setBackend))
        .def("setToFConfidenceScale", &ToFStereoFusion::setToFConfidenceScale, py::arg("scale"), DOC(dai, beta, node, ToFStereoFusion, setToFConfidenceScale))
        .def("setNetworkInputSize",
             &ToFStereoFusion::setNetworkInputSize,
             py::arg("width"),
             py::arg("height"),
             DOC(dai, beta, node, ToFStereoFusion, setNetworkInputSize));

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
        .def("build", &ToFStereoFusion::build, py::arg("left"), py::arg("right"), py::arg("fps") = 30.0f, DOC(dai, beta, node, ToFStereoFusion, build));

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
