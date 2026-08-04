#include <pybind11/stl.h>

#include "depthai/beta/node/MPPalmDetectionParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_mppalmdetectionparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto mpPalmDetectionParser = ADD_BETA_NODE_DERIVED(MPPalmDetectionParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    mpPalmDetectionParser.def_readonly("input", &MPPalmDetectionParser::input, DOC(dai, beta, node, MPPalmDetectionParser, input))
        .def_readonly("out", &MPPalmDetectionParser::out, DOC(dai, beta, node, MPPalmDetectionParser, out))
        .def(
            "build",
            [](MPPalmDetectionParser& self, Node::Output& nnInput, const MPPalmDetectionParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, MPPalmDetectionParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&MPPalmDetectionParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, MPPalmDetectionParser, build, 2))
        .def("setNNArchive", &MPPalmDetectionParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, MPPalmDetectionParser, setNNArchive))
        .def("setNNArchiveHead", &MPPalmDetectionParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, MPPalmDetectionParser, setNNArchiveHead))
        .def("setOutputLayerNames",
             &MPPalmDetectionParser::setOutputLayerNames,
             py::arg("outputLayerNames"),
             DOC(dai, beta, node, MPPalmDetectionParser, setOutputLayerNames))
        .def("getOutputLayerNames", &MPPalmDetectionParser::getOutputLayerNames, DOC(dai, beta, node, MPPalmDetectionParser, getOutputLayerNames))
        .def("setConfidenceThreshold",
             &MPPalmDetectionParser::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, node, MPPalmDetectionParser, setConfidenceThreshold))
        .def("getConfidenceThreshold", &MPPalmDetectionParser::getConfidenceThreshold, DOC(dai, beta, node, MPPalmDetectionParser, getConfidenceThreshold))
        .def("setIouThreshold", &MPPalmDetectionParser::setIouThreshold, py::arg("threshold"), DOC(dai, beta, node, MPPalmDetectionParser, setIouThreshold))
        .def("getIouThreshold", &MPPalmDetectionParser::getIouThreshold, DOC(dai, beta, node, MPPalmDetectionParser, getIouThreshold))
        .def("setMaxDetections",
             &MPPalmDetectionParser::setMaxDetections,
             py::arg("maxDetections"),
             DOC(dai, beta, node, MPPalmDetectionParser, setMaxDetections))
        .def("getMaxDetections", &MPPalmDetectionParser::getMaxDetections, DOC(dai, beta, node, MPPalmDetectionParser, getMaxDetections))
        .def("setScale", &MPPalmDetectionParser::setScale, py::arg("scale"), DOC(dai, beta, node, MPPalmDetectionParser, setScale))
        .def("getScale", &MPPalmDetectionParser::getScale, DOC(dai, beta, node, MPPalmDetectionParser, getScale))
        .def("setLabelNames", &MPPalmDetectionParser::setLabelNames, py::arg("labelNames"), DOC(dai, beta, node, MPPalmDetectionParser, setLabelNames))
        .def("getLabelNames", &MPPalmDetectionParser::getLabelNames, DOC(dai, beta, node, MPPalmDetectionParser, getLabelNames))
        .def("setRunOnHost", &MPPalmDetectionParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, MPPalmDetectionParser, setRunOnHost))
        .def("runOnHost", &MPPalmDetectionParser::runOnHost, DOC(dai, beta, node, MPPalmDetectionParser, runOnHost));
}
