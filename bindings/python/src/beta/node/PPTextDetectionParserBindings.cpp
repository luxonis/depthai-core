#include <pybind11/stl.h>

#include "depthai/beta/node/PPTextDetectionParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_pptextdetectionparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto ppTextDetectionParser = ADD_BETA_NODE_DERIVED(PPTextDetectionParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    ppTextDetectionParser.def_readonly("input", &PPTextDetectionParser::input, DOC(dai, beta, node, PPTextDetectionParser, input))
        .def_readonly("out", &PPTextDetectionParser::out, DOC(dai, beta, node, PPTextDetectionParser, out))
        .def(
            "build",
            [](PPTextDetectionParser& self, Node::Output& nnInput, const PPTextDetectionParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, PPTextDetectionParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&PPTextDetectionParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, PPTextDetectionParser, build, 2))
        .def("setNNArchive", &PPTextDetectionParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, PPTextDetectionParser, setNNArchive))
        .def("setNNArchiveHead", &PPTextDetectionParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, PPTextDetectionParser, setNNArchiveHead))
        .def("setOutputLayerName",
             &PPTextDetectionParser::setOutputLayerName,
             py::arg("outputLayerName"),
             DOC(dai, beta, node, PPTextDetectionParser, setOutputLayerName))
        .def("getOutputLayerName", &PPTextDetectionParser::getOutputLayerName, DOC(dai, beta, node, PPTextDetectionParser, getOutputLayerName))
        .def("setConfidenceThreshold",
             &PPTextDetectionParser::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, node, PPTextDetectionParser, setConfidenceThreshold))
        .def("getConfidenceThreshold", &PPTextDetectionParser::getConfidenceThreshold, DOC(dai, beta, node, PPTextDetectionParser, getConfidenceThreshold))
        .def("setMaskThreshold",
             &PPTextDetectionParser::setMaskThreshold,
             py::arg("maskThreshold"),
             DOC(dai, beta, node, PPTextDetectionParser, setMaskThreshold))
        .def("getMaskThreshold", &PPTextDetectionParser::getMaskThreshold, DOC(dai, beta, node, PPTextDetectionParser, getMaskThreshold))
        .def("setMaxDetections",
             &PPTextDetectionParser::setMaxDetections,
             py::arg("maxDetections"),
             DOC(dai, beta, node, PPTextDetectionParser, setMaxDetections))
        .def("getMaxDetections", &PPTextDetectionParser::getMaxDetections, DOC(dai, beta, node, PPTextDetectionParser, getMaxDetections));
}
