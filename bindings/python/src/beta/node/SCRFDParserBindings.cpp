#include <pybind11/stl.h>

#include "depthai/beta/node/SCRFDParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_scrfdparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto scrfdParser = ADD_BETA_NODE_DERIVED(SCRFDParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    scrfdParser.def_readonly("input", &SCRFDParser::input, DOC(dai, beta, node, SCRFDParser, input))
        .def_readonly("out", &SCRFDParser::out, DOC(dai, beta, node, SCRFDParser, out))
        .def(
            "build",
            [](SCRFDParser& self, Node::Output& nnInput, const SCRFDParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, SCRFDParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&SCRFDParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, SCRFDParser, build, 2))
        .def("setNNArchive", &SCRFDParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, SCRFDParser, setNNArchive))
        .def("setNNArchiveHead", &SCRFDParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, SCRFDParser, setNNArchiveHead))
        .def("setOutputLayerNames", &SCRFDParser::setOutputLayerNames, py::arg("outputLayerNames"), DOC(dai, beta, node, SCRFDParser, setOutputLayerNames))
        .def("getOutputLayerNames", &SCRFDParser::getOutputLayerNames, DOC(dai, beta, node, SCRFDParser, getOutputLayerNames))
        .def("setConfidenceThreshold", &SCRFDParser::setConfidenceThreshold, py::arg("threshold"), DOC(dai, beta, node, SCRFDParser, setConfidenceThreshold))
        .def("getConfidenceThreshold", &SCRFDParser::getConfidenceThreshold, DOC(dai, beta, node, SCRFDParser, getConfidenceThreshold))
        .def("setIouThreshold", &SCRFDParser::setIouThreshold, py::arg("threshold"), DOC(dai, beta, node, SCRFDParser, setIouThreshold))
        .def("getIouThreshold", &SCRFDParser::getIouThreshold, DOC(dai, beta, node, SCRFDParser, getIouThreshold))
        .def("setMaxDetections", &SCRFDParser::setMaxDetections, py::arg("maxDetections"), DOC(dai, beta, node, SCRFDParser, setMaxDetections))
        .def("getMaxDetections", &SCRFDParser::getMaxDetections, DOC(dai, beta, node, SCRFDParser, getMaxDetections))
        .def("setInputSize", &SCRFDParser::setInputSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, SCRFDParser, setInputSize))
        .def("getInputSize", &SCRFDParser::getInputSize, DOC(dai, beta, node, SCRFDParser, getInputSize))
        .def("setFeatStrideFPN", &SCRFDParser::setFeatStrideFPN, py::arg("featStrideFpn"), DOC(dai, beta, node, SCRFDParser, setFeatStrideFPN))
        .def("getFeatStrideFPN", &SCRFDParser::getFeatStrideFPN, DOC(dai, beta, node, SCRFDParser, getFeatStrideFPN))
        .def("setNumAnchors", &SCRFDParser::setNumAnchors, py::arg("numAnchors"), DOC(dai, beta, node, SCRFDParser, setNumAnchors))
        .def("getNumAnchors", &SCRFDParser::getNumAnchors, DOC(dai, beta, node, SCRFDParser, getNumAnchors))
        .def("setLabelNames", &SCRFDParser::setLabelNames, py::arg("labelNames"), DOC(dai, beta, node, SCRFDParser, setLabelNames))
        .def("getLabelNames", &SCRFDParser::getLabelNames, DOC(dai, beta, node, SCRFDParser, getLabelNames));
}
