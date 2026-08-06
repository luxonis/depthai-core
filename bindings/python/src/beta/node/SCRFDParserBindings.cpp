#include <pybind11/stl.h>

#include "depthai/beta/node/SCRFDParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_scrfdparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::SCRFDParserProperties> scrfdParserProperties(betaModule, "SCRFDParserProperties");
    auto scrfdParser = ADD_BETA_NODE_DERIVED(SCRFDParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    scrfdParserProperties.def_readwrite("initialConfig", &beta::SCRFDParserProperties::initialConfig)
        .def_readwrite("outputLayerNames", &beta::SCRFDParserProperties::outputLayerNames)
        .def_readwrite("inputSize", &beta::SCRFDParserProperties::inputSize)
        .def_readwrite("featStrideFpn", &beta::SCRFDParserProperties::featStrideFpn)
        .def_readwrite("numAnchors", &beta::SCRFDParserProperties::numAnchors)
        .def_readwrite("labelNames", &beta::SCRFDParserProperties::labelNames);

    scrfdParser.def_readonly("input", &SCRFDParser::input, DOC(dai, beta, node, SCRFDParser, input))
        .def_readonly("inputConfig", &SCRFDParser::inputConfig, DOC(dai, beta, node, SCRFDParser, inputConfig))
        .def_readonly("out", &SCRFDParser::out, DOC(dai, beta, node, SCRFDParser, out))
        .def_readonly("initialConfig", &SCRFDParser::initialConfig, DOC(dai, beta, node, SCRFDParser, initialConfig))
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
        .def("getLabelNames", &SCRFDParser::getLabelNames, DOC(dai, beta, node, SCRFDParser, getLabelNames))
        .def("setRunOnHost", &SCRFDParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, SCRFDParser, setRunOnHost))
        .def("runOnHost", &SCRFDParser::runOnHost, DOC(dai, beta, node, SCRFDParser, runOnHost));

    scrfdParser.attr("Properties") = scrfdParserProperties;
}
