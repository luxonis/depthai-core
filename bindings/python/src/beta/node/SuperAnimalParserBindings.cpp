#include <pybind11/stl.h>

#include "depthai/beta/node/SuperAnimalParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_superanimalparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::SuperAnimalParserProperties> properties(betaModule, "SuperAnimalParserProperties");
    auto superAnimalParser = ADD_BETA_NODE_DERIVED(SuperAnimalParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    properties.def_readwrite("initialConfig", &beta::SuperAnimalParserProperties::initialConfig)
        .def_readwrite("outputLayerName", &beta::SuperAnimalParserProperties::outputLayerName)
        .def_readwrite("scaleFactor", &beta::SuperAnimalParserProperties::scaleFactor)
        .def_readwrite("nKeypoints", &beta::SuperAnimalParserProperties::nKeypoints)
        .def_readwrite("labelNames", &beta::SuperAnimalParserProperties::labelNames)
        .def_readwrite("edges", &beta::SuperAnimalParserProperties::edges);

    superAnimalParser.def_readonly("inputConfig", &SuperAnimalParser::inputConfig, DOC(dai, beta, node, SuperAnimalParser, inputConfig))
        .def_readonly("initialConfig", &SuperAnimalParser::initialConfig, DOC(dai, beta, node, SuperAnimalParser, initialConfig))
        .def_readonly("input", &SuperAnimalParser::input, DOC(dai, beta, node, SuperAnimalParser, input))
        .def_readonly("out", &SuperAnimalParser::out, DOC(dai, beta, node, SuperAnimalParser, out))
        .def(
            "build",
            [](SuperAnimalParser& self, Node::Output& nnInput, const SuperAnimalParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, SuperAnimalParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&SuperAnimalParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, SuperAnimalParser, build, 2))
        .def("setNNArchive", &SuperAnimalParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, SuperAnimalParser, setNNArchive))
        .def("setNNArchiveHead", &SuperAnimalParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, SuperAnimalParser, setNNArchiveHead))
        .def("setOutputLayerName",
             &SuperAnimalParser::setOutputLayerName,
             py::arg("outputLayerName"),
             DOC(dai, beta, node, SuperAnimalParser, setOutputLayerName))
        .def("getOutputLayerName", &SuperAnimalParser::getOutputLayerName, DOC(dai, beta, node, SuperAnimalParser, getOutputLayerName))
        .def("setScaleFactor", &SuperAnimalParser::setScaleFactor, py::arg("scaleFactor"), DOC(dai, beta, node, SuperAnimalParser, setScaleFactor))
        .def("getScaleFactor", &SuperAnimalParser::getScaleFactor, DOC(dai, beta, node, SuperAnimalParser, getScaleFactor))
        .def("setNumKeypoints", &SuperAnimalParser::setNumKeypoints, py::arg("nKeypoints"), DOC(dai, beta, node, SuperAnimalParser, setNumKeypoints))
        .def("getNumKeypoints", &SuperAnimalParser::getNumKeypoints, DOC(dai, beta, node, SuperAnimalParser, getNumKeypoints))
        .def("setScoreThreshold", &SuperAnimalParser::setScoreThreshold, py::arg("threshold"), DOC(dai, beta, node, SuperAnimalParser, setScoreThreshold))
        .def("getScoreThreshold", &SuperAnimalParser::getScoreThreshold, DOC(dai, beta, node, SuperAnimalParser, getScoreThreshold))
        .def("setLabelNames", &SuperAnimalParser::setLabelNames, py::arg("labelNames"), DOC(dai, beta, node, SuperAnimalParser, setLabelNames))
        .def("getLabelNames", &SuperAnimalParser::getLabelNames, DOC(dai, beta, node, SuperAnimalParser, getLabelNames))
        .def("setEdges", &SuperAnimalParser::setEdges, py::arg("edges"), DOC(dai, beta, node, SuperAnimalParser, setEdges))
        .def("getEdges", &SuperAnimalParser::getEdges, DOC(dai, beta, node, SuperAnimalParser, getEdges))
        .def("setRunOnHost", &SuperAnimalParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, SuperAnimalParser, setRunOnHost))
        .def("runOnHost", &SuperAnimalParser::runOnHost, DOC(dai, beta, node, SuperAnimalParser, runOnHost));

    superAnimalParser.attr("Properties") = properties;
}
