#include <pybind11/stl.h>

#include "depthai/beta/node/KeypointParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_keypointparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto keypointParser = ADD_BETA_NODE_DERIVED(KeypointParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    keypointParser.def_readonly("input", &KeypointParser::input, DOC(dai, beta, node, KeypointParser, input))
        .def_readonly("out", &KeypointParser::out, DOC(dai, beta, node, KeypointParser, out))
        .def(
            "build",
            [](KeypointParser& self, Node::Output& nnInput, const KeypointParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, KeypointParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&KeypointParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, KeypointParser, build, 2))
        .def("setNNArchive", &KeypointParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, KeypointParser, setNNArchive))
        .def("setNNArchiveHead", &KeypointParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, KeypointParser, setNNArchiveHead))
        .def("setOutputLayerName", &KeypointParser::setOutputLayerName, py::arg("outputLayerName"), DOC(dai, beta, node, KeypointParser, setOutputLayerName))
        .def("getOutputLayerName", &KeypointParser::getOutputLayerName, DOC(dai, beta, node, KeypointParser, getOutputLayerName))
        .def("setScaleFactor", &KeypointParser::setScaleFactor, py::arg("scaleFactor"), DOC(dai, beta, node, KeypointParser, setScaleFactor))
        .def("getScaleFactor", &KeypointParser::getScaleFactor, DOC(dai, beta, node, KeypointParser, getScaleFactor))
        .def("setNumKeypoints", &KeypointParser::setNumKeypoints, py::arg("nKeypoints"), DOC(dai, beta, node, KeypointParser, setNumKeypoints))
        .def("getNumKeypoints", &KeypointParser::getNumKeypoints, DOC(dai, beta, node, KeypointParser, getNumKeypoints))
        .def("setScoreThreshold", &KeypointParser::setScoreThreshold, py::arg("threshold"), DOC(dai, beta, node, KeypointParser, setScoreThreshold))
        .def("getScoreThreshold", &KeypointParser::getScoreThreshold, DOC(dai, beta, node, KeypointParser, getScoreThreshold))
        .def("setLabelNames", &KeypointParser::setLabelNames, py::arg("labelNames"), DOC(dai, beta, node, KeypointParser, setLabelNames))
        .def("getLabelNames", &KeypointParser::getLabelNames, DOC(dai, beta, node, KeypointParser, getLabelNames))
        .def("setEdges", &KeypointParser::setEdges, py::arg("edges"), DOC(dai, beta, node, KeypointParser, setEdges))
        .def("getEdges", &KeypointParser::getEdges, DOC(dai, beta, node, KeypointParser, getEdges))
        .def("setRunOnHost", &KeypointParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, KeypointParser, setRunOnHost))
        .def("runOnHost", &KeypointParser::runOnHost, DOC(dai, beta, node, KeypointParser, runOnHost));
}
