#include <pybind11/stl.h>

#include "depthai/beta/node/SuperAnimalParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_superanimalparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto superAnimalParser = ADD_BETA_NODE_DERIVED(SuperAnimalParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    superAnimalParser.def_readonly("input", &SuperAnimalParser::input, DOC(dai, beta, node, SuperAnimalParser, input))
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
        .def("getEdges", &SuperAnimalParser::getEdges, DOC(dai, beta, node, SuperAnimalParser, getEdges));
}
