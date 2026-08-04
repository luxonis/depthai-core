#include <pybind11/stl.h>

#include "depthai/beta/node/HRNetParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_hrnetparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto hrnetParser = ADD_BETA_NODE_DERIVED(HRNetParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    hrnetParser.def_readonly("input", &HRNetParser::input, DOC(dai, beta, node, HRNetParser, input))
        .def_readonly("out", &HRNetParser::out, DOC(dai, beta, node, HRNetParser, out))
        .def(
            "build",
            [](HRNetParser& self, Node::Output& nnInput, const HRNetParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, HRNetParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&HRNetParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, HRNetParser, build, 2))
        .def("setNNArchive", &HRNetParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, HRNetParser, setNNArchive))
        .def("setNNArchiveHead", &HRNetParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, HRNetParser, setNNArchiveHead))
        .def("setOutputLayerName", &HRNetParser::setOutputLayerName, py::arg("outputLayerName"), DOC(dai, beta, node, HRNetParser, setOutputLayerName))
        .def("getOutputLayerName", &HRNetParser::getOutputLayerName, DOC(dai, beta, node, HRNetParser, getOutputLayerName))
        .def("setScoreThreshold", &HRNetParser::setScoreThreshold, py::arg("threshold"), DOC(dai, beta, node, HRNetParser, setScoreThreshold))
        .def("getScoreThreshold", &HRNetParser::getScoreThreshold, DOC(dai, beta, node, HRNetParser, getScoreThreshold))
        .def("setLabelNames", &HRNetParser::setLabelNames, py::arg("labelNames"), DOC(dai, beta, node, HRNetParser, setLabelNames))
        .def("getLabelNames", &HRNetParser::getLabelNames, DOC(dai, beta, node, HRNetParser, getLabelNames))
        .def("setEdges", &HRNetParser::setEdges, py::arg("edges"), DOC(dai, beta, node, HRNetParser, setEdges))
        .def("getEdges", &HRNetParser::getEdges, DOC(dai, beta, node, HRNetParser, getEdges))
        .def("setRunOnHost", &HRNetParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, HRNetParser, setRunOnHost))
        .def("runOnHost", &HRNetParser::runOnHost, DOC(dai, beta, node, HRNetParser, runOnHost));
}
