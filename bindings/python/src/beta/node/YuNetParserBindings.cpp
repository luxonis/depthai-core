#include <pybind11/stl.h>

#include "depthai/beta/node/YuNetParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_yunetparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto yunetParser = ADD_BETA_NODE_DERIVED(YuNetParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    yunetParser.def_readonly("input", &YuNetParser::input, DOC(dai, beta, node, YuNetParser, input))
        .def_readonly("out", &YuNetParser::out, DOC(dai, beta, node, YuNetParser, out))
        .def(
            "build",
            [](YuNetParser& self, Node::Output& nnInput, const YuNetParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, YuNetParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&YuNetParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, YuNetParser, build, 2))
        .def("setNNArchive", &YuNetParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, YuNetParser, setNNArchive))
        .def("setNNArchiveHead", &YuNetParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, YuNetParser, setNNArchiveHead))
        .def("setOutputLayerLoc", &YuNetParser::setOutputLayerLoc, py::arg("locOutputLayerName"), DOC(dai, beta, node, YuNetParser, setOutputLayerLoc))
        .def("getOutputLayerLoc", &YuNetParser::getOutputLayerLoc, DOC(dai, beta, node, YuNetParser, getOutputLayerLoc))
        .def("setOutputLayerConf", &YuNetParser::setOutputLayerConf, py::arg("confOutputLayerName"), DOC(dai, beta, node, YuNetParser, setOutputLayerConf))
        .def("getOutputLayerConf", &YuNetParser::getOutputLayerConf, DOC(dai, beta, node, YuNetParser, getOutputLayerConf))
        .def("setOutputLayerIou", &YuNetParser::setOutputLayerIou, py::arg("iouOutputLayerName"), DOC(dai, beta, node, YuNetParser, setOutputLayerIou))
        .def("getOutputLayerIou", &YuNetParser::getOutputLayerIou, DOC(dai, beta, node, YuNetParser, getOutputLayerIou))
        .def("setConfidenceThreshold", &YuNetParser::setConfidenceThreshold, py::arg("threshold"), DOC(dai, beta, node, YuNetParser, setConfidenceThreshold))
        .def("getConfidenceThreshold", &YuNetParser::getConfidenceThreshold, DOC(dai, beta, node, YuNetParser, getConfidenceThreshold))
        .def("setIouThreshold", &YuNetParser::setIouThreshold, py::arg("threshold"), DOC(dai, beta, node, YuNetParser, setIouThreshold))
        .def("getIouThreshold", &YuNetParser::getIouThreshold, DOC(dai, beta, node, YuNetParser, getIouThreshold))
        .def("setMaxDetections", &YuNetParser::setMaxDetections, py::arg("maxDetections"), DOC(dai, beta, node, YuNetParser, setMaxDetections))
        .def("getMaxDetections", &YuNetParser::getMaxDetections, DOC(dai, beta, node, YuNetParser, getMaxDetections))
        .def("setInputSize", &YuNetParser::setInputSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, YuNetParser, setInputSize))
        .def("getInputSize", &YuNetParser::getInputSize, DOC(dai, beta, node, YuNetParser, getInputSize))
        .def("setLabelNames", &YuNetParser::setLabelNames, py::arg("labelNames"), DOC(dai, beta, node, YuNetParser, setLabelNames))
        .def("getLabelNames", &YuNetParser::getLabelNames, DOC(dai, beta, node, YuNetParser, getLabelNames));
}
