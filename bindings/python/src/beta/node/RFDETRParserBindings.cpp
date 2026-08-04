#include <pybind11/stl.h>

#include "depthai/beta/node/RFDETRParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_rfdetrparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto rfdetrParser = ADD_BETA_NODE_DERIVED(RFDETRParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    rfdetrParser.def_readonly("input", &RFDETRParser::input, DOC(dai, beta, node, RFDETRParser, input))
        .def_readonly("out", &RFDETRParser::out, DOC(dai, beta, node, RFDETRParser, out))
        .def(
            "build",
            [](RFDETRParser& self, Node::Output& nnInput, const RFDETRParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, RFDETRParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&RFDETRParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, RFDETRParser, build, 2))
        .def("setNNArchive", &RFDETRParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, RFDETRParser, setNNArchive))
        .def("setNNArchiveHead", &RFDETRParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, RFDETRParser, setNNArchiveHead))
        .def("setConfidenceThreshold", &RFDETRParser::setConfidenceThreshold, py::arg("threshold"), DOC(dai, beta, node, RFDETRParser, setConfidenceThreshold))
        .def("getConfidenceThreshold", &RFDETRParser::getConfidenceThreshold, DOC(dai, beta, node, RFDETRParser, getConfidenceThreshold))
        .def("setMaxDetections", &RFDETRParser::setMaxDetections, py::arg("maxDetections"), DOC(dai, beta, node, RFDETRParser, setMaxDetections))
        .def("getMaxDetections", &RFDETRParser::getMaxDetections, DOC(dai, beta, node, RFDETRParser, getMaxDetections))
        .def("setLabelNames", &RFDETRParser::setLabelNames, py::arg("labelNames"), DOC(dai, beta, node, RFDETRParser, setLabelNames))
        .def("getLabelNames", &RFDETRParser::getLabelNames, DOC(dai, beta, node, RFDETRParser, getLabelNames))
        .def("setMaskConfidence", &RFDETRParser::setMaskConfidence, py::arg("maskConfidence"), DOC(dai, beta, node, RFDETRParser, setMaskConfidence))
        .def("getMaskConfidence", &RFDETRParser::getMaskConfidence, DOC(dai, beta, node, RFDETRParser, getMaskConfidence))
        .def("setOutputLayerNames", &RFDETRParser::setOutputLayerNames, py::arg("outputLayerNames"), DOC(dai, beta, node, RFDETRParser, setOutputLayerNames))
        .def("getOutputLayerNames", &RFDETRParser::getOutputLayerNames, DOC(dai, beta, node, RFDETRParser, getOutputLayerNames))
        .def("setInputSize", &RFDETRParser::setInputSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, RFDETRParser, setInputSize))
        .def("getInputSize", &RFDETRParser::getInputSize, DOC(dai, beta, node, RFDETRParser, getInputSize))
        .def("setRunOnHost", &RFDETRParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, RFDETRParser, setRunOnHost))
        .def("runOnHost", &RFDETRParser::runOnHost, DOC(dai, beta, node, RFDETRParser, runOnHost));
}
