#include <pybind11/stl.h>

#include "depthai/beta/node/LaneDetectionParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_lanedetectionparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto laneDetectionParser = ADD_BETA_NODE_DERIVED(LaneDetectionParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    laneDetectionParser.def_readonly("input", &LaneDetectionParser::input, DOC(dai, beta, node, LaneDetectionParser, input))
        .def_readonly("out", &LaneDetectionParser::out, DOC(dai, beta, node, LaneDetectionParser, out))
        .def(
            "build",
            [](LaneDetectionParser& self, Node::Output& nnInput, const LaneDetectionParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, LaneDetectionParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&LaneDetectionParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, LaneDetectionParser, build, 2))
        .def("setNNArchive", &LaneDetectionParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, LaneDetectionParser, setNNArchive))
        .def("setNNArchiveHead", &LaneDetectionParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, LaneDetectionParser, setNNArchiveHead))
        .def("setOutputLayerName",
             &LaneDetectionParser::setOutputLayerName,
             py::arg("outputLayerName"),
             DOC(dai, beta, node, LaneDetectionParser, setOutputLayerName))
        .def("getOutputLayerName", &LaneDetectionParser::getOutputLayerName, DOC(dai, beta, node, LaneDetectionParser, getOutputLayerName))
        .def("setRowAnchors", &LaneDetectionParser::setRowAnchors, py::arg("rowAnchors"), DOC(dai, beta, node, LaneDetectionParser, setRowAnchors))
        .def("getRowAnchors", &LaneDetectionParser::getRowAnchors, DOC(dai, beta, node, LaneDetectionParser, getRowAnchors))
        .def("setGridingNum", &LaneDetectionParser::setGridingNum, py::arg("gridingNum"), DOC(dai, beta, node, LaneDetectionParser, setGridingNum))
        .def("getGridingNum", &LaneDetectionParser::getGridingNum, DOC(dai, beta, node, LaneDetectionParser, getGridingNum))
        .def("setClsNumPerLane", &LaneDetectionParser::setClsNumPerLane, py::arg("clsNumPerLane"), DOC(dai, beta, node, LaneDetectionParser, setClsNumPerLane))
        .def("getClsNumPerLane", &LaneDetectionParser::getClsNumPerLane, DOC(dai, beta, node, LaneDetectionParser, getClsNumPerLane))
        .def("setInputSize", &LaneDetectionParser::setInputSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, LaneDetectionParser, setInputSize))
        .def("getInputSize", &LaneDetectionParser::getInputSize, DOC(dai, beta, node, LaneDetectionParser, getInputSize));
}
