#include <pybind11/stl.h>

#include "depthai/beta/node/MapOutputParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_mapoutputparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto mapOutputParser = ADD_BETA_NODE_DERIVED(MapOutputParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    mapOutputParser.def_readonly("input", &MapOutputParser::input, DOC(dai, beta, node, MapOutputParser, input))
        .def_readonly("out", &MapOutputParser::out, DOC(dai, beta, node, MapOutputParser, out))
        .def(
            "build",
            [](MapOutputParser& self, Node::Output& nnInput, const MapOutputParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, MapOutputParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&MapOutputParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, MapOutputParser, build, 2))
        .def("setNNArchive", &MapOutputParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, MapOutputParser, setNNArchive))
        .def("setNNArchiveHead", &MapOutputParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, MapOutputParser, setNNArchiveHead))
        .def("setOutputLayerName", &MapOutputParser::setOutputLayerName, py::arg("outputLayerName"), DOC(dai, beta, node, MapOutputParser, setOutputLayerName))
        .def("getOutputLayerName", &MapOutputParser::getOutputLayerName, DOC(dai, beta, node, MapOutputParser, getOutputLayerName))
        .def("setMinMaxScaling", &MapOutputParser::setMinMaxScaling, py::arg("minMaxScaling") = true, DOC(dai, beta, node, MapOutputParser, setMinMaxScaling))
        .def("getMinMaxScaling", &MapOutputParser::getMinMaxScaling, DOC(dai, beta, node, MapOutputParser, getMinMaxScaling))
        .def("setRunOnHost", &MapOutputParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, MapOutputParser, setRunOnHost))
        .def("runOnHost", &MapOutputParser::runOnHost, DOC(dai, beta, node, MapOutputParser, runOnHost));
}
