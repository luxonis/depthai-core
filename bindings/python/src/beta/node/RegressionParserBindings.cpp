#include <pybind11/stl.h>

#include "depthai/beta/node/RegressionParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_regressionparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto regressionParser = ADD_BETA_NODE_DERIVED(RegressionParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    regressionParser.def_readonly("input", &RegressionParser::input, DOC(dai, beta, node, RegressionParser, input))
        .def_readonly("out", &RegressionParser::out, DOC(dai, beta, node, RegressionParser, out))
        .def(
            "build",
            [](RegressionParser& self, Node::Output& nnInput, const RegressionParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, RegressionParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&RegressionParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, RegressionParser, build, 2))
        .def("setNNArchive", &RegressionParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, RegressionParser, setNNArchive))
        .def("setNNArchiveHead", &RegressionParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, RegressionParser, setNNArchiveHead))
        .def(
            "setOutputLayerName", &RegressionParser::setOutputLayerName, py::arg("outputLayerName"), DOC(dai, beta, node, RegressionParser, setOutputLayerName))
        .def("getOutputLayerName", &RegressionParser::getOutputLayerName, DOC(dai, beta, node, RegressionParser, getOutputLayerName))
        .def("setRunOnHost", &RegressionParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, RegressionParser, setRunOnHost))
        .def("runOnHost", &RegressionParser::runOnHost, DOC(dai, beta, node, RegressionParser, runOnHost));
}
