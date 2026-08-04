#include <pybind11/stl.h>

#include "depthai/beta/node/MLSDParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_mlsdparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto mlsdParser = ADD_BETA_NODE_DERIVED(MLSDParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    mlsdParser.def_readonly("input", &MLSDParser::input, DOC(dai, beta, node, MLSDParser, input))
        .def_readonly("out", &MLSDParser::out, DOC(dai, beta, node, MLSDParser, out))
        .def(
            "build",
            [](MLSDParser& self, Node::Output& nnInput, const MLSDParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, MLSDParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&MLSDParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, MLSDParser, build, 2))
        .def("setNNArchive", &MLSDParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, MLSDParser, setNNArchive))
        .def("setNNArchiveHead", &MLSDParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, MLSDParser, setNNArchiveHead))
        .def("setOutputLayerTPMap", &MLSDParser::setOutputLayerTPMap, py::arg("outputLayerTPMap"), DOC(dai, beta, node, MLSDParser, setOutputLayerTPMap))
        .def("getOutputLayerTPMap", &MLSDParser::getOutputLayerTPMap, DOC(dai, beta, node, MLSDParser, getOutputLayerTPMap))
        .def("setOutputLayerHeat", &MLSDParser::setOutputLayerHeat, py::arg("outputLayerHeat"), DOC(dai, beta, node, MLSDParser, setOutputLayerHeat))
        .def("getOutputLayerHeat", &MLSDParser::getOutputLayerHeat, DOC(dai, beta, node, MLSDParser, getOutputLayerHeat))
        .def("setTopK", &MLSDParser::setTopK, py::arg("topK"), DOC(dai, beta, node, MLSDParser, setTopK))
        .def("getTopK", &MLSDParser::getTopK, DOC(dai, beta, node, MLSDParser, getTopK))
        .def("setScoreThreshold", &MLSDParser::setScoreThreshold, py::arg("scoreThreshold"), DOC(dai, beta, node, MLSDParser, setScoreThreshold))
        .def("getScoreThreshold", &MLSDParser::getScoreThreshold, DOC(dai, beta, node, MLSDParser, getScoreThreshold))
        .def("setDistanceThreshold", &MLSDParser::setDistanceThreshold, py::arg("distanceThreshold"), DOC(dai, beta, node, MLSDParser, setDistanceThreshold))
        .def("getDistanceThreshold", &MLSDParser::getDistanceThreshold, DOC(dai, beta, node, MLSDParser, getDistanceThreshold))
        .def("setInputSize", &MLSDParser::setInputSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, MLSDParser, setInputSize))
        .def("getInputSize", &MLSDParser::getInputSize, DOC(dai, beta, node, MLSDParser, getInputSize))
        .def("setRunOnHost", &MLSDParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, MLSDParser, setRunOnHost))
        .def("runOnHost", &MLSDParser::runOnHost, DOC(dai, beta, node, MLSDParser, runOnHost));
}
