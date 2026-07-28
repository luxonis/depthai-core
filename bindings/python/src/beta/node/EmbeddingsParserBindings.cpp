#include <pybind11/stl.h>

#include "depthai/beta/node/EmbeddingsParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_embeddingsparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto embeddingsParser = ADD_BETA_NODE_DERIVED(EmbeddingsParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    embeddingsParser.def_readonly("input", &EmbeddingsParser::input, DOC(dai, beta, node, EmbeddingsParser, input))
        .def_readonly("out", &EmbeddingsParser::out, DOC(dai, beta, node, EmbeddingsParser, out))
        .def(
            "build",
            [](EmbeddingsParser& self, Node::Output& nnInput, const EmbeddingsParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, EmbeddingsParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&EmbeddingsParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, EmbeddingsParser, build, 2))
        .def("setNNArchive", &EmbeddingsParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, EmbeddingsParser, setNNArchive))
        .def("setNNArchiveHead", &EmbeddingsParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, EmbeddingsParser, setNNArchiveHead))
        .def(
            "setOutputLayerName", &EmbeddingsParser::setOutputLayerName, py::arg("outputLayerName"), DOC(dai, beta, node, EmbeddingsParser, setOutputLayerName))
        .def("getOutputLayerName", &EmbeddingsParser::getOutputLayerName, DOC(dai, beta, node, EmbeddingsParser, getOutputLayerName));
}
