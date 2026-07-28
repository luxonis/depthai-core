#include <pybind11/stl.h>

#include "depthai/beta/node/ImageOutputParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_imageoutputparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto imageOutputParser = ADD_BETA_NODE_DERIVED(ImageOutputParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    imageOutputParser.def_readonly("input", &ImageOutputParser::input, DOC(dai, beta, node, ImageOutputParser, input))
        .def_readonly("out", &ImageOutputParser::out, DOC(dai, beta, node, ImageOutputParser, out))
        .def(
            "build",
            [](ImageOutputParser& self, Node::Output& nnInput, const ImageOutputParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, ImageOutputParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&ImageOutputParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, ImageOutputParser, build, 2))
        .def("setNNArchive", &ImageOutputParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, ImageOutputParser, setNNArchive))
        .def("setNNArchiveHead", &ImageOutputParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, ImageOutputParser, setNNArchiveHead))
        .def("setOutputLayerName",
             &ImageOutputParser::setOutputLayerName,
             py::arg("outputLayerName"),
             DOC(dai, beta, node, ImageOutputParser, setOutputLayerName))
        .def("getOutputLayerName", &ImageOutputParser::getOutputLayerName, DOC(dai, beta, node, ImageOutputParser, getOutputLayerName))
        .def("setBGROutput", &ImageOutputParser::setBGROutput, py::arg("outputIsBGR") = true, DOC(dai, beta, node, ImageOutputParser, setBGROutput))
        .def("getBGROutput", &ImageOutputParser::getBGROutput, DOC(dai, beta, node, ImageOutputParser, getBGROutput));
}
