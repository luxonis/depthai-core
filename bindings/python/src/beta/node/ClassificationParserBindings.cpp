#include <pybind11/stl.h>

#include "depthai/beta/node/ClassificationParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_classificationparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto classificationParser = ADD_BETA_NODE_DERIVED(ClassificationParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    classificationParser.def_readonly("input", &ClassificationParser::input, DOC(dai, beta, node, ClassificationParser, input))
        .def_readonly("out", &ClassificationParser::out, DOC(dai, beta, node, ClassificationParser, out))
        .def(
            "build",
            [](ClassificationParser& self, Node::Output& nnInput, const ClassificationParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, ClassificationParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&ClassificationParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, ClassificationParser, build, 2))
        .def("setNNArchive", &ClassificationParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, ClassificationParser, setNNArchive))
        .def("setNNArchiveHead", &ClassificationParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, ClassificationParser, setNNArchiveHead))
        .def("setOutputLayerName",
             &ClassificationParser::setOutputLayerName,
             py::arg("outputLayerName"),
             DOC(dai, beta, node, ClassificationParser, setOutputLayerName))
        .def("getOutputLayerName", &ClassificationParser::getOutputLayerName, DOC(dai, beta, node, ClassificationParser, getOutputLayerName))
        .def("setClasses", &ClassificationParser::setClasses, py::arg("classes"), DOC(dai, beta, node, ClassificationParser, setClasses))
        .def("getClasses", &ClassificationParser::getClasses, DOC(dai, beta, node, ClassificationParser, getClasses))
        .def("setSoftmax", &ClassificationParser::setSoftmax, py::arg("isSoftmax"), DOC(dai, beta, node, ClassificationParser, setSoftmax))
        .def("getSoftmax", &ClassificationParser::getSoftmax, DOC(dai, beta, node, ClassificationParser, getSoftmax));
}
