#include <pybind11/stl.h>

#include "depthai/beta/node/ClassificationSequenceParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_classificationsequenceparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::ClassificationSequenceParserProperties> properties(betaModule, "ClassificationSequenceParserProperties");
    auto classificationSequenceParser = ADD_BETA_NODE_DERIVED(ClassificationSequenceParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    properties.def_readwrite("initialConfig", &beta::ClassificationSequenceParserProperties::initialConfig)
        .def_readwrite("outputLayerName", &beta::ClassificationSequenceParserProperties::outputLayerName)
        .def_readwrite("classes", &beta::ClassificationSequenceParserProperties::classes)
        .def_readwrite("nClasses", &beta::ClassificationSequenceParserProperties::nClasses)
        .def_readwrite("isSoftmax", &beta::ClassificationSequenceParserProperties::isSoftmax);

    classificationSequenceParser.def_readonly("inputConfig", &ClassificationSequenceParser::inputConfig, DOC(dai, beta, node, ClassificationSequenceParser, inputConfig))
        .def_readonly("initialConfig", &ClassificationSequenceParser::initialConfig, DOC(dai, beta, node, ClassificationSequenceParser, initialConfig))
        .def_readonly("input", &ClassificationSequenceParser::input, DOC(dai, beta, node, ClassificationSequenceParser, input))
        .def_readonly("out", &ClassificationSequenceParser::out, DOC(dai, beta, node, ClassificationSequenceParser, out))
        .def(
            "build",
            [](ClassificationSequenceParser& self, Node::Output& nnInput, const ClassificationSequenceParser::Model& model) {
                return self.build(nnInput, model);
            },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, ClassificationSequenceParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&ClassificationSequenceParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, ClassificationSequenceParser, build, 2))
        .def(
            "setNNArchive", &ClassificationSequenceParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, ClassificationSequenceParser, setNNArchive))
        .def("setNNArchiveHead",
             &ClassificationSequenceParser::setNNArchiveHead,
             py::arg("head"),
             DOC(dai, beta, node, ClassificationSequenceParser, setNNArchiveHead))
        .def("setOutputLayerName",
             &ClassificationSequenceParser::setOutputLayerName,
             py::arg("outputLayerName"),
             DOC(dai, beta, node, ClassificationSequenceParser, setOutputLayerName))
        .def("getOutputLayerName", &ClassificationSequenceParser::getOutputLayerName, DOC(dai, beta, node, ClassificationSequenceParser, getOutputLayerName))
        .def("setClasses", &ClassificationSequenceParser::setClasses, py::arg("classes"), DOC(dai, beta, node, ClassificationSequenceParser, setClasses))
        .def("getClasses", &ClassificationSequenceParser::getClasses, DOC(dai, beta, node, ClassificationSequenceParser, getClasses))
        .def("setSoftmax", &ClassificationSequenceParser::setSoftmax, py::arg("isSoftmax"), DOC(dai, beta, node, ClassificationSequenceParser, setSoftmax))
        .def("getSoftmax", &ClassificationSequenceParser::getSoftmax, DOC(dai, beta, node, ClassificationSequenceParser, getSoftmax))
        .def("setIgnoredIndexes",
             &ClassificationSequenceParser::setIgnoredIndexes,
             py::arg("ignoredIndexes"),
             DOC(dai, beta, node, ClassificationSequenceParser, setIgnoredIndexes))
        .def("getIgnoredIndexes", &ClassificationSequenceParser::getIgnoredIndexes, DOC(dai, beta, node, ClassificationSequenceParser, getIgnoredIndexes))
        .def("setRemoveDuplicates",
             &ClassificationSequenceParser::setRemoveDuplicates,
             py::arg("removeDuplicates"),
             DOC(dai, beta, node, ClassificationSequenceParser, setRemoveDuplicates))
        .def("getRemoveDuplicates", &ClassificationSequenceParser::getRemoveDuplicates, DOC(dai, beta, node, ClassificationSequenceParser, getRemoveDuplicates))
        .def("setConcatenateClasses",
             &ClassificationSequenceParser::setConcatenateClasses,
             py::arg("concatenateClasses"),
             DOC(dai, beta, node, ClassificationSequenceParser, setConcatenateClasses))
        .def("getConcatenateClasses",
             &ClassificationSequenceParser::getConcatenateClasses,
             DOC(dai, beta, node, ClassificationSequenceParser, getConcatenateClasses))
        .def(
            "setRunOnHost", &ClassificationSequenceParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, ClassificationSequenceParser, setRunOnHost))
        .def("runOnHost", &ClassificationSequenceParser::runOnHost, DOC(dai, beta, node, ClassificationSequenceParser, runOnHost));

    classificationSequenceParser.attr("Properties") = properties;
}
