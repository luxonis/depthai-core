#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SegmentationParser.hpp"

void bind_segmentationparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<SegmentationParserProperties> segmentationParserProperties(m, "SegmentationParserProperties", DOC(dai, SegmentationParserProperties));
    auto segmentationParser = ADD_NODE(SegmentationParser);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    segmentationParserProperties.def_readwrite("labels", &SegmentationParserProperties::labels, DOC(dai, SegmentationParserProperties, labels))
        .def_readwrite("networkOutputName", &SegmentationParserProperties::networkOutputName, DOC(dai, SegmentationParserProperties, networkOutputName))
        .def_readwrite("classesInOneLayer", &SegmentationParserProperties::classesInOneLayer, DOC(dai, SegmentationParserProperties, classesInOneLayer))
        .def_readwrite("backgroundClass", &SegmentationParserProperties::backgroundClass, DOC(dai, SegmentationParserProperties, backgroundClass));

    segmentationParser.def_readonly("input", &SegmentationParser::input, DOC(dai, node, SegmentationParser, input))
        .def_readonly("inputConfig", &SegmentationParser::inputConfig, DOC(dai, node, SegmentationParser, inputConfig))
        .def_readonly("out", &SegmentationParser::out, DOC(dai, node, SegmentationParser, out))
        .def_readonly("initialConfig", &SegmentationParser::initialConfig, DOC(dai, node, SegmentationParser, initialConfig))
        .def(
            "build",
            [](SegmentationParser& self, Node::Output& input, const SegmentationParser::Model& model) { return self.build(input, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, node, SegmentationParser, build))
        .def(
            "build",
            [](SegmentationParser& self, Node::Output& input, const NNArchive& nnArchive) { return self.build(input, SegmentationParser::Model{nnArchive}); },
            py::arg("input"),
            py::arg("nnArchive"),
            DOC(dai, node, SegmentationParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&SegmentationParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, node, SegmentationParser, build, 2))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&>(&SegmentationParser::setNNArchive),
             py::arg("nnArchive"),
             DOC(dai, node, SegmentationParser, setNNArchive))
        .def("setNNArchiveHead", &SegmentationParser::setNNArchiveHead, py::arg("head"), DOC(dai, node, SegmentationParser, setNNArchiveHead))
        .def("setLabels", &SegmentationParser::setLabels, py::arg("labels"), DOC(dai, node, SegmentationParser, setLabels))
        .def("getLabels", &SegmentationParser::getLabels, DOC(dai, node, SegmentationParser, getLabels))
        .def("setBackgroundClass", &SegmentationParser::setBackgroundClass, py::arg("backgroundClass"), DOC(dai, node, SegmentationParser, setBackgroundClass))
        .def("getBackgroundClass", &SegmentationParser::getBackgroundClass, DOC(dai, node, SegmentationParser, getBackgroundClass))
        .def("setRunOnHost", &SegmentationParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, SegmentationParser, setRunOnHost))
        .def("runOnHost", &SegmentationParser::runOnHost, DOC(dai, node, SegmentationParser, runOnHost));

    daiNodeModule.attr("SegmentationParser").attr("Properties") = segmentationParserProperties;
}
