#include <pybind11/stl.h>

#include "depthai/beta/node/XFeatMonoParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_xfeatmonoparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::XFeatMonoParserProperties> properties(betaModule, "XFeatMonoParserProperties");
    auto xfeatMonoParser = ADD_BETA_NODE_DERIVED(XFeatMonoParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    properties.def_readwrite("initialConfig", &beta::XFeatMonoParserProperties::initialConfig)
        .def_readwrite("outputLayerFeats", &beta::XFeatMonoParserProperties::outputLayerFeats)
        .def_readwrite("outputLayerKeypoints", &beta::XFeatMonoParserProperties::outputLayerKeypoints)
        .def_readwrite("outputLayerHeatmaps", &beta::XFeatMonoParserProperties::outputLayerHeatmaps)
        .def_readwrite("originalSize", &beta::XFeatMonoParserProperties::originalSize)
        .def_readwrite("inputSize", &beta::XFeatMonoParserProperties::inputSize);

    xfeatMonoParser.def_readonly("inputConfig", &XFeatMonoParser::inputConfig, DOC(dai, beta, node, XFeatMonoParser, inputConfig))
        .def_readonly("initialConfig", &XFeatMonoParser::initialConfig, DOC(dai, beta, node, XFeatMonoParser, initialConfig))
        .def_readonly("input", &XFeatMonoParser::input, DOC(dai, beta, node, XFeatMonoParser, input))
        .def_readonly("out", &XFeatMonoParser::out, DOC(dai, beta, node, XFeatMonoParser, out))
        .def(
            "build",
            [](XFeatMonoParser& self, Node::Output& nnInput, const XFeatMonoParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, XFeatMonoParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&XFeatMonoParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, XFeatMonoParser, build, 2))
        .def("setNNArchive", &XFeatMonoParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, XFeatMonoParser, setNNArchive))
        .def("setNNArchiveHead", &XFeatMonoParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, XFeatMonoParser, setNNArchiveHead))
        .def("setOutputLayerFeats",
             &XFeatMonoParser::setOutputLayerFeats,
             py::arg("outputLayerFeats"),
             DOC(dai, beta, node, XFeatMonoParser, setOutputLayerFeats))
        .def("getOutputLayerFeats", &XFeatMonoParser::getOutputLayerFeats, DOC(dai, beta, node, XFeatMonoParser, getOutputLayerFeats))
        .def("setOutputLayerKeypoints",
             &XFeatMonoParser::setOutputLayerKeypoints,
             py::arg("outputLayerKeypoints"),
             DOC(dai, beta, node, XFeatMonoParser, setOutputLayerKeypoints))
        .def("getOutputLayerKeypoints", &XFeatMonoParser::getOutputLayerKeypoints, DOC(dai, beta, node, XFeatMonoParser, getOutputLayerKeypoints))
        .def("setOutputLayerHeatmaps",
             &XFeatMonoParser::setOutputLayerHeatmaps,
             py::arg("outputLayerHeatmaps"),
             DOC(dai, beta, node, XFeatMonoParser, setOutputLayerHeatmaps))
        .def("getOutputLayerHeatmaps", &XFeatMonoParser::getOutputLayerHeatmaps, DOC(dai, beta, node, XFeatMonoParser, getOutputLayerHeatmaps))
        .def("setOriginalSize", &XFeatMonoParser::setOriginalSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, XFeatMonoParser, setOriginalSize))
        .def("getOriginalSize", &XFeatMonoParser::getOriginalSize, DOC(dai, beta, node, XFeatMonoParser, getOriginalSize))
        .def("setInputSize", &XFeatMonoParser::setInputSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, XFeatMonoParser, setInputSize))
        .def("getInputSize", &XFeatMonoParser::getInputSize, DOC(dai, beta, node, XFeatMonoParser, getInputSize))
        .def("setMaxKeypoints", &XFeatMonoParser::setMaxKeypoints, py::arg("maxKeypoints"), DOC(dai, beta, node, XFeatMonoParser, setMaxKeypoints))
        .def("getMaxKeypoints", &XFeatMonoParser::getMaxKeypoints, DOC(dai, beta, node, XFeatMonoParser, getMaxKeypoints))
        .def("setTrigger", &XFeatMonoParser::setTrigger, DOC(dai, beta, node, XFeatMonoParser, setTrigger))
        .def("setRunOnHost", &XFeatMonoParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, XFeatMonoParser, setRunOnHost))
        .def("runOnHost", &XFeatMonoParser::runOnHost, DOC(dai, beta, node, XFeatMonoParser, runOnHost));

    xfeatMonoParser.attr("Properties") = properties;
}
