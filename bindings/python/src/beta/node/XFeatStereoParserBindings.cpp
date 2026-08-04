#include <pybind11/stl.h>

#include "depthai/beta/node/XFeatStereoParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_xfeatstereoparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto xfeatStereoParser = ADD_BETA_NODE_DERIVED(XFeatStereoParser, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    xfeatStereoParser.def_readonly("referenceInput", &XFeatStereoParser::referenceInput, DOC(dai, beta, node, XFeatStereoParser, referenceInput))
        .def_readonly("targetInput", &XFeatStereoParser::targetInput, DOC(dai, beta, node, XFeatStereoParser, targetInput))
        .def_readonly("out", &XFeatStereoParser::out, DOC(dai, beta, node, XFeatStereoParser, out))
        .def(
            "build",
            [](XFeatStereoParser& self, Node::Output& reference, Node::Output& target, const XFeatStereoParser::Model& model) {
                return self.build(reference, target, model);
            },
            py::arg("reference"),
            py::arg("target"),
            py::arg("model"),
            DOC(dai, beta, node, XFeatStereoParser, build))
        .def("build",
             py::overload_cast<Node::Output&, Node::Output&, const dai::nn_archive::v1::Head&>(&XFeatStereoParser::build),
             py::arg("reference"),
             py::arg("target"),
             py::arg("head"),
             DOC(dai, beta, node, XFeatStereoParser, build, 2))
        .def("setNNArchive", &XFeatStereoParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, XFeatStereoParser, setNNArchive))
        .def("setNNArchiveHead", &XFeatStereoParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, XFeatStereoParser, setNNArchiveHead))
        .def("setOutputLayerFeats",
             &XFeatStereoParser::setOutputLayerFeats,
             py::arg("outputLayerFeats"),
             DOC(dai, beta, node, XFeatStereoParser, setOutputLayerFeats))
        .def("getOutputLayerFeats", &XFeatStereoParser::getOutputLayerFeats, DOC(dai, beta, node, XFeatStereoParser, getOutputLayerFeats))
        .def("setOutputLayerKeypoints",
             &XFeatStereoParser::setOutputLayerKeypoints,
             py::arg("outputLayerKeypoints"),
             DOC(dai, beta, node, XFeatStereoParser, setOutputLayerKeypoints))
        .def("getOutputLayerKeypoints", &XFeatStereoParser::getOutputLayerKeypoints, DOC(dai, beta, node, XFeatStereoParser, getOutputLayerKeypoints))
        .def("setOutputLayerHeatmaps",
             &XFeatStereoParser::setOutputLayerHeatmaps,
             py::arg("outputLayerHeatmaps"),
             DOC(dai, beta, node, XFeatStereoParser, setOutputLayerHeatmaps))
        .def("getOutputLayerHeatmaps", &XFeatStereoParser::getOutputLayerHeatmaps, DOC(dai, beta, node, XFeatStereoParser, getOutputLayerHeatmaps))
        .def("setOriginalSize",
             &XFeatStereoParser::setOriginalSize,
             py::arg("width"),
             py::arg("height"),
             DOC(dai, beta, node, XFeatStereoParser, setOriginalSize))
        .def("getOriginalSize", &XFeatStereoParser::getOriginalSize, DOC(dai, beta, node, XFeatStereoParser, getOriginalSize))
        .def("setInputSize", &XFeatStereoParser::setInputSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, XFeatStereoParser, setInputSize))
        .def("getInputSize", &XFeatStereoParser::getInputSize, DOC(dai, beta, node, XFeatStereoParser, getInputSize))
        .def("setMaxKeypoints", &XFeatStereoParser::setMaxKeypoints, py::arg("maxKeypoints"), DOC(dai, beta, node, XFeatStereoParser, setMaxKeypoints))
        .def("getMaxKeypoints", &XFeatStereoParser::getMaxKeypoints, DOC(dai, beta, node, XFeatStereoParser, getMaxKeypoints))
        .def("setRunOnHost", &XFeatStereoParser::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, XFeatStereoParser, setRunOnHost))
        .def("runOnHost", &XFeatStereoParser::runOnHost, DOC(dai, beta, node, XFeatStereoParser, runOnHost));
}
