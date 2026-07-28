#include <pybind11/stl.h>

#include "depthai/beta/node/FastSAMParser.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_fastsamparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto fastsamParser = ADD_BETA_NODE_DERIVED(FastSAMParser, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    fastsamParser.def_readonly("input", &FastSAMParser::input, DOC(dai, beta, node, FastSAMParser, input))
        .def_readonly("out", &FastSAMParser::out, DOC(dai, beta, node, FastSAMParser, out))
        .def(
            "build",
            [](FastSAMParser& self, Node::Output& nnInput, const FastSAMParser::Model& model) { return self.build(nnInput, model); },
            py::arg("input"),
            py::arg("model"),
            DOC(dai, beta, node, FastSAMParser, build))
        .def("build",
             py::overload_cast<Node::Output&, const dai::nn_archive::v1::Head&>(&FastSAMParser::build),
             py::arg("input"),
             py::arg("head"),
             DOC(dai, beta, node, FastSAMParser, build, 2))
        .def("setNNArchive", &FastSAMParser::setNNArchive, py::arg("nnArchive"), DOC(dai, beta, node, FastSAMParser, setNNArchive))
        .def("setNNArchiveHead", &FastSAMParser::setNNArchiveHead, py::arg("head"), DOC(dai, beta, node, FastSAMParser, setNNArchiveHead))
        .def(
            "setConfidenceThreshold", &FastSAMParser::setConfidenceThreshold, py::arg("threshold"), DOC(dai, beta, node, FastSAMParser, setConfidenceThreshold))
        .def("getConfidenceThreshold", &FastSAMParser::getConfidenceThreshold, DOC(dai, beta, node, FastSAMParser, getConfidenceThreshold))
        .def("setNumClasses", &FastSAMParser::setNumClasses, py::arg("numClasses"), DOC(dai, beta, node, FastSAMParser, setNumClasses))
        .def("getNumClasses", &FastSAMParser::getNumClasses, DOC(dai, beta, node, FastSAMParser, getNumClasses))
        .def("setIouThreshold", &FastSAMParser::setIouThreshold, py::arg("iouThreshold"), DOC(dai, beta, node, FastSAMParser, setIouThreshold))
        .def("getIouThreshold", &FastSAMParser::getIouThreshold, DOC(dai, beta, node, FastSAMParser, getIouThreshold))
        .def("setMaskConfidence", &FastSAMParser::setMaskConfidence, py::arg("maskConfidence"), DOC(dai, beta, node, FastSAMParser, setMaskConfidence))
        .def("getMaskConfidence", &FastSAMParser::getMaskConfidence, DOC(dai, beta, node, FastSAMParser, getMaskConfidence))
        .def("setPrompt", &FastSAMParser::setPrompt, py::arg("prompt"), DOC(dai, beta, node, FastSAMParser, setPrompt))
        .def("getPrompt", &FastSAMParser::getPrompt, DOC(dai, beta, node, FastSAMParser, getPrompt))
        .def("setPoints", &FastSAMParser::setPoints, py::arg("x"), py::arg("y"), DOC(dai, beta, node, FastSAMParser, setPoints))
        .def("getPoints", &FastSAMParser::getPoints, DOC(dai, beta, node, FastSAMParser, getPoints))
        .def("setPointLabel", &FastSAMParser::setPointLabel, py::arg("pointLabel"), DOC(dai, beta, node, FastSAMParser, setPointLabel))
        .def("getPointLabel", &FastSAMParser::getPointLabel, DOC(dai, beta, node, FastSAMParser, getPointLabel))
        .def("setBoundingBox", &FastSAMParser::setBoundingBox, py::arg("bbox"), DOC(dai, beta, node, FastSAMParser, setBoundingBox))
        .def("getBoundingBox", &FastSAMParser::getBoundingBox, DOC(dai, beta, node, FastSAMParser, getBoundingBox))
        .def("setYoloOutputs", &FastSAMParser::setYoloOutputs, py::arg("yoloOutputs"), DOC(dai, beta, node, FastSAMParser, setYoloOutputs))
        .def("getYoloOutputs", &FastSAMParser::getYoloOutputs, DOC(dai, beta, node, FastSAMParser, getYoloOutputs))
        .def("setMaskOutputs", &FastSAMParser::setMaskOutputs, py::arg("maskOutputs"), DOC(dai, beta, node, FastSAMParser, setMaskOutputs))
        .def("getMaskOutputs", &FastSAMParser::getMaskOutputs, DOC(dai, beta, node, FastSAMParser, getMaskOutputs))
        .def("setProtosOutput", &FastSAMParser::setProtosOutput, py::arg("protosOutput"), DOC(dai, beta, node, FastSAMParser, setProtosOutput))
        .def("getProtosOutput", &FastSAMParser::getProtosOutput, DOC(dai, beta, node, FastSAMParser, getProtosOutput));
}
