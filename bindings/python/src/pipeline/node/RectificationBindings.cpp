
#include <pybind11/eval.h>

#include "Common.hpp"
#include "depthai/pipeline/node/Rectification.hpp"

void bind_rectification(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<RectificationProperties> RectificationProperties(m, "RectificationProperties", DOC(dai, RectificationProperties));

    auto rectification = ADD_NODE(Rectification);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    RectificationProperties.def(py::init<>());

    RectificationProperties
        .def_readwrite("enableRectification", &dai::RectificationProperties::enableRectification, DOC(dai, RectificationProperties, enableRectification))
        .def_readwrite("outputWidth", &RectificationProperties::outputWidth, DOC(dai, RectificationProperties, outputWidth))
        .def_readwrite("outputHeight", &RectificationProperties::outputHeight, DOC(dai, RectificationProperties, outputHeight));

    rectification.def_readonly("passthrough1", &Rectification::passthrough1, DOC(dai, node, Rectification, passthrough1))
        .def_readonly("passthrough2", &Rectification::passthrough2, DOC(dai, node, Rectification, passthrough2))
        .def_readonly("output1", &Rectification::output1, DOC(dai, node, Rectification, output1))
        .def_readonly("output2", &Rectification::output2, DOC(dai, node, Rectification, output2))
        .def_readonly("input1", &Rectification::input1, DOC(dai, node, Rectification, input1))
        .def_readonly("input2", &Rectification::input2, DOC(dai, node, Rectification, input2))
        .def("setRunOnHost", &Rectification::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, Rectification, setRunOnHost))
        .def("enableRectification", &Rectification::enableRectification, py::arg("enable"), DOC(dai, node, Rectification, enableRectification))
        .def("setOutputSize",
             (Rectification & (Rectification::*)(uint32_t, uint32_t)) & Rectification::setOutputSize,
             py::arg("width"),
             py::arg("height"),
             DOC(dai, node, Rectification, setOutputSize))
        .def("setOutputSize",
             (Rectification & (Rectification::*)(std::pair<uint32_t, uint32_t>)) & Rectification::setOutputSize,
             py::arg("size"),
             DOC(dai, node, Rectification, setOutputSize, 2));

    daiNodeModule.attr("Rectification").attr("Properties") = RectificationProperties;
}