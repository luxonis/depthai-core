#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Align.hpp"

void bind_align(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    py::class_<AlignProperties> alignProperties(m, "AlignProperties", DOC(dai, AlignProperties));
    auto align = ADD_NODE(Align);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    alignProperties.def_readwrite("initialConfig", &AlignProperties::initialConfig, DOC(dai, AlignProperties, initialConfig))
        .def_readwrite("numFramesPool", &AlignProperties::numFramesPool, DOC(dai, AlignProperties, numFramesPool))
        .def_readwrite("interpolation", &AlignProperties::interpolation, DOC(dai, AlignProperties, interpolation));

    align.def_readonly("initialConfig", &Align::initialConfig, DOC(dai, node, Align, initialConfig))
        .def_readonly("inputConfig", &Align::inputConfig, DOC(dai, node, Align, inputConfig))
        .def_readonly("input", &Align::input, DOC(dai, node, Align, input))
        .def_readonly("inputAlignTo", &Align::inputAlignTo, DOC(dai, node, Align, inputAlignTo))
        .def_readonly("passthroughInput", &Align::passthroughInput, DOC(dai, node, Align, passthroughInput))
        .def_readonly("outputAligned", &Align::outputAligned, DOC(dai, node, Align, outputAligned))
        .def("setNumFramesPool", &Align::setNumFramesPool, py::arg("numFramesPool"), DOC(dai, node, Align, setNumFramesPool))
        .def("setInterpolation", &Align::setInterpolation, py::arg("interpolation"), DOC(dai, node, Align, setInterpolation))
        .def("setRunOnHost", &Align::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, Align, setRunOnHost))
        .def("runOnHost", &Align::runOnHost, DOC(dai, node, Align, runOnHost));

    daiNodeModule.attr("Align").attr("Properties") = alignProperties;
}
