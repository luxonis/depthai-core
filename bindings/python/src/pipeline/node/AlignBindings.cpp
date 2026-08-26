#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Align.hpp"

void bind_align(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Node and Properties declare upfront
    py::class_<AlignProperties> alignProperties(m, "AlignProperties", DOC(dai, AlignProperties));
    auto align = ADD_NODE(Align);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Properties
    alignProperties.def_readwrite("initialConfig", &AlignProperties::initialConfig, DOC(dai, AlignProperties, initialConfig))
        .def_readwrite("numFramesPool", &AlignProperties::numFramesPool, DOC(dai, AlignProperties, numFramesPool))
        .def_readwrite("alignWidth", &AlignProperties::alignWidth, DOC(dai, AlignProperties, alignWidth))
        .def_readwrite("alignHeight", &AlignProperties::alignHeight, DOC(dai, AlignProperties, alignHeight));

    // Node
    align.def_readonly("initialConfig", &Align::initialConfig, DOC(dai, node, Align, initialConfig))
        .def_readonly("inputConfig", &Align::inputConfig, DOC(dai, node, Align, inputConfig))
        .def_readonly("input", &Align::input, DOC(dai, node, Align, input))
        .def_readonly("inputAlignTo", &Align::inputAlignTo, DOC(dai, node, Align, inputAlignTo))
        .def_readonly("passthroughInput", &Align::passthroughInput, DOC(dai, node, Align, passthroughInput))
        .def_readonly("outputAligned", &Align::outputAligned, DOC(dai, node, Align, outputAligned))
        .def("setOutputSize", &Align::setOutputSize, py::arg("alignWidth"), py::arg("alignHeight"), DOC(dai, node, Align, setOutputSize))
        .def("setNumFramesPool", &Align::setNumFramesPool, py::arg("numFramesPool"), DOC(dai, node, Align, setNumFramesPool))
        .def("setRunOnHost", &Align::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, Align, setRunOnHost))
        .def("runOnHost", &Align::runOnHost, DOC(dai, node, Align, runOnHost));

    // ALIAS
    daiNodeModule.attr("Align").attr("Properties") = alignProperties;
}
