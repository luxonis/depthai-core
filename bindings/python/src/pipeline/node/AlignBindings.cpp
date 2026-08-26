#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Align.hpp"

void bind_align(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

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

    align.def_readonly("initialConfig", &Align::initialConfig, DOC(dai, node, Align, initialConfig))
        .def_readonly("inputConfig", &Align::inputConfig, DOC(dai, node, Align, inputConfig))
        .def_readonly("input", &Align::input, DOC(dai, node, Align, input))
        .def_readonly("inputAlignTo", &Align::inputAlignTo, DOC(dai, node, Align, inputAlignTo))
        .def_readonly("passthroughInput", &Align::passthroughInput, DOC(dai, node, Align, passthroughInput))
        .def_readonly("outputAligned", &Align::outputAligned, DOC(dai, node, Align, outputAligned))
        .def("setOutputSize", &Align::setOutputSize, py::arg("alignWidth"), py::arg("alignHeight"), DOC(dai, node, Align, setOutputSize))
        .def("setInterpolation", &Align::setInterpolation, py::arg("interp"), DOC(dai, node, Align, setInterpolation))
        .def("setNumShaves", &Align::setNumShaves, py::arg("numShaves"), DOC(dai, node, Align, setNumShaves))
        .def("setNumFramesPool", &Align::setNumFramesPool, py::arg("numFramesPool"), DOC(dai, node, Align, setNumFramesPool))
        .def("setRunOnHost", &Align::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, Align, setRunOnHost))
        .def("runOnHost", &Align::runOnHost, DOC(dai, node, Align, runOnHost));

    daiNodeModule.attr("Align").attr("Properties") = m.attr("ImageAlignProperties");
}
