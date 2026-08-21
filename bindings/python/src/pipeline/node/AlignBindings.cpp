#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Align.hpp"

void bind_align(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    auto align = ADD_NODE_DOC(Align, "Aligns ImgFrame and Transformable messages using ImgTransformation metadata.");

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

    align.def_readonly("initialConfig", &Align::initialConfig, "Initial config")
        .def_readonly("inputConfig", &Align::inputConfig, "Input message with ability to modify parameters in runtime")
        .def_readonly("input", &Align::input, "Input message to align")
        .def_readonly("inputAlignTo", &Align::inputAlignTo, "Input message to align to")
        .def_readonly("passthroughInput", &Align::passthroughInput, "Passthrough input message")
        .def_readonly("outputAligned", &Align::outputAligned, "Aligned output message")
        .def("setOutputSize", &Align::setOutputSize, py::arg("alignWidth"), py::arg("alignHeight"))
        .def("setInterpolation", &Align::setInterpolation, py::arg("interp"))
        .def("setNumShaves", &Align::setNumShaves, py::arg("numShaves"))
        .def("setNumFramesPool", &Align::setNumFramesPool, py::arg("numFramesPool"))
        .def("setRunOnHost", &Align::setRunOnHost, py::arg("runOnHost"))
        .def("runOnHost", &Align::runOnHost);

    daiNodeModule.attr("Align").attr("Properties") = m.attr("ImageAlignProperties");
}
