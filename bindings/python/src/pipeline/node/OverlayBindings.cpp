#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Overlay.hpp"
#include "depthai/properties/OverlayProperties.hpp"

void bind_overlay(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<OverlayProperties> overrlayProperties(m, "OverlayProperties", DOC(dai, OverlayProperties));
    auto overrlayProperties = ADD_NODE(Overlay);

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

    overrlayProperties.def_readwrite("alpha", &OverlayProperties::alpha);
    overrlayProperties.def_readwrite("interpolationType", &OverlayProperties::interpolationType);

    // Node
    overrlayProperties.def_readonly("inputFrame", &Overlay::inputFrame, DOC(dai, node, Overlay, inputFrame))
        .def_readonly("inputDetections", &Overlay::inputDetections, DOC(dai, node, Overlay, inputDetections))
        .def_readonly("out", &Overlay::out, DOC(dai, node, Overlay, out))
        .def("setOverlayAlpha", &Overlay::setOverlayAlpha, py::arg("alpha") = 0.5f, DOC(dai, node, Overlay, setOverlayAlpha))
        .def("setInterpolationType", &Overlay::setInterpolationType, py::arg("interpolationType") = 1, DOC(dai, node, Overlay, setInterpolationType))
        // ALIAS
        daiNodeModule.attr("Overlay")
        .attr("Properties") = overrlayProperties;
}
