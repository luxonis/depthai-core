#include <pybind11/stl.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/CoordinateFrameTransform.hpp"

extern py::handle daiNodeModule;

void bind_coordinateframetransform(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto coordinateFrameTransformNode = ADD_NODE_DERIVED(CoordinateFrameTransform, ThreadedHostNode);

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

    coordinateFrameTransformNode.def_readonly("inputs", &CoordinateFrameTransform::inputs, DOC(dai, node, CoordinateFrameTransform, inputs))
        .def_readonly("outputs", &CoordinateFrameTransform::outputs, DOC(dai, node, CoordinateFrameTransform, outputs))
        .def("build",
             py::overload_cast<const std::vector<Node::Output*>&, const CoordinateFrame&>(&CoordinateFrameTransform::build),
             py::arg("sources"),
             py::arg("target"),
             DOC(dai, node, CoordinateFrameTransform, build))
        .def("build",
             py::overload_cast<size_t, const CoordinateFrame&>(&CoordinateFrameTransform::build),
             py::arg("numInputs"),
             py::arg("target"),
             DOC(dai, node, CoordinateFrameTransform, build, 2))
        .def("getNumInputs", &CoordinateFrameTransform::getNumInputs, DOC(dai, node, CoordinateFrameTransform, getNumInputs))
        .def("setTarget", &CoordinateFrameTransform::setTarget, py::arg("target"), DOC(dai, node, CoordinateFrameTransform, setTarget))
        .def("getTarget", &CoordinateFrameTransform::getTarget, DOC(dai, node, CoordinateFrameTransform, getTarget))
        .def("setSourceFrame",
             &CoordinateFrameTransform::setSourceFrame,
             py::arg("inputIndex"),
             py::arg("frame"),
             DOC(dai, node, CoordinateFrameTransform, setSourceFrame))
        .def("setCalibration", &CoordinateFrameTransform::setCalibration, py::arg("calibration"), DOC(dai, node, CoordinateFrameTransform, setCalibration));
}
