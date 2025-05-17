#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/rtabmap/RTABMapVIO.hpp"

extern py::handle daiNodeModule;

void bind_rtabmapvionode(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto rtabmapVIONode = ADD_NODE_DERIVED(RTABMapVIO, ThreadedHostNode);

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

    // RTABMapVIO Node
    rtabmapVIONode.def_property_readonly(
                      "rect", [](RTABMapVIO& node) { return &node.rect; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "depth", [](RTABMapVIO& node) { return &node.depth; }, py::return_value_policy::reference_internal)
        .def_readonly("imu", &RTABMapVIO::imu, DOC(dai, node, RTABMapVIO, imu))
        .def_readonly("features", &RTABMapVIO::features, DOC(dai, node, RTABMapVIO, features))
        .def_readonly("transform", &RTABMapVIO::transform, DOC(dai, node, RTABMapVIO, transform))
        .def_readonly("passthroughRect", &RTABMapVIO::passthroughRect, DOC(dai, node, RTABMapVIO, passthroughRect))
        .def_readonly("passthroughDepth", &RTABMapVIO::passthroughDepth, DOC(dai, node, RTABMapVIO, passthroughDepth))
        .def_readonly("passthroughFeatures", &RTABMapVIO::passthroughFeatures, DOC(dai, node, RTABMapVIO, passthroughFeatures))
        .def("setParams", &RTABMapVIO::setParams, py::arg("params"), DOC(dai, node, RTABMapVIO, setParams))
        .def("setUseFeatures", &RTABMapVIO::setUseFeatures, py::arg("useFeatures"), DOC(dai, node, RTABMapVIO, setUseFeatures))
        .def("setLocalTransform", &RTABMapVIO::setLocalTransform, py::arg("transform"), DOC(dai, node, RTABMapVIO, setLocalTransform))
        .def("reset", &RTABMapVIO::reset, py::arg("transform"), DOC(dai, node, RTABMapVIO, reset));
}
