#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/rtabmap/RTABMapVIO.hpp"

#include <pybind11/eval.h>

extern py::handle daiNodeModule;

void bind_rtabmapvionode(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto rtabmapVIONode = ADD_NODE(RTABMapVIO);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // RTABMapVIO Node
    rtabmapVIONode
        .def_readonly("rect", &RTABMapVIO::rect, DOC(dai, node, RTABMapVIO, rect))
        .def_readonly("depth", &RTABMapVIO::depth, DOC(dai, node, RTABMapVIO, depth))
        .def_readonly("imu", &RTABMapVIO::imu, DOC(dai, node, RTABMapVIO, imu))
        .def_readonly("features", &RTABMapVIO::features, DOC(dai, node, RTABMapVIO, features))
        .def_readonly("transform", &RTABMapVIO::transform, DOC(dai, node, RTABMapVIO, transform))
        .def_readonly("passthrough_rect", &RTABMapVIO::passthrough_rect, DOC(dai, node, RTABMapVIO, passthrough_rect))
        .def_readonly("passthrough_depth", &RTABMapVIO::passthrough_depth, DOC(dai, node, RTABMapVIO, passthrough_depth))
        .def_readonly("passthrough_features", &RTABMapVIO::passthrough_features, DOC(dai, node, RTABMapVIO, passthrough_features))
        .def("build", &RTABMapVIO::build)
        .def("setParams", &RTABMapVIO::setParams, py::arg("params"), DOC(dai, node, RTABMapVIO, setParams))
        .def("setUseFeatures", &RTABMapVIO::setUseFeatures, py::arg("useFeatures"), DOC(dai, node, RTABMapVIO, setUseFeatures))
        .def("setLocalTransform", &RTABMapVIO::setLocalTransform, py::arg("transform"), DOC(dai, node, RTABMapVIO, setLocalTransform));
        .def("reset", &RTABMapVIO::reset, py::args("transform"), DOC(dai, node, RTABMapVIO, reset));
}
