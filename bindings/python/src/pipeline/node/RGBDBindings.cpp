
#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"

extern py::handle daiNodeModule;

void bind_rgbd(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto rgbdNode = ADD_NODE_DERIVED(RGBD, ThreadedHostNode);

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

    // RGBD Node
    rgbdNode
        .def_property_readonly(
            "inColor", [](RGBD& node) { return &node.inColor; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "inDepth", [](RGBD& node) { return &node.inDepth; }, py::return_value_policy::reference_internal)
        .def_readonly("pcl", &RGBD::pcl, DOC(dai, node, RGBD, pcl))
        .def("build", static_cast<std::shared_ptr<RGBD> (RGBD::*)()>(&RGBD::build))
        .def("build",
             static_cast<std::shared_ptr<RGBD> (RGBD::*)(bool, std::pair<int, int>)>(&RGBD::build),
             py::arg("autocreate"),
             py::arg("size"),
             DOC(dai, node, RGBD, build, 2))
        .def("setOutputMeters", &RGBD::setOutputMeters, py::arg("outputMeters"), DOC(dai, node, RGBD, setOutputMeters))
        .def("useCPU", &RGBD::useCPU, DOC(dai, node, RGBD, useCPU))
        .def("useCPUMT", &RGBD::useCPUMT, py::arg("numThreads") = 2, DOC(dai, node, RGBD, useCPUMT))
        .def("useGPU", &RGBD::useGPU, py::arg("device") = 0, DOC(dai, node, RGBD, useGPU))
        .def("printDevices", &RGBD::printDevices, DOC(dai, node, RGBD, printDevices));
}